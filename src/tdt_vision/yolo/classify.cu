#include "infer.hpp"
#include "classify.hpp"

namespace classify{
    using namespace std;
    #define GPU_BLOCK_THREADS 512
    #define checkRuntime(call)                                              \
        do {                                                                 \
            auto ___call__ret_code__ = (call);                                \
            if (___call__ret_code__ != cudaSuccess) {                          \
            INFO("CUDA Runtime error💥 %s # %s, code = %s [ %d ]", #call,       \
                cudaGetErrorString(___call__ret_code__),                         \
                cudaGetErrorName(___call__ret_code__), ___call__ret_code__);      \
            abort();                                                               \
            }                                                                       \
        } while (0)
    inline int upbound(int n, int align = 32) {
        return (n + align - 1) / align * align;
    }

    #define checkKernel(...)                 \
    do {                                      \
        { (__VA_ARGS__); }                     \
        checkRuntime(cudaPeekAtLastError());    \
    } while (0)

    // static dim3 grid_dims(int numJobs) {
    //     int numBlockThreads =
    //         numJobs < GPU_BLOCK_THREADS ? numJobs : GPU_BLOCK_THREADS;
    //     return dim3(((numJobs + numBlockThreads - 1) / (float)numBlockThreads));
    // }

    // static dim3 block_dims(int numJobs) {
    //     return numJobs < GPU_BLOCK_THREADS ? numJobs : GPU_BLOCK_THREADS;
    // }

    struct AffineMatrix {
    float i2d[6];  // image to dst(network), 2x3 matrix
    float d2i[6];  // dst to image, 2x3 matrix

    void compute(const std::tuple<int, int> &from,
                const std::tuple<int, int> &to) {
        float scale_x = get<0>(to) / (float)get<0>(from);
        float scale_y = get<1>(to) / (float)get<1>(from);
        float scale = std::min(scale_x, scale_y);
        i2d[0] = scale;
        i2d[1] = 0;
        i2d[2] = -scale * get<0>(from) * 0.5 + get<0>(to) * 0.5 + scale * 0.5 - 0.5;
        i2d[3] = 0;
        i2d[4] = scale;
        i2d[5] = -scale * get<1>(from) * 0.5 + get<1>(to) * 0.5 + scale * 0.5 - 0.5;

        double D = i2d[0] * i2d[4] - i2d[1] * i2d[3];
        D = D != 0. ? double(1.) / D : double(0.);
        double A11 = i2d[4] * D, A22 = i2d[0] * D, A12 = -i2d[1] * D,
            A21 = -i2d[3] * D;
        double b1 = -A11 * i2d[2] - A12 * i2d[5];
        double b2 = -A21 * i2d[2] - A22 * i2d[5];

        d2i[0] = A11;
        d2i[1] = A12;
        d2i[2] = b1;
        d2i[3] = A21;
        d2i[4] = A22;
        d2i[5] = b2;
    }
    };
    
    enum class NormType : int { None = 0, MeanStd = 1, AlphaBeta = 2 };

    enum class ChannelType : int { None = 0, SwapRB = 1 };

    int postprocess(vector<float> &output_array) {
        int max_index = 0;
        float max_value = output_array[0];
        for (int i = 1; i < output_array.size(); i++) {
            if (output_array[i] > max_value) {
                max_value = output_array[i];
                max_index = i;
            }
        }
        return max_index;
    }

    struct Norm {
        float mean[3];
        float std[3];
        float alpha, beta;
        NormType type = NormType::None;
        ChannelType channel_type = ChannelType::None;

        // out = (x * alpha - mean) / std
        static Norm mean_std(const float mean[3], const float std[3],
                            float alpha = 1 / 255.0f,
                            ChannelType channel_type = ChannelType::None);

        // out = x * alpha + beta
        static Norm alpha_beta(float alpha, float beta = 0,
                                ChannelType channel_type = ChannelType::None);

        // None
        static Norm None();
    };

    static __global__ void warp_affine_bilinear_and_normalize_plane_kernel(
    uint8_t *src, int src_line_size, int src_width, int src_height, float *dst,
    int dst_width, int dst_height, uint8_t const_value_st,
    float *warp_affine_matrix_2_3, Norm norm) {
        int dx = blockDim.x * blockIdx.x + threadIdx.x;
        int dy = blockDim.y * blockIdx.y + threadIdx.y;
        if (dx >= dst_width || dy >= dst_height) return;

        float m_x1 = warp_affine_matrix_2_3[0];
        float m_y1 = warp_affine_matrix_2_3[1];
        float m_z1 = warp_affine_matrix_2_3[2];
        float m_x2 = warp_affine_matrix_2_3[3];
        float m_y2 = warp_affine_matrix_2_3[4];
        float m_z2 = warp_affine_matrix_2_3[5];

        float src_x = m_x1 * dx + m_y1 * dy + m_z1;
        float src_y = m_x2 * dx + m_y2 * dy + m_z2;
        float c0, c1, c2;

        if (src_x <= -1 || src_x >= src_width || src_y <= -1 || src_y >= src_height) {
            // out of range
            c0 = const_value_st;
            c1 = const_value_st;
            c2 = const_value_st;
        } else {
            int y_low = floorf(src_y);
            int x_low = floorf(src_x);
            int y_high = y_low + 1;
            int x_high = x_low + 1;

            uint8_t const_value[] = {const_value_st, const_value_st, const_value_st};
            float ly = src_y - y_low;
            float lx = src_x - x_low;
            float hy = 1 - ly;
            float hx = 1 - lx;
            float w1 = hy * hx, w2 = hy * lx, w3 = ly * hx, w4 = ly * lx;
            uint8_t *v1 = const_value;
            uint8_t *v2 = const_value;
            uint8_t *v3 = const_value;
            uint8_t *v4 = const_value;
            if (y_low >= 0) {
            if (x_low >= 0) v1 = src + y_low * src_line_size + x_low * 3;

            if (x_high < src_width) v2 = src + y_low * src_line_size + x_high * 3;
            }

            if (y_high < src_height) {
            if (x_low >= 0) v3 = src + y_high * src_line_size + x_low * 3;

            if (x_high < src_width) v4 = src + y_high * src_line_size + x_high * 3;
            }

            // same to opencv
            c0 = floorf(w1 * v1[0] + w2 * v2[0] + w3 * v3[0] + w4 * v4[0] + 0.5f);
            c1 = floorf(w1 * v1[1] + w2 * v2[1] + w3 * v3[1] + w4 * v4[1] + 0.5f);
            c2 = floorf(w1 * v1[2] + w2 * v2[2] + w3 * v3[2] + w4 * v4[2] + 0.5f);
        }

        if (norm.channel_type == ChannelType::SwapRB) {
            float t = c2;
            c2 = c0;
            c0 = t;
        }

        if (norm.type == NormType::MeanStd) {
            c0 = (c0 * norm.alpha - norm.mean[0]) / norm.std[0];
            c1 = (c1 * norm.alpha - norm.mean[1]) / norm.std[1];
            c2 = (c2 * norm.alpha - norm.mean[2]) / norm.std[2];
        } else if (norm.type == NormType::AlphaBeta) {
            c0 = c0 * norm.alpha + norm.beta;
            c1 = c1 * norm.alpha + norm.beta;
            c2 = c2 * norm.alpha + norm.beta;
        }

        int area = dst_width * dst_height;
        float *pdst_c0 = dst + dy * dst_width + dx;
        float *pdst_c1 = pdst_c0 + area;
        float *pdst_c2 = pdst_c1 + area;
        *pdst_c0 = c0;
        *pdst_c1 = c1;
        *pdst_c2 = c2;
    }

    static void warp_affine_bilinear_and_normalize_plane(
        uint8_t *src, int src_line_size, int src_width, int src_height, float *dst,
        int dst_width, int dst_height, float *matrix_2_3, uint8_t const_value,
        const Norm &norm, cudaStream_t stream) {
    dim3 grid((dst_width + 31) / 32, (dst_height + 31) / 32);
    dim3 block(32, 32);

    checkKernel(warp_affine_bilinear_and_normalize_plane_kernel<<<grid, block, 0,
                                                                    stream>>>(
        src, src_line_size, src_width, src_height, dst, dst_width, dst_height,
        const_value, matrix_2_3, norm));
    }
    Norm Norm::mean_std(const float mean[3], const float std[3], float alpha,
                        ChannelType channel_type) {
    Norm out;
    out.type = NormType::MeanStd;
    out.alpha = alpha;
    out.channel_type = channel_type;
    memcpy(out.mean, mean, sizeof(out.mean));
    memcpy(out.std, std, sizeof(out.std));
    return out;
    }

    Norm Norm::alpha_beta(float alpha, float beta, ChannelType channel_type) {
    Norm out;
    out.type = NormType::AlphaBeta;
    out.alpha = alpha;
    out.beta = beta;
    out.channel_type = channel_type;
    return out;
    }

    Norm Norm::None() { return Norm(); }


    class InferImpl : public Infer {
    public:
    shared_ptr<trt::Infer> trt_;
    string engine_file_;
    Type type_;
    vector<shared_ptr<trt::Memory<unsigned char>>> preprocess_buffers_;
    trt::Memory<float> input_buffer_, output_array_;
    int num_class_=0;
    int network_input_width_, network_input_height_;
    bool isdynamic_model_ = false;
    Norm normalize_;
    float mean_[3], std_[3];
    // vector<int> bbox_head_dims_;
    // bool has_segment_ = false;
    // bool has_keyPoint = false;
    // bool isdynamic_model_ = false;
    // vector<shared_ptr<trt::Memory<unsigned char>>> box_segment_cache_;

    virtual ~InferImpl() = default;

    void adjust_memory(int batch_size) {//complete
        // the inference batch_size
        size_t input_numel = network_input_width_ * network_input_height_ * 3;
        input_buffer_.gpu(batch_size * input_numel);
        output_array_.gpu(batch_size * num_class_);
        output_array_.cpu(batch_size * num_class_);

        if ((int)preprocess_buffers_.size() < batch_size) {
        for (int i = preprocess_buffers_.size(); i < batch_size; ++i)
            preprocess_buffers_.push_back(
                make_shared<trt::Memory<unsigned char>>());
        }
    }

    void preprocess(int ibatch, const Image &image,
                    shared_ptr<trt::Memory<unsigned char>> preprocess_buffer,
                    AffineMatrix &affine, void *stream = nullptr) {
        affine.compute(make_tuple(image.width, image.height),
                    make_tuple(network_input_width_, network_input_height_));

        size_t input_numel = network_input_width_ * network_input_height_ * 3;
        float *input_device = input_buffer_.gpu() + ibatch * input_numel;
        size_t size_image = image.width * image.height * 3;
        size_t size_matrix = upbound(sizeof(affine.d2i), 32);
        uint8_t *gpu_workspace = preprocess_buffer->gpu(size_matrix + size_image);
        float *affine_matrix_device = (float *)gpu_workspace;
        uint8_t *image_device = gpu_workspace + size_matrix;

        uint8_t *cpu_workspace = preprocess_buffer->cpu(size_matrix + size_image);
        float *affine_matrix_host = (float *)cpu_workspace;
        uint8_t *image_host = cpu_workspace + size_matrix;

        // speed up
        cudaStream_t stream_ = (cudaStream_t)stream;
        std::chrono ::high_resolution_clock::time_point a1 =
            std::chrono::high_resolution_clock::now();
        memcpy(image_host, image.bgrptr, size_image);
        memcpy(affine_matrix_host, affine.d2i, sizeof(affine.d2i));
        std::chrono ::high_resolution_clock::time_point a2 =
            std::chrono::high_resolution_clock::now();
        auto time_used2 =
            std::chrono::duration_cast<std::chrono::duration<double>>(a2 - a1);
    //    INFO("memcpy time: %f", time_used2.count() * 1000);
        checkRuntime(cudaMemcpyAsync(image_device, image_host, size_image,
                                    cudaMemcpyHostToDevice, stream_));
        checkRuntime(cudaMemcpyAsync(affine_matrix_device, affine_matrix_host,
                                    sizeof(affine.d2i), cudaMemcpyHostToDevice,
                                    stream_));
        warp_affine_bilinear_and_normalize_plane(
            image_device, image.width * 3, image.width, image.height, input_device,
            network_input_width_, network_input_height_, affine_matrix_device, 114,
            normalize_, stream_);
    }

    bool load(const string &engine_file, Type type) {//complete
        trt_ = trt::load(engine_file);
        if (trt_ == nullptr) return false;

        trt_->print();

        this->type_ = type;

        auto input_dim = trt_->static_dims(0);
        auto output_dims_ = trt_->static_dims(1);
        puts("input_dim");
        for(int i=0;i<4;i++){
            std::cout<<input_dim[i]<<std::endl;
        }
        // puts(input_dim[1]);
        // puts(input_dim[2]);
        // puts(input_dim[3]);
        network_input_width_ = input_dim[3];
        network_input_height_ = input_dim[2];
        isdynamic_model_ = trt_->has_dynamic_dim();

        mean_[0] = 0.485;
        mean_[1] = 0.456;
        mean_[2] = 0.406;
        std_[0] = 0.229;
        std_[1] = 0.224;
        std_[2] = 0.225;
        normalize_ = Norm::mean_std(mean_, std_, 1.0 / 255.0, ChannelType::SwapRB);
        num_class_ = output_dims_[1];
        // std::cout<<"dims:";
        // std::cout<<output_dims_[0]<<std::endl;
        // std::cout<<output_dims_[1]<<std::endl;
        // std::cout<<output_dims_[2]<<std::endl;


        // if (type == Type::densenet121){

        // } else {
        // INFO("Unsupport type %d", type);
        // }
        return true;
    }

    virtual int forward(const Image &image,
                            void *stream = nullptr) override {
        auto output = forwards({image}, stream);
        if (output.empty()) return {};
        return output[0];
    }

    virtual vector<int> forwards(const vector<Image> &images,
                                        void *stream = nullptr) override {
        int num_image = images.size();
        if (num_image == 0) return {};

        auto input_dims = trt_->static_dims(0);
        int infer_batch_size = input_dims[0];
        if (infer_batch_size != num_image) {
        if (isdynamic_model_) {
            infer_batch_size = num_image;
            input_dims[0] = num_image;
            if (!trt_->set_run_dims(0, input_dims)) return {};
        } else {
            if (infer_batch_size < num_image) {
            INFO(
                "When using static shape model, number of images[%d] must be "
                "less than or equal to the maximum batch[%d].",
                num_image, infer_batch_size);
            return {};
            }
        }
        }

 std::chrono ::high_resolution_clock::time_point a1 =
        std::chrono::high_resolution_clock::now();
    adjust_memory(infer_batch_size);//调用内存
    std::chrono ::high_resolution_clock::time_point a2 =
        std::chrono::high_resolution_clock::now();
    auto time_used2 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a2 - a1);
//    INFO("adjust_memory time: %f", time_used2.count() * 1000);

    vector<AffineMatrix> affine_matrixs(num_image);
    std::chrono ::high_resolution_clock::time_point a3 =
        std::chrono::high_resolution_clock::now();
    cudaStream_t stream_ = (cudaStream_t)stream;
    for (int i = 0; i < num_image; ++i)
      preprocess(i, images[i], preprocess_buffers_[i], affine_matrixs[i],
                 stream);

    vector<void *> bindings{input_buffer_.gpu(), output_array_.gpu()};
    //binding是什么

    std::chrono ::high_resolution_clock::time_point a3d1 =
        std::chrono::high_resolution_clock::now();
    auto time_used3 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a3d1 - a3);
//    INFO("preprocess time: %f", time_used3.count() * 1000);
    if (!trt_->forward(bindings, stream)) {
      INFO("Failed to tensorRT forward.");
      return {};
    }
    std::chrono ::high_resolution_clock::time_point a3d2 =
        std::chrono::high_resolution_clock::now();
    auto time_used3d2 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a3d2 - a3d1);
//    INFO("trt forward time: %f", time_used3d2.count() * 1000);

    std::chrono ::high_resolution_clock::time_point a3d3 =
        std::chrono::high_resolution_clock::now();
    auto time_used3d3 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a3d3 - a3d2);
//    INFO("decode_kernel_invoker time: %f", time_used3d3.count() * 1000);
    std::chrono ::high_resolution_clock::time_point a3d4 =
        std::chrono::high_resolution_clock::now();
    checkRuntime(cudaMemcpyAsync(output_array_.cpu(), output_array_.gpu(),
                                 output_array_.gpu_bytes(),
                                 cudaMemcpyDeviceToHost, stream_));
    checkRuntime(cudaStreamSynchronize(stream_));
    std::chrono ::high_resolution_clock::time_point a4 =
        std::chrono::high_resolution_clock::now();
    auto time_used3d4 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a4 - a3d4);
//    INFO("copy output_boxarray_ time: %f", time_used3d4.count() * 1000);
    auto time_used4 =
        std::chrono::duration_cast<std::chrono::duration<double>>(a4 - a3);
    INFO("forward and decode_kernel_invoker time: %f",
         time_used4.count() * 1000);
    std::chrono ::high_resolution_clock::time_point a5 =
        std::chrono::high_resolution_clock::now();
    vector<int> arrout(num_image);
    for(int ib =0; ib < num_image ; ++ib) {
        float *parray = output_array_.cpu() +
        ib * num_class_;
        vector<float> output(num_class_);
        for(int o=0;o<num_class_;o++){
            output[o]=parray[o];
        }
        // std::cout<<ib<<"are"<<std::endl;
        // for(auto out:output){
        //     std::cout<<out<<" ";
        // }
        arrout[ib]=postprocess(output);
    }
    
    

    
    

    return arrout;
  }
};
    Infer *loadraw(const std::string &engine_file, Type type) {
    InferImpl *impl = new InferImpl();
    if (!impl->load(engine_file, type)) {
        delete impl;
        impl = nullptr;
    }
    return impl;
    }
    shared_ptr<Infer> load(const string &engine_file, Type type) {
        return std::shared_ptr<InferImpl>((InferImpl *)loadraw(
            engine_file, type));
    }
}
