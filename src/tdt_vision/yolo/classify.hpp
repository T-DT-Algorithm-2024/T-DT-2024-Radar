#ifndef __CLASSIFY_HPP__
#define __CLASSIFY_HPP__

#include <future>
#include <memory>
#include <string>
#include <vector>
#include "NvInferRuntime.h"
#include <NvInfer.h>
#include <fstream>
#include "cuda_runtime_api.h"
#include "logging.h"

namespace classify {
enum class Type : int {
  densenet121 = 0
};

struct Image {
  const void *bgrptr = nullptr;
  int width = 0, height = 0;

  Image() = default;
  Image(const void *bgrptr, int width, int height)
      : bgrptr(bgrptr), width(width), height(height) {}
};

class Infer {
public:
  virtual int forward(const Image &image, void *stream = nullptr) = 0;
  virtual std::vector<int> forwards(const std::vector<Image> &images,
                                         void *stream = nullptr) = 0;
};

std::shared_ptr<Infer> load(const std::string &engine_file, Type type);

const char *type_name(Type type);

}



#endif // __CLASSIFY_HPP__