import argparse
import subprocess

def run_trtexec(onnx_path, save_engine_path, min_batch, opt_batch, max_batch, input_name,shape):
    command = ["/usr/src/tensorrt/bin/trtexec",
               "--onnx=" + onnx_path,
               "--saveEngine=" + save_engine_path,
               "--minShapes=" + input_name + ":" + min_batch + "x3x" +shape,
               "--optShapes=" + input_name + ":" + opt_batch + "x3x" +shape,
               "--maxShapes=" + input_name + ":" + max_batch + "x3x" +shape]
    subprocess.run(command)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run TensorRT with specified parameters.")
    parser.add_argument("--onnx", default="", help="Path to the ONNX file.")
    parser.add_argument("--saveEngine", default="", help="Path to save the TensorRT engine file.")
    parser.add_argument("--Shape", default="224x224", help="")
    parser.add_argument("--minBatch", default="1", help="Minimum batch size for input.")
    parser.add_argument("--optBatch", default="10", help="Optimum batch size for input.")
    parser.add_argument("--maxBatch", default="20", help="Maximum batch size for input.")
    parser.add_argument("--input_name", default="input", help="Name of the input tensor.")
    args = parser.parse_args()

    run_trtexec(args.onnx, args.saveEngine, args.minBatch, args.optBatch, args.maxBatch,args.input_name,args.Shape)
