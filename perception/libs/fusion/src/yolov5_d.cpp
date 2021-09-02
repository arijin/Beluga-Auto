#include "fusions/yolov5_d.h"
#include "common.hpp"

#define USE_FP16 // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0 // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

static Logger gLogger;

Yolov5DET::Yolov5DET(std::string names_file, std::string engine_file) : names_file_(names_file), engine_file_(engine_file)
{
  //---------------------CUDA engine deserilize to detect.
  cudaSetDevice(DEVICE);
  // deserialize the .engine and run inference
  std::ifstream file(engine_file_, std::ios::binary);
  if (!file.good())
  {
    std::cerr << "read " << engine_file_ << " error!" << std::endl;
  }
  char *trtModelStream = nullptr;
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  trtModelStream = new char[size];
  assert(trtModelStream);
  file.read(trtModelStream, size);
  file.close();

  static float prob[BATCH_SIZE * OUTPUT_SIZE];
  IRuntime *runtime = createInferRuntime(gLogger);
  assert(runtime != nullptr);
  ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
  assert(engine != nullptr);
  context = engine->createExecutionContext();
  assert(context != nullptr);
  delete[] trtModelStream;
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
  const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
  // Create stream
  CUDA_CHECK(cudaStreamCreate(&stream));

  obj_names = objects_names_from_file(names_file_);
}

Yolov5DET::~Yolov5DET() {}

void Yolov5DET::doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output, int batchSize)
{
  // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
  CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
  context.enqueue(batchSize, buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

std::vector<std::string> Yolov5DET::objects_names_from_file(std::string const filename)
{
  std::ifstream file(filename);
  std::vector<std::string> file_lines;
  if (!file.is_open())
    return file_lines;
  for (std::string line; getline(file, line);)
    file_lines.push_back(line);
  std::cout << "object names loaded \n";
  return file_lines;
}

// single image input
void Yolov5DET::process_func(std::vector<cv::Mat> frames)
{
  // prepare input data ---------------------------
  static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
  static float prob[BATCH_SIZE * OUTPUT_SIZE];
  for (int b = 0; b < BATCH_SIZE; b++)
  {
    cv::Mat img = frames[b];
    if (img.empty())
      continue;
    cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
    int i = 0;
    for (int row = 0; row < INPUT_H; ++row)
    {
      uchar *uc_pixel = pr_img.data + row * pr_img.step;
      for (int col = 0; col < INPUT_W; ++col)
      {
        data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
        data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
        data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
        uc_pixel += 3;
        ++i;
      }
    }
  }
  // forward ---------------------
  auto start = std::chrono::system_clock::now();
  doInference(*context, stream, buffers, data, prob, BATCH_SIZE); // 输出prob，prob[0]是输出的个数。
  auto end = std::chrono::system_clock::now();
  std::vector<std::vector<Yolo::Detection>> batch_res(BATCH_SIZE);
  for (int b = 0; b < BATCH_SIZE; b++)
  {
    auto &res = batch_res[b];
    nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
  }
  // value update ------------------
  std::vector<cv::Rect> bboxes;
  std::vector<std::string> labels;
  std::vector<int> class_ids;
  auto &res = batch_res[0];
  for (size_t j = 0; j < res.size(); j++)
  {
    cv::Rect r = get_rect(frames[0], res[j].bbox);
    std::string label = obj_names[(int)res[j].class_id];
    char str[10];
    sprintf(str, "%.2f", res[j].conf);
    std::string prob_text(str);
    std::string label_text = label + " " + prob_text;
    bboxes.push_back(r);
    labels.push_back(label_text);
    std::cout << label_text << std::endl;
    class_ids.push_back((int)res[j].class_id);
  }
  bboxes_ = bboxes;
  labels_ = labels;
  class_ids_ = class_ids;
  return;
}

std::vector<cv::Rect> Yolov5DET::get_bboxes(void)
{
  return bboxes_;
}

std::vector<std::string> Yolov5DET::get_labels(void)
{
  return labels_;
}

std::vector<int> Yolov5DET::get_class_ids(void)
{
  return class_ids_;
}

std::vector<std::string> Yolov5DET::get_class_names(void)
{
  std::vector<std::string> class_names;
  for (int i = 0; i < class_ids_.size(); i++)
  {
    class_names.push_back(obj_names[class_ids_[i]]);
  }
  return class_names;
}