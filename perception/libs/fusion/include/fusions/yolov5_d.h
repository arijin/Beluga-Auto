#ifndef YOLOV5_D_H_
#define YOLOV5_D_H_

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <future>
#include <atomic>
#include <mutex> // std::mutex, std::unique_lock
#include <cmath>
using namespace std;

#include "NvInfer.h"
#include "cuda_utils.h"
#include "yololayer.h"
#include "logging.h"
// #include "common.hpp"
#include "utils.h"
using namespace nvinfer1;

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define OPENCV

class Yolov5DET
{
private:
    // stuff we know about the network and the input/output blobs
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int CLASS_NUM = Yolo::CLASS_NUM;
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1

    const char *INPUT_BLOB_NAME = "data";
    const char *OUTPUT_BLOB_NAME = "prob";

    std::string names_file_;
    std::string engine_file_;

    std::vector<std::string> obj_names;
    IExecutionContext *context = nullptr;
    void *buffers[2];
    cudaStream_t stream;

    std::vector<cv::Rect> bboxes_;
    std::vector<std::string> labels_;
    std::vector<int> class_ids_;

    void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output, int batchSize);
    std::vector<std::string> objects_names_from_file(std::string const filename);

public:
    Yolov5DET(std::string names_file, std::string engine_file);
    ~Yolov5DET();
    void process_func(std::vector<cv::Mat> frames);

    std::vector<cv::Rect> get_bboxes(void);
    std::vector<std::string> get_labels(void);
    std::vector<int> get_class_ids(void);
    std::vector<std::string> get_class_names(void);
};

#endif // YOLOV5_D_H_
