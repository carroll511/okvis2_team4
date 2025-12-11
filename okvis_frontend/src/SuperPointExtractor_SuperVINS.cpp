/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file SuperPointExtractor.cpp
 * @brief SuperPoint feature extractor implementation (SuperVINS style)
 * @author Based on SuperVINS Extractor_DPL implementation
 */

#include <okvis/SuperPointExtractor.hpp>
#include <okvis/TransformDPL.hpp>
#include <okvis/assert_macros.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <thread>
#include <functional>

#ifdef OKVIS_USE_SUPERPOINT
#include <onnxruntime_cxx_api.h>
#endif

namespace okvis {

#ifdef OKVIS_USE_SUPERPOINT

// SuperVINS constants
const unsigned int IMAGE_SIZE_DPL = 512;
const int SUPERPOINT = 0;  // Extractor type

SuperPointExtractor::SuperPointExtractor(const std::string& model_path,
                                         int max_keypoints,
                                         float keypoint_threshold,
                                         bool use_gpu,
                                         bool use_clahe)
    : initialized_(false),
      max_keypoints_(max_keypoints),
      keypoint_threshold_(keypoint_threshold),
      use_gpu_(use_gpu),
      use_clahe_(use_clahe),
      scale_(1.0f),
      env_(ORT_LOGGING_LEVEL_WARNING, "SuperPointExtractor"),
      allocator_() {
  
  initialize(model_path);
}

void SuperPointExtractor::initialize(const std::string& model_path) {
  try {
    session_options_ = Ort::SessionOptions();
    session_options_.SetInterOpNumThreads(std::thread::hardware_concurrency());
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (use_gpu_) {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = 0;
      cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
      cuda_options.gpu_mem_limit = 0;
      cuda_options.arena_extend_strategy = 1;
      cuda_options.do_copy_in_default_stream = 1;
      cuda_options.has_user_compute_stream = 0;
      cuda_options.default_memory_arena_cfg = nullptr;

      session_options_.AppendExecutionProvider_CUDA(cuda_options);
      session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    }

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    // Get input/output names and shapes (SuperVINS style)
    size_t numInputNodes = session_->GetInputCount();
    InputNodeNames_.reserve(numInputNodes);
    InputNodeShapes_.reserve(numInputNodes);
    for (size_t i = 0; i < numInputNodes; i++) {
      InputNodeNames_.emplace_back(strdup(session_->GetInputNameAllocated(i, allocator_).get()));
      InputNodeShapes_.emplace_back(session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }

    size_t numOutputNodes = session_->GetOutputCount();
    OutputNodeNames_.reserve(numOutputNodes);
    OutputNodeShapes_.reserve(numOutputNodes);
    for (size_t i = 0; i < numOutputNodes; i++) {
      OutputNodeNames_.emplace_back(strdup(session_->GetOutputNameAllocated(i, allocator_).get()));
      OutputNodeShapes_.emplace_back(session_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }

    initialized_ = true;
    LOG(INFO) << "SuperPoint extractor initialized from: " << model_path;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to initialize SuperPoint extractor: " << e.what();
    initialized_ = false;
  }
}

SuperPointExtractor::~SuperPointExtractor() {
  // Clean up allocated strings
  for (auto* name : InputNodeNames_) {
    free(const_cast<char*>(name));
  }
  for (auto* name : OutputNodeNames_) {
    free(const_cast<char*>(name));
  }
}

cv::Mat SuperPointExtractor::pre_process(const cv::Mat& image, float& scale) {
  // SuperVINS style preprocessing
  float temp_scale = scale;
  cv::Mat tempImage = image.clone();
  std::string fn = "max";
  std::string interp = "area";
  
  cv::Mat resize_img = transform_dpl::ResizeImage(tempImage, IMAGE_SIZE_DPL, scale, fn, interp);
  cv::Mat resultImage = transform_dpl::NormalizeImage(resize_img, use_clahe_);
  
  if (tempImage.channels() == 3) {
    LOG(INFO) << "[INFO] ExtractorType Superpoint turn RGB to Grayscale";
    resultImage = transform_dpl::RGB2Grayscale(resultImage);
  }
  
  return resultImage;
}

std::pair<std::vector<cv::Point2f>, float*> SuperPointExtractor::extract_featurepoints(const cv::Mat& image) {
  // Preprocess
  cv::Mat preprocessed = pre_process(image, scale_);
  
  // Prepare input tensor (SuperVINS style)
  int InputTensorSize;
  InputNodeShapes_[0] = {1, 1, preprocessed.rows, preprocessed.cols};
  InputTensorSize = InputNodeShapes_[0][0] * InputNodeShapes_[0][1] * 
                    InputNodeShapes_[0][2] * InputNodeShapes_[0][3];

  std::vector<float> srcInputTensorValues(InputTensorSize);
  srcInputTensorValues.assign(preprocessed.begin<float>(), preprocessed.end<float>());

  auto memory_info_handler = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeCPU);

  std::vector<Ort::Value> input_tensors;
  input_tensors.push_back(Ort::Value::CreateTensor<float>(
      memory_info_handler, srcInputTensorValues.data(), srcInputTensorValues.size(),
      InputNodeShapes_[0].data(), InputNodeShapes_[0].size()));

  // Run inference
  auto output_tensor = session_->Run(Ort::RunOptions{nullptr}, 
                                     InputNodeNames_.data(), input_tensors.data(),
                                     input_tensors.size(), 
                                     OutputNodeNames_.data(), OutputNodeNames_.size());

  // Postprocess (SuperVINS style)
  std::pair<std::vector<cv::Point2f>, float*> result = post_process(std::move(output_tensor));
  
  return result;
}

std::pair<std::vector<cv::Point2f>, float*> SuperPointExtractor::post_process(std::vector<Ort::Value> tensor) {
  // SuperVINS output format: [keypoints (int64), scores (float), descriptors (float)]
  std::pair<std::vector<cv::Point2f>, float*> extractor_result;
  
  std::vector<int64_t> kpts_Shape = tensor[0].GetTensorTypeAndShapeInfo().GetShape();
  int64_t* kpts = (int64_t*)tensor[0].GetTensorMutableData<void>();

  std::vector<int64_t> score_Shape = tensor[1].GetTensorTypeAndShapeInfo().GetShape();
  float* scores = (float*)tensor[1].GetTensorMutableData<void>();

  std::vector<int64_t> descriptors_Shape = tensor[2].GetTensorTypeAndShapeInfo().GetShape();
  float* desc = (float*)tensor[2].GetTensorMutableData<void>();
  
  std::vector<cv::Point2f> kpts_f;
  for (int i = 0; i < kpts_Shape[1] * 2; i += 2) {
    kpts_f.emplace_back(cv::Point2f(static_cast<float>(kpts[i]), static_cast<float>(kpts[i + 1])));
  }

  extractor_result.first = kpts_f;
  extractor_result.second = desc;
  return extractor_result;
}

bool SuperPointExtractor::extract(const cv::Mat& image,
                                  std::vector<cv::KeyPoint>& keypoints,
                                  cv::Mat& descriptors) {
  if (!initialized_) {
    LOG(ERROR) << "SuperPoint extractor not initialized";
    return false;
  }

  if (image.empty()) {
    LOG(ERROR) << "Input image is empty";
    return false;
  }

  // Extract features (SuperVINS style)
  auto result = extract_featurepoints(image);
  std::vector<cv::Point2f> kpts = result.first;
  float* desc_data = result.second;

  // Convert to cv::KeyPoint format and scale back to original image coordinates
  // CRITICAL FIX: Track valid keypoint indices to match with descriptors
  keypoints.clear();
  keypoints.reserve(kpts.size());
  std::vector<size_t> valid_indices;  // Map from keypoints index to original kpts index
  valid_indices.reserve(kpts.size());
  
  for (size_t orig_idx = 0; orig_idx < kpts.size(); ++orig_idx) {
    const auto& kpt = kpts[orig_idx];
    // Scale back to original image coordinates
    cv::Point2f pt = cv::Point2f((kpt.x + 0.5f) / scale_ - 0.5f, 
                                  (kpt.y + 0.5f) / scale_ - 0.5f);
    
    // Check bounds
    if (pt.x < 0 || pt.x >= image.cols || pt.y < 0 || pt.y >= image.rows) {
      continue;  // Skip out-of-bounds keypoints
    }
    
    cv::KeyPoint kp;
    kp.pt = pt;
    kp.size = 8.0f;
    kp.response = 1.0f;  // SuperVINS doesn't return response in extract_featurepoints
    keypoints.push_back(kp);
    valid_indices.push_back(orig_idx);  // Store mapping to original index
  }

  // Convert descriptors to cv::Mat - use only valid keypoint indices
  if (desc_data && !keypoints.empty() && !valid_indices.empty()) {
    int num_kpts = keypoints.size();
    descriptors = cv::Mat(num_kpts, 256, CV_32F);
    
    // Use valid_indices to map keypoints to correct descriptors
    for (size_t i = 0; i < valid_indices.size(); ++i) {
      size_t orig_idx = valid_indices[i];
      float* desc_ptr = desc_data + orig_idx * 256;  // Use original index for descriptor
      for (int j = 0; j < 256; ++j) {
        descriptors.at<float>(i, j) = desc_ptr[j];
      }
    }
  } else {
    descriptors = cv::Mat();
  }

  return true;
}

#endif // OKVIS_USE_SUPERPOINT

} // namespace okvis

