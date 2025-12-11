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
 * @file LightGlueMatcher.cpp
 * @brief LightGlue feature matcher implementation (SuperVINS style)
 * @author Based on SuperVINS Matcher_DPL implementation
 */

#include <okvis/LightGlueMatcher.hpp>
#include <okvis/TransformDPL.hpp>
#include <okvis/assert_macros.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <thread>
#include <iostream>

#ifdef OKVIS_USE_LIGHTGLUE
#include <onnxruntime_cxx_api.h>
#endif

namespace okvis {

#ifdef OKVIS_USE_LIGHTGLUE

// SuperVINS constants
const int SUPERPOINT = 0;
const int DISK = 1;
const int SUPERPOINT_SIZE = 256;
const int DISK_SIZE = 128;

LightGlueMatcher::LightGlueMatcher(const std::string& model_path,
                                   float match_threshold,
                                   bool use_gpu,
                                   int extractor_type)
    : initialized_(false),
      match_threshold_(match_threshold),
      use_gpu_(use_gpu),
      extractor_type_(extractor_type) {
  
  initialize(model_path, extractor_type_, match_threshold_);
}

void LightGlueMatcher::initialize(const std::string& model_path, 
                                  int extractor_type, 
                                  float match_threshold) {
  match_threshold_ = match_threshold;
  extractor_type_ = extractor_type;
  
  std::cout << "match threshold = " << match_threshold_ << std::endl;
  
  try {
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LightGlueMatcher");
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
    LOG(INFO) << "LightGlue matcher initialized from: " << model_path;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to initialize LightGlue matcher: " << e.what();
    initialized_ = false;
  }
}

LightGlueMatcher::~LightGlueMatcher() {
  // Clean up allocated strings
  for (auto* name : InputNodeNames_) {
    free(const_cast<char*>(name));
  }
  for (auto* name : OutputNodeNames_) {
    free(const_cast<char*>(name));
  }
}

std::vector<cv::Point2f> LightGlueMatcher::pre_process(std::vector<cv::Point2f> kpts, int h, int w) {
  // SuperVINS style: normalize keypoints
  return transform_dpl::NormalizeKeypoints(kpts, h, w);
}

std::vector<std::pair<int, int>> LightGlueMatcher::match_featurepoints(
    std::vector<cv::Point2f> kpts0,
    std::vector<cv::Point2f> kpts1,
    float* desc0,
    float* desc1) {
  
  // Prepare input shapes (SuperVINS style)
  InputNodeShapes_[0] = {1, static_cast<int>(kpts0.size()), 2};
  InputNodeShapes_[1] = {1, static_cast<int>(kpts1.size()), 2};
  
  int desc_dim = (extractor_type_ == SUPERPOINT) ? SUPERPOINT_SIZE : DISK_SIZE;
  InputNodeShapes_[2] = {1, static_cast<int>(kpts0.size()), desc_dim};
  InputNodeShapes_[3] = {1, static_cast<int>(kpts1.size()), desc_dim};

  auto memory_info_handler = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeCPU);

  // Prepare keypoint data
  float* kpts0_data = new float[kpts0.size() * 2];
  float* kpts1_data = new float[kpts1.size() * 2];

  for (size_t i = 0; i < kpts0.size(); ++i) {
    kpts0_data[i * 2] = kpts0[i].x;
    kpts0_data[i * 2 + 1] = kpts0[i].y;
  }
  for (size_t i = 0; i < kpts1.size(); ++i) {
    kpts1_data[i * 2] = kpts1[i].x;
    kpts1_data[i * 2 + 1] = kpts1[i].y;
  }

  std::vector<Ort::Value> input_tensors;
  input_tensors.push_back(Ort::Value::CreateTensor<float>(
      memory_info_handler, kpts0_data, kpts0.size() * 2 * sizeof(float),
      InputNodeShapes_[0].data(), InputNodeShapes_[0].size()));
  input_tensors.push_back(Ort::Value::CreateTensor<float>(
      memory_info_handler, kpts1_data, kpts1.size() * 2 * sizeof(float),
      InputNodeShapes_[1].data(), InputNodeShapes_[1].size()));
  input_tensors.push_back(Ort::Value::CreateTensor<float>(
      memory_info_handler, desc0, kpts0.size() * desc_dim * sizeof(float),
      InputNodeShapes_[2].data(), InputNodeShapes_[2].size()));
  input_tensors.push_back(Ort::Value::CreateTensor<float>(
      memory_info_handler, desc1, kpts1.size() * desc_dim * sizeof(float),
      InputNodeShapes_[3].data(), InputNodeShapes_[3].size()));

  // Run inference
  auto output_tensor = session_->Run(Ort::RunOptions{nullptr}, 
                                     InputNodeNames_.data(), input_tensors.data(),
                                     input_tensors.size(), 
                                     OutputNodeNames_.data(), OutputNodeNames_.size());

  // Check output tensors
  for (auto& tensor : output_tensor) {
    if (!tensor.IsTensor() || !tensor.HasValue()) {
      std::cerr << "[ERROR] Inference output tensor is not a tensor or don't have value" << std::endl;
    }
  }
  
  // Store output tensors for post-processing (don't clear until after post_process_with_confidence)
  outputtensors_ = std::move(output_tensor);

  // Postprocess (for backward compatibility, but don't clear outputtensors_)
  std::vector<std::pair<int, int>> result_matches = post_process();

  // Cleanup keypoint data (but keep outputtensors_ for confidence extraction)
  delete[] kpts0_data;
  delete[] kpts1_data;
  // Note: outputtensors_ will be cleared in match() after post_process_with_confidence

  return result_matches;
}

std::vector<std::pair<int, int>> LightGlueMatcher::post_process() {
  // SuperVINS output format: [matches (int64_t) [N, 2], mscores (float) [N]]
  std::vector<std::pair<int, int>> good_matches;
  
  std::vector<int64_t> matches_Shape = outputtensors_[0].GetTensorTypeAndShapeInfo().GetShape();
  int64_t* matches = (int64_t*)outputtensors_[0].GetTensorMutableData<void>();
  
  std::vector<int64_t> mscore_Shape = outputtensors_[1].GetTensorTypeAndShapeInfo().GetShape();
  float* mscores = (float*)outputtensors_[1].GetTensorMutableData<void>();
  
  // SuperVINS format: matches is [N, 2] where each row is [idx0, idx1]
  int num_matches = matches_Shape[0];
  for (int i = 0; i < num_matches; i++) {
    if (mscores[i] > match_threshold_) {
      good_matches.emplace_back(std::make_pair(
          static_cast<int>(matches[i * 2]), 
          static_cast<int>(matches[i * 2 + 1])));
    }
  }
  
  return good_matches;
}

std::vector<std::pair<std::pair<int, int>, float>> LightGlueMatcher::post_process_with_confidence() {
  // SuperVINS output format: [matches (int64_t) [N, 2], mscores (float) [N]]
  std::vector<std::pair<std::pair<int, int>, float>> good_matches;
  
  std::vector<int64_t> matches_Shape = outputtensors_[0].GetTensorTypeAndShapeInfo().GetShape();
  int64_t* matches = (int64_t*)outputtensors_[0].GetTensorMutableData<void>();
  
  std::vector<int64_t> mscore_Shape = outputtensors_[1].GetTensorTypeAndShapeInfo().GetShape();
  float* mscores = (float*)outputtensors_[1].GetTensorMutableData<void>();
  
  // SuperVINS format: matches is [N, 2] where each row is [idx0, idx1]
  int num_matches = matches_Shape[0];
  for (int i = 0; i < num_matches; i++) {
    if (mscores[i] > match_threshold_) {
      good_matches.emplace_back(std::make_pair(
          std::make_pair(
              static_cast<int>(matches[i * 2]), 
              static_cast<int>(matches[i * 2 + 1])),
          mscores[i]));  // Return actual confidence score
    }
  }
  
  return good_matches;
}

bool LightGlueMatcher::match(const std::vector<cv::KeyPoint>& keypoints0,
                             const cv::Mat& descriptors0,
                             const std::vector<cv::KeyPoint>& keypoints1,
                             const cv::Mat& descriptors1,
                             const cv::Size& image0_size,
                             const cv::Size& image1_size,
                             std::vector<Match>& matches) {
  matches.clear();
  
  if (!initialized_) {
    LOG(ERROR) << "LightGlue matcher not initialized";
    return false;
  }

  if (keypoints0.empty() || keypoints1.empty() ||
      descriptors0.empty() || descriptors1.empty()) {
    return true; // No matches, but not an error
  }

  // Convert cv::KeyPoint to cv::Point2f
  std::vector<cv::Point2f> kpts0, kpts1;
  kpts0.reserve(keypoints0.size());
  kpts1.reserve(keypoints1.size());
  
  for (const auto& kp : keypoints0) {
    kpts0.push_back(kp.pt);
  }
  for (const auto& kp : keypoints1) {
    kpts1.push_back(kp.pt);
  }

  // Normalize keypoints (SuperVINS style) - CRITICAL FIX
  kpts0 = pre_process(kpts0, image0_size.height, image0_size.width);
  kpts1 = pre_process(kpts1, image1_size.height, image1_size.width);

  // Get descriptor data pointers
  // Assume descriptors are CV_32F format
  if (descriptors0.type() != CV_32F || descriptors1.type() != CV_32F) {
    LOG(ERROR) << "Descriptors must be CV_32F format";
    return false;
  }

  float* desc0_data = (float*)descriptors0.data;
  float* desc1_data = (float*)descriptors1.data;
  
  // Match feature points (SuperVINS style)
  // This will populate outputtensors_ for post-processing
  match_featurepoints(kpts0, kpts1, desc0_data, desc1_data);

  // Get matches with confidence scores (outputtensors_ is still valid)
  std::vector<std::pair<std::pair<int, int>, float>> match_pairs_with_conf = 
      post_process_with_confidence();

  // Cleanup outputtensors_ after extracting confidence
  outputtensors_.clear();

  // Convert to Match structure with actual confidence scores
  matches.reserve(match_pairs_with_conf.size());
  for (const auto& pair_conf : match_pairs_with_conf) {
    Match m;
    m.query_idx = pair_conf.first.first;
    m.train_idx = pair_conf.first.second;
    m.confidence = pair_conf.second;  // Use actual confidence from LightGlue
    matches.push_back(m);
  }

  return true;
}

#endif // OKVIS_USE_LIGHTGLUE

} // namespace okvis

