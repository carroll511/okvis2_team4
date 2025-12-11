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
 * @file SuperPointExtractor.hpp
 * @brief SuperPoint feature extractor using ONNX Runtime
 * @author Based on SuperVINS implementation
 */

#ifndef INCLUDE_OKVIS_SUPERPOINT_EXTRACTOR_HPP_
#define INCLUDE_OKVIS_SUPERPOINT_EXTRACTOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef OKVIS_USE_SUPERPOINT
#include <onnxruntime_cxx_api.h>
#endif

namespace okvis {

/**
 * @brief SuperPoint feature extractor
 * 
 * Extracts keypoints and descriptors using SuperPoint neural network.
 * Uses ONNX Runtime for inference.
 */
class SuperPointExtractor {
 public:
  /**
   * @brief Constructor
   * @param model_path Path to SuperPoint ONNX model file
   * @param max_keypoints Maximum number of keypoints to extract
   * @param keypoint_threshold Keypoint detection threshold
   * @param use_gpu Whether to use GPU for inference
   * @param use_clahe Whether to use CLAHE for lighting correction (better for local variations)
   */
  SuperPointExtractor(const std::string& model_path,
                      int max_keypoints = 2048,
                      float keypoint_threshold = 0.015f,
                      bool use_gpu = true,
                      bool use_clahe = false);

  /**
   * @brief Destructor
   */
  ~SuperPointExtractor();

  /**
   * @brief Extract keypoints and descriptors from image
   * @param image Input image (grayscale)
   * @param keypoints Output keypoints
   * @param descriptors Output descriptors (CV_32F, 256 dimensions)
   * @return True if successful
   */
  bool extract(const cv::Mat& image,
               std::vector<cv::KeyPoint>& keypoints,
               cv::Mat& descriptors);

  /**
   * @brief Check if extractor is initialized
   * @return True if initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get descriptor size
   * @return Descriptor size (256 for SuperPoint)
   */
  int descriptorSize() const { return 256; }

  /**
   * @brief Get descriptor type
   * @return CV_32F
   */
  int descriptorType() const { return CV_32F; }

 private:
  bool initialized_;
  int max_keypoints_;
  float keypoint_threshold_;
  bool use_gpu_;
  bool use_clahe_;  // Use CLAHE for lighting correction
  float scale_;  // Image scale factor from preprocessing

#ifdef OKVIS_USE_SUPERPOINT
  std::unique_ptr<Ort::Session> session_;
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  
  // SuperVINS style: use char* for names (need to free)
  std::vector<char*> InputNodeNames_;
  std::vector<std::vector<int64_t>> InputNodeShapes_;
  std::vector<char*> OutputNodeNames_;
  std::vector<std::vector<int64_t>> OutputNodeShapes_;
#endif

  /**
   * @brief Initialize extractor (SuperVINS style)
   * @param model_path Path to ONNX model
   */
  void initialize(const std::string& model_path);

  /**
   * @brief Preprocess image (SuperVINS style)
   * @param image Input image
   * @param scale Output scale factor
   * @return Preprocessed image
   */
  cv::Mat pre_process(const cv::Mat& image, float& scale);

  /**
   * @brief Extract feature points (SuperVINS style)
   * @param image Preprocessed image
   * @return Pair of keypoints and descriptor pointer
   */
  std::pair<std::vector<cv::Point2f>, float*> extract_featurepoints(const cv::Mat& image);

  /**
   * @brief Postprocess ONNX output (SuperVINS style)
   * @param tensor Output tensors from ONNX
   * @return Pair of keypoints and descriptor pointer
   */
  std::pair<std::vector<cv::Point2f>, float*> post_process(std::vector<Ort::Value> tensor);
};

} // namespace okvis

#endif // INCLUDE_OKVIS_SUPERPOINT_EXTRACTOR_HPP_

