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
 * @file LightGlueMatcher.hpp
 * @brief LightGlue feature matcher using ONNX Runtime
 * @author Based on SuperVINS implementation
 */

#ifndef INCLUDE_OKVIS_LIGHTGLUE_MATCHER_HPP_
#define INCLUDE_OKVIS_LIGHTGLUE_MATCHER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>

#ifdef OKVIS_USE_LIGHTGLUE
#include <onnxruntime_cxx_api.h>
#endif

namespace okvis {

/**
 * @brief Match result structure
 */
struct Match {
  int query_idx;  ///< Index in query keypoints
  int train_idx; ///< Index in train keypoints
  float confidence; ///< Match confidence score
};

/**
 * @brief LightGlue feature matcher
 * 
 * Matches keypoints and descriptors using LightGlue neural network.
 * Uses ONNX Runtime for inference.
 */
class LightGlueMatcher {
 public:
  /**
   * @brief Constructor (SuperVINS style)
   * @param model_path Path to LightGlue ONNX model file
   * @param match_threshold Minimum confidence threshold for matches
   * @param use_gpu Whether to use GPU for inference
   * @param extractor_type Extractor type (0=SUPERPOINT, 1=DISK, etc.)
   */
  LightGlueMatcher(const std::string& model_path,
                   float match_threshold = 0.2f,
                   bool use_gpu = true,
                   int extractor_type = 0);

  /**
   * @brief Destructor
   */
  ~LightGlueMatcher();

  /**
   * @brief Match keypoints and descriptors between two frames
   * @param keypoints0 Keypoints from first frame
   * @param descriptors0 Descriptors from first frame (CV_32F, 256 dimensions)
   * @param keypoints1 Keypoints from second frame
   * @param descriptors1 Descriptors from second frame (CV_32F, 256 dimensions)
   * @param image0_size Image size for first frame (width, height) - for keypoint normalization
   * @param image1_size Image size for second frame (width, height) - for keypoint normalization
   * @param matches Output matches with confidence scores
   * @return True if successful
   */
  bool match(const std::vector<cv::KeyPoint>& keypoints0,
             const cv::Mat& descriptors0,
             const std::vector<cv::KeyPoint>& keypoints1,
             const cv::Mat& descriptors1,
             const cv::Size& image0_size,
             const cv::Size& image1_size,
             std::vector<Match>& matches);

  /**
   * @brief Check if matcher is initialized
   * @return True if initialized
   */
  bool isInitialized() const { return initialized_; }

 private:
  bool initialized_;
  float match_threshold_;
  bool use_gpu_;
  int extractor_type_;  // SUPERPOINT=0, DISK=1, etc.

#ifdef OKVIS_USE_LIGHTGLUE
  std::unique_ptr<Ort::Session> session_;
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  
  // SuperVINS style: use char* for names (need to free)
  std::vector<char*> InputNodeNames_;
  std::vector<std::vector<int64_t>> InputNodeShapes_;
  std::vector<char*> OutputNodeNames_;
  std::vector<std::vector<int64_t>> OutputNodeShapes_;
  
  std::vector<Ort::Value> outputtensors_;  // Store output for post_process
#endif

  /**
   * @brief Initialize matcher (SuperVINS style)
   * @param model_path Path to ONNX model
   * @param extractor_type Extractor type (0=SUPERPOINT, 1=DISK, etc.)
   * @param match_threshold Match threshold
   */
  void initialize(const std::string& model_path, int extractor_type, float match_threshold);

  /**
   * @brief Preprocess keypoints (SuperVINS style: normalize)
   * @param kpts Keypoints
   * @param h Image height
   * @param w Image width
   * @return Normalized keypoints
   */
  std::vector<cv::Point2f> pre_process(std::vector<cv::Point2f> kpts, int h, int w);

  /**
   * @brief Match feature points (SuperVINS style)
   * @param kpts0 Keypoints from frame 0
   * @param kpts1 Keypoints from frame 1
   * @param desc0 Descriptor pointer for frame 0
   * @param desc1 Descriptor pointer for frame 1
   * @return Vector of match pairs (idx0, idx1)
   */
  std::vector<std::pair<int, int>> match_featurepoints(
      std::vector<cv::Point2f> kpts0,
      std::vector<cv::Point2f> kpts1,
      float* desc0,
      float* desc1);

  /**
   * @brief Postprocess ONNX output (SuperVINS style)
   * @return Vector of match pairs (idx0, idx1)
   */
  std::vector<std::pair<int, int>> post_process();

  /**
   * @brief Postprocess ONNX output with confidence scores
   * @return Vector of match pairs with confidence scores
   */
  std::vector<std::pair<std::pair<int, int>, float>> post_process_with_confidence();
};

} // namespace okvis

#endif // INCLUDE_OKVIS_LIGHTGLUE_MATCHER_HPP_

