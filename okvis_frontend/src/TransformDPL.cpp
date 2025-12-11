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

#include <okvis/TransformDPL.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <cmath>

namespace okvis {
namespace transform_dpl {

cv::Mat NormalizeImage(cv::Mat& image) {
  // SuperVINS style: BGR2RGB then normalize to [0, 1]
  cv::Mat normalizedImage = image.clone();
  
  if (image.channels() == 3) {
    cv::cvtColor(normalizedImage, normalizedImage, cv::COLOR_BGR2RGB);
    normalizedImage.convertTo(normalizedImage, CV_32F, 1.0 / 255.0);
  } else if (image.channels() == 1) {
    image.convertTo(normalizedImage, CV_32F, 1.0 / 255.0);
  } else {
    throw std::invalid_argument("[ERROR] Not an image");
  }
  
  return normalizedImage;
}

cv::Mat ResizeImage(const cv::Mat& image, int size, float& scale, 
                   const std::string& fn, const std::string& interp) {
  // SuperVINS style: resize based on max/min edge
  int h = image.rows;
  int w = image.cols;
  
  std::function<int(int, int)> func;
  if (fn == "max") {
    func = [](int a, int b) { return std::max(a, b); };
  } else if (fn == "min") {
    func = [](int a, int b) { return std::min(a, b); };
  } else {
    throw std::invalid_argument("[ERROR] Incorrect function: " + fn);
  }
  
  int h_new, w_new;
  if (size == 512 || size == 1024 || size == 2048) {
    scale = static_cast<float>(size) / static_cast<float>(func(h, w));
    h_new = static_cast<int>(round(h * scale));
    w_new = static_cast<int>(round(w * scale));
  } else {
    throw std::invalid_argument("Incorrect new size: " + std::to_string(size));
  }
  
  int mode;
  if (interp == "linear") {
    mode = cv::INTER_LINEAR;
  } else if (interp == "cubic") {
    mode = cv::INTER_CUBIC;
  } else if (interp == "nearest") {
    mode = cv::INTER_NEAREST;
  } else if (interp == "area") {
    mode = cv::INTER_AREA;
  } else {
    throw std::invalid_argument("[ERROR] Incorrect interpolation mode: " + interp);
  }
  
  cv::Mat resizeImage;
  cv::resize(image, resizeImage, cv::Size(w_new, h_new), 0, 0, mode);
  
  return resizeImage;
}

std::vector<cv::Point2f> NormalizeKeypoints(std::vector<cv::Point2f> kpts, int h, int w) {
  // SuperVINS style: center shift + scale normalization
  cv::Size size(w, h);
  cv::Point2f shift(static_cast<float>(w) / 2, static_cast<float>(h) / 2);
  float scale = static_cast<float>(std::max(w, h)) / 2;
  
  std::vector<cv::Point2f> normalizedKpts;
  for (const cv::Point2f& kpt : kpts) {
    cv::Point2f normalizedKpt = (kpt - shift) / scale;
    normalizedKpts.push_back(normalizedKpt);
  }
  
  return normalizedKpts;
}

cv::Mat RGB2Grayscale(cv::Mat& image) {
  // SuperVINS style: RGB to Grayscale
  cv::Mat resultImage;
  cv::cvtColor(image, resultImage, cv::COLOR_RGB2GRAY);
  return resultImage;
}


} // namespace transform_dpl
} // namespace okvis

