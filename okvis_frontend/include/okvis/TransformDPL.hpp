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
 * @file TransformDPL.hpp
 * @brief Preprocessing and postprocessing utilities for Deep Learning models
 * @author Based on SuperVINS transform_dpl implementation
 */

#ifndef INCLUDE_OKVIS_TRANSFORM_DPL_HPP_
#define INCLUDE_OKVIS_TRANSFORM_DPL_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <Eigen/Dense>

namespace okvis {
namespace transform_dpl {

/**
 * @brief Normalize image to [0, 1] range (SuperVINS style: BGR2RGB then normalize)
 * @param image Input image (BGR or grayscale)
 * @return Normalized image [0, 1]
 */
cv::Mat NormalizeImage(cv::Mat& image);

/**
 * @brief Resize image based on max/min edge (SuperVINS style)
 * @param image Input image
 * @param size Target size (512, 1024, or 2048)
 * @param scale Output scale factor
 * @param fn Function: "max" or "min"
 * @param interp Interpolation: "linear", "cubic", "nearest", or "area"
 * @return Resized image
 */
cv::Mat ResizeImage(const cv::Mat& image, int size, float& scale, 
                   const std::string& fn = "max",
                   const std::string& interp = "area");

/**
 * @brief Normalize keypoints (SuperVINS style: center shift + scale)
 * @param kpts Input keypoints in pixel coordinates
 * @param h Image height
 * @param w Image width
 * @return Normalized keypoints
 */
std::vector<cv::Point2f> NormalizeKeypoints(std::vector<cv::Point2f> kpts, int h, int w);

/**
 * @brief Convert RGB to Grayscale (SuperVINS style)
 * @param image Input RGB image
 * @return Grayscale image
 */
cv::Mat RGB2Grayscale(cv::Mat& image);

} // namespace transform_dpl
} // namespace okvis

#endif // INCLUDE_OKVIS_TRANSFORM_DPL_HPP_

