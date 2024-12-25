#pragma once

#include "VirtualSensor.h"
#include <opencv2/opencv.hpp>
#include <vector>

class DepthProcessor {
public:
  /**
   * Applies bilateral filtering to smooth the depth map while preserving edges.
   * @param depthMap Input depth map (1-channel float, with invalid values as
   * MINF).
   * @param diameter Diameter of the pixel neighborhood used for filtering.
   * @param sigmaColor Filter sigma in color space (affects range of colors
   * considered).
   * @param sigmaSpace Filter sigma in coordinate space (affects range of pixels
   * considered).
   * @return Filtered depth map.
   */
  static cv::Mat BilateralFilter(const cv::Mat &depthMap, int diameter,
                                 double sigmaColor, double sigmaSpace) {
    // Convert MINF to 0 for OpenCV processing
    cv::Mat validDepth = depthMap.clone();
    validDepth.setTo(0, validDepth == MINF);

    // Apply bilateral filter
    cv::Mat filteredDepth;
    cv::bilateralFilter(validDepth, filteredDepth, diameter, sigmaColor,
                        sigmaSpace);

    // Restore MINF for invalid values
    filteredDepth.setTo(MINF, depthMap == MINF);
    return filteredDepth;
  }

  /**
   * Replaces invalid depth values (MINF) with interpolated values based on
   * surrounding pixels.
   * @param depthMap Input depth map (1-channel float, with invalid values as
   * MINF).
   * @return Depth map with invalid values filled.
   */
  static cv::Mat FillInvalidDepth(const cv::Mat &depthMap) {
    cv::Mat filledDepth = depthMap.clone();

    // Replace invalid values with a large number temporarily
    cv::Mat mask = (filledDepth == MINF);
    filledDepth.setTo(1000.0, mask);

    // Inpaint the invalid regions
    cv::Mat inpaintedDepth;
    cv::inpaint(filledDepth, mask, inpaintedDepth, 3, cv::INPAINT_TELEA);

    // Restore MINF for invalid values
    inpaintedDepth.setTo(MINF, mask);
    return inpaintedDepth;
  }

  /**
   * Normalizes the depth map to a [0, 1] range for visualization.
   * @param depthMap Input depth map (1-channel float, with invalid values as
   * MINF).
   * @return Normalized depth map.
   */
  static cv::Mat NormalizeDepth(const cv::Mat &depthMap) {
    cv::Mat validDepth = depthMap.clone();
    validDepth.setTo(0, validDepth == MINF); // Replace invalid values with 0

    double minVal, maxVal;
    cv::minMaxLoc(validDepth, &minVal, &maxVal, nullptr, nullptr,
                  validDepth > 0);

    cv::Mat normalizedDepth;
    if (maxVal > minVal) {
      normalizedDepth = (validDepth - minVal) / (maxVal - minVal);
    } else {
      normalizedDepth = cv::Mat::zeros(validDepth.size(), CV_32F);
    }

    return normalizedDepth;
  }
};

#endif // DEPTH_PROCESSOR_H
