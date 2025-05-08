// Copyright 2022 Chen Jun
// Copyright 2025 Boombroke
// Licensed under the MIT License.

#ifndef RM_OMNI_PERCEPTION_NUMBER_CLASSIFIER_HPP
#define RM_OMNI_PERCEPTION_NUMBER_CLASSIFIER_HPP

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "rm_omni_perception/armor.hpp"

namespace rm_omni_perception {
    
class NumberClassifier
{
public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void classify(std::vector<Armor> & armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

} // namespace rm_omni_perception
#endif // RM_OMNI_PERCEPTION_NUMBER_CLASSIFIER_HPP  