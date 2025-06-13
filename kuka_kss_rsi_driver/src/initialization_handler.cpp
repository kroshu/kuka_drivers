// Copyright 2025 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>
#include <regex>

#include "kuka_kss_rsi_driver/initialization_handler.hpp"

namespace kuka_kss_rsi_driver
{

std::string FindRobotModelInUrdfName(const std::string & input)
{
  // Regex pattern to match robot model names
  std::regex pattern(
    "kr\\d+_r\\d+_[a-z\\d]*(?=\\b)",
    std::regex_constants::ECMAScript | std::regex_constants::icase);
  std::smatch match;
  std::string last_match;
  auto search_start = input.cbegin();
  while (std::regex_search(search_start, input.cend(), match, pattern))
  {
    last_match = match.str();
    search_start = match.suffix().first;
  }

  std::transform(last_match.begin(), last_match.end(), last_match.begin(), ::tolower);
  last_match.erase(std::remove(last_match.begin(), last_match.end(), '_'), last_match.end());
  return last_match;
}

std::string ProcessKrcReportedRobotName(const std::string & input)
{
  std::string trimmed = input.substr(1);
  std::size_t space_pos = trimmed.find(' ');
  std::string robot_model = trimmed.substr(0, space_pos);
  std::transform(robot_model.begin(), robot_model.end(), robot_model.begin(), ::tolower);
  robot_model.erase(std::remove(robot_model.begin(), robot_model.end(), '_'), robot_model.end());
  return robot_model;
}

void EkiInitializationHandler::Initialize(
  const kuka::external::control::kss::InitializationData & init_data)
{
  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    auto expected = FindRobotModelInUrdfName(hardware_info_.hardware_parameters["robot_name"]);
    auto reported = ProcessKrcReportedRobotName(init_data.model_name);
    if (
      expected.find(reported) == std::string::npos && reported.find(expected) == std::string::npos)
    {
      std::ostringstream oss;
      oss << "Robot model mismatch detected: expected model '" << expected << "', but received '"
          << reported << "'";
      init_report_ = {true, false, oss.str()};
      return;
    }

    if (hardware_info_.joints.size() != init_data.GetTotalAxisCount())
    {
      std::ostringstream oss;
      oss << "Mismatch in axis count: Driver expects " << hardware_info_.joints.size()
          << ", but EKI server reported " << init_data.GetTotalAxisCount();
      init_report_ = {true, false, oss.str()};
      return;
    }

    init_report_ = {true, true, ""};
  }
  init_cv_.notify_one();
}

void MxaInitializationHandler::Initialize(const kuka::external::control::kss::InitializationData &)
{
  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    init_report_ = {true, true, ""};
  }
  init_cv_.notify_one();
}

}  // namespace kuka_kss_rsi_driver
