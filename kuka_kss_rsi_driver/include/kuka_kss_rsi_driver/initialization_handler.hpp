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

#ifndef KUKA_KSS_RSI_DRIVER__INITIALIZATION_HANDLER_HPP_
#define KUKA_KSS_RSI_DRIVER__INITIALIZATION_HANDLER_HPP_

#include <condition_variable>
#include <hardware_interface/hardware_info.hpp>

#include "kuka/external-control-sdk/kss/extension.h"

namespace kuka_kss_rsi_driver
{

struct InitSequenceReport
{
  bool sequence_complete = false;
  bool ok = false;
  std::string reason = "";
};

class IInitializationHandler
{
public:
  virtual ~IInitializationHandler() = default;

  virtual void Initialize(const kuka::external::control::kss::InitializationData & init_data) = 0;
};

class EkiInitializationHandler : public IInitializationHandler
{
public:
  EkiInitializationHandler(
    InitSequenceReport & init_report, std::mutex & init_mtx, std::condition_variable & init_cv,
    hardware_interface::HardwareInfo & hardware_info)
  : init_report_(init_report), init_mtx_(init_mtx), init_cv_(init_cv), hardware_info_(hardware_info)
  {
  }

  void Initialize(const kuka::external::control::kss::InitializationData & init_data) override;

private:
  InitSequenceReport & init_report_;
  std::mutex & init_mtx_;
  std::condition_variable & init_cv_;
  hardware_interface::HardwareInfo & hardware_info_;
};

class MxaInitializationHandler : public IInitializationHandler
{
public:
  MxaInitializationHandler(
    InitSequenceReport & init_report, std::mutex & init_mtx, std::condition_variable & init_cv)
  : init_report_(init_report), init_mtx_(init_mtx), init_cv_(init_cv)
  {
  }

  void Initialize(const kuka::external::control::kss::InitializationData &) override;

private:
  InitSequenceReport & init_report_;
  std::mutex & init_mtx_;
  std::condition_variable & init_cv_;
};

}  // namespace kuka_kss_rsi_driver

#endif  // KUKA_KSS_RSI_DRIVER__INITIALIZATION_HANDLER_HPP_
