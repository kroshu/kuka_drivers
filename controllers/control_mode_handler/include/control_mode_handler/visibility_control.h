//    Copyright 2023 Aron Svastits
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef CONTROL_MODE_HANDLER__VISIBILITY_CONTROL_H_
#define CONTROL_MODE_HANDLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CONTROL_MODE_HANDLER_EXPORT __attribute__((dllexport))
#define CONTROL_MODE_HANDLER_IMPORT __attribute__((dllimport))
#else
#define CONTROL_MODE_HANDLER_EXPORT __declspec(dllexport)
#define CONTROL_MODE_HANDLER_IMPORT __declspec(dllimport)
#endif
#ifdef CONTROL_MODE_HANDLER_BUILDING_LIBRARY
#define CONTROL_MODE_HANDLER_PUBLIC CONTROL_MODE_HANDLER_EXPORT
#else
#define CONTROL_MODE_HANDLER_PUBLIC CONTROL_MODE_HANDLER_IMPORT
#endif
#define CONTROL_MODE_HANDLER_PUBLIC_TYPE CONTROL_MODE_HANDLER_PUBLIC
#define CONTROL_MODE_HANDLER_LOCAL
#else
#define CONTROL_MODE_HANDLER_EXPORT __attribute__((visibility("default")))
#define CONTROL_MODE_HANDLER_IMPORT
#if __GNUC__ >= 4
#define CONTROL_MODE_HANDLER_PUBLIC __attribute__((visibility("default")))
#define CONTROL_MODE_HANDLER_LOCAL __attribute__((visibility("hidden")))
#else
#define CONTROL_MODE_HANDLER_PUBLIC
#define CONTROL_MODE_HANDLER_LOCAL
#endif
#define CONTROL_MODE_HANDLER_PUBLIC_TYPE
#endif

#endif  // CONTROL_MODE_HANDLER__VISIBILITY_CONTROL_H_
