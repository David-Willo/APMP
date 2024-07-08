#pragma once
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

namespace xslam {
#define INFOLOG(args...) LOG(INFO) << fmt::format(args);

#define WARNLOG(args...) LOG(WARNING) << fmt::format(args);

#define ERRLOG(args...) LOG(ERROR) << fmt::format(args);

#define FATALLOG(args...) LOG(FATAL) << fmt::format(args);
}  // namespace xslam
