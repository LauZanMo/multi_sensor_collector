#pragma once

#include "core/logger.h"

#include <absl/strings/str_split.h>
#include <string>

namespace MSC {

enum FileType {
    TXT = 0,
    BIN = 1,
};

class FileBase {
public:
    FileBase() = delete;
    FileBase(const std::string &file) {
        std::vector<absl::string_view> splits = absl::StrSplit(file, '.');
        if (splits.back() == "bin") {
            file_type_ = FileType::BIN;
        } else if (splits.back() == "txt" || splits.back() == "csv" || splits.back() == "nav") {
            file_type_ = FileType::TXT;
        } else
            LOGF << "Invalid file suffix: " << splits.back();
    }
    ~FileBase() = default;

protected:
    int file_type_;
};

} // namespace MSC