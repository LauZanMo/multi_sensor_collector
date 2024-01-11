#include "fileio/file_writer.h"

#include <absl/strings/str_format.h>
#include <absl/strings/str_join.h>
#include <absl/strings/string_view.h>

namespace MSC {

FileWriter::FileWriter(const std::string &file, const bool &auto_flush, const std::string &separator)
    : FileBase(file)
    , auto_flush_(auto_flush)
    , separator_(separator) {
    auto mode = std::ios::out | std::ios::trunc;
    if (file_type_ == FileType::BIN) {
        mode |= std::ios::binary;
    }

    file_.open(file, mode);
    CHECK(file_.is_open()) << "Failed to open file: " << file;
}

FileWriter::~FileWriter() {
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

FileWriter::Ptr FileWriter::create(const std::string &file, const bool &auto_flush, const std::string &separator) {
    CHECK(!file.empty()) << "Write file name empty!";
    return std::make_shared<FileWriter>(file, auto_flush, separator);
}

void FileWriter::dump(const std::string &data) {
    CHECK(file_type_ == FileType::TXT) << "File type does not support string dump!";
    file_ << data << "\n";
    if (auto_flush_)
        file_.flush();
}

void FileWriter::dump(const std::vector<double> &data) {
    if (file_type_ == FileType::TXT) {
        constexpr absl::string_view format = "%-15.9lf";
        auto line                          = absl::StrFormat(format, data[0]);
        for (size_t k = 1; k < data.size(); k++) {
            absl::StrAppend(&line, separator_);
            absl::StrAppendFormat(&line, format, data[k]);
        }
        absl::StripTrailingAsciiWhitespace(&line);
        dump(line);
    } else {
        file_.write(reinterpret_cast<const char *>(data.data()), sizeof(double) * data.size());
        if (auto_flush_)
            file_.flush();
    }
}

} // namespace MSC