#include "fileio/file_reader.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_split.h>

namespace MSC {

FileReader::FileReader(const std::string &file, const int &skip_lines, const int &columns)
    : FileBase(file)
    , columns_(columns) {
    auto mode = std::ios::in;
    if (file_type_ == FileType::BIN)
        mode |= std::ios::binary;

    file_.open(file, mode);
    CHECK(file_.is_open()) << "Failed to open file: " << file;

    if (file_type_ == FileType::TXT) {
        for (int i = 0; i < skip_lines; ++i)
            file_.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 跳过前n行
    } else {
        std::vector<std::vector<double>> data;
        loadn(data, skip_lines);
    }
}

FileReader::~FileReader() {
    if (file_.is_open())
        file_.close();
}

FileReader::Ptr FileReader::create(const std::string &file, const int &skip_lines, const int &columns) {
    CHECK(!file.empty()) << "Read file name empty!";
    return std::make_shared<FileReader>(file, skip_lines, columns);
}

bool FileReader::load(std::vector<double> &data) {
    if (file_.eof())
        return false;

    if (file_type_ == FileType::TXT) {
        std::string line;
        std::getline(file_, line);
        if (line.empty())
            return false;

        std::vector<absl::string_view> splits = absl::StrSplit(line, absl::ByAnyChar(", \t"), absl::SkipWhitespace());

        data.resize(splits.size());
        for (size_t i = 0; i < splits.size(); ++i)
            absl::SimpleAtod(splits[i], &data[i]);

    } else if (file_type_ == FileType::BIN) {
        data.resize(columns_);
        file_.read((char *) data.data(), columns_ * sizeof(double));
    }
    return true;
}

bool FileReader::loadn(std::vector<std::vector<double>> &data, const int &epochs) {
    data.resize(epochs);
    for (int i = 0; i < epochs; ++i) {
        if (!load(data[i])) {
            data.resize(i);
            return false;
        }
    }
    return true;
}

} // namespace MSC