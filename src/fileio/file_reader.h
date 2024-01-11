#pragma once

#include "fileio/file_base.h"

#include <fstream>
#include <memory>
#include <vector>

namespace MSC {

class FileReader : public FileBase {
public:
    typedef std::shared_ptr<FileReader> Ptr;

    FileReader() = delete;
    FileReader(const std::string &file, const int &skip_lines = 0, const int &columns = 7);
    ~FileReader();

    static Ptr create(const std::string &file, const int &skip_lines = 0, const int &columns = 7);

    bool load(std::vector<double> &data);
    bool loadn(std::vector<std::vector<double>> &data, const int &epochs);

private:
    std::fstream file_;
    int columns_; ///< 列数，仅在bin文件中有效
};

} // namespace MSC