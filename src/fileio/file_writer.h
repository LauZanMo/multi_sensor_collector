#pragma once

#include "fileio/file_base.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace MSC {

class FileWriter : public FileBase {
public:
    typedef std::shared_ptr<FileWriter> Ptr;

    FileWriter() = delete;
    FileWriter(const std::string &file, const bool &auto_flush = true, const std::string &separator = " ");
    ~FileWriter();

    static Ptr create(const std::string &file, const bool &auto_flush = true, const std::string &separator = " ");

    void dump(const std::string &data);
    void dump(const std::vector<double> &data);

    void flush() {
        file_.flush();
    }

private:
    std::fstream file_;
    bool auto_flush_;
    std::string separator_; ///< 分隔符，仅在txt文件中有效
};

} // namespace MSC