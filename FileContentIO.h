#pragma once

#include <fstream>
#include <string>
#include <string_view>
#include <memory>

struct FileContentIO {
    std::string path;

    FileContentIO(std::string path) : path(std::move(path)) {}

    std::string getContent() const {
        std::string content;
        std::ifstream ifs(path);
        std::copy(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>(), std::back_inserter(content));
        return content;
    }

    void putContent(std::string_view content) const {
        std::ofstream ofs(path);
        std::copy(content.begin(), content.end(), std::ostreambuf_iterator<char>(ofs));
    }

    std::unique_ptr<std::ifstream> readStream() const {
        return std::make_unique<std::ifstream>(path);
    }

    std::unique_ptr<std::ofstream> writeStream() const {
        return std::make_unique<std::ofstream>(path);
    }
};
