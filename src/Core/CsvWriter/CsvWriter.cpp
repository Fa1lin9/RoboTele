#include <CsvWriter/CsvWriter.hpp>

CsvWriter::CsvWriter(){
    this->savePath = std::string(SOURCE_FILE_PATH) + "/log/" + this->GetDateStamp();
    std::string fileName = this->savePath + "/" + this->GetTimestamp() + ".csv";
    OpenFile(fileName);
}

CsvWriter::CsvWriter(const std::string& fileName){
    this->savePath = std::string(SOURCE_FILE_PATH) + "/log/" + this->GetDateStamp();
    std::string fileName_ = this->savePath + "/" + fileName + ".csv";
    OpenFile(fileName_);
}

CsvWriter::~CsvWriter(){
    if (ofs_.is_open()) {
        ofs_.close();
    }
}

void CsvWriter::WriteEigenVector(const Eigen::VectorXd& vec) {
    for (size_t i = 0; i < vec.size(); ++i) {
        ofs_ << vec[i];
        if (i != vec.size() - 1) ofs_ << ",";
    }
    ofs_ << "\n";
    ofs_.flush();
//    ofs_.close();
}

void CsvWriter::OpenFile(const std::string& filename) {
    // 确保目录存在
    std::filesystem::create_directories(this->savePath);

    ofs_.open(filename, std::ios::out | std::ios::app);
    if (!ofs_.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
}

std::string CsvWriter::GetTimestamp() {
    std::time_t now = std::time(nullptr);
    std::tm tmNow{};
#ifdef _WIN32
    localtime_s(&tmNow, &now);
#else
    localtime_r(&now, &tmNow);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tmNow);
    return buf;
}

std::string CsvWriter::GetDateStamp() {
    std::time_t now = std::time(nullptr);
    std::tm tmNow{};
#ifdef _WIN32
    localtime_s(&tmNow, &now);
#else
    localtime_r(&now, &tmNow);
#endif
    char buf[16];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tmNow);
    return buf;
}
