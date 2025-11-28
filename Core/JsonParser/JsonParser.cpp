#include <JsonParser/JsonParser.hpp>

JsonParser::JsonParser(){

}

JsonParser::JsonParser(const std::string &filePath_)
{
    this->Init(filePath_);
}

void JsonParser::Init(const std::string &filePath_){
    this->filePath = filePath_;

    try {
        // read the file
        std::ifstream file(filePath);
        if(!file.is_open()){
            throw std::runtime_error("Failed to open : " + filePath);
        }
        std::ostringstream oss;
        oss << file.rdbuf();
        std::string jsonStr = oss.str();

        // parse the file
        this->rootValue = boost::json::parse(jsonStr);

        if(this->rootValue.is_object()){
           this->rootObject = this->rootValue.as_object();
        }else{
            std::cerr<<"[JsonParser] The root's type is not a object!"<<std::endl;
        }
    }  catch (const boost::json::system_error& e) {
        std::cerr<<"Failed to parse the json:"<<e.what()<<std::endl;
    }  catch (const std::exception &e) {
        std::cerr<<"Error:"<<e.what()<<std::endl;
    }
}

JsonParser::~JsonParser(){

}

Eigen::MatrixXd JsonParser::JsonArray2EigenMatrixXd(const json::array &array){
    if(array.empty()){
        std::cout<<"[JsonParser::JsonArray2EigenMatrixXd] The input array is empty!"<<std::endl;
        return Eigen::MatrixXd();
    }

    const size_t rows = static_cast<size_t>(array.size());
    const size_t cols = static_cast<size_t>(json::value_to<json::array>(array[0]).size());

    for(size_t i=0;i<array.size();i++){
        const auto& row = json::value_to<json::array>(array[i]);
        if(row.size() != static_cast<size_t>(cols)){
            throw std::invalid_argument("[JsonParser::JsonArray2EigenMatrixXd] All rows must be the same");
        }
    }

    Eigen::MatrixXd mat(rows,cols);
    for(size_t i=0;i<rows;i++){
        const auto& row = json::value_to<json::array>(array[i]);
        for(size_t j=0;j<cols;j++){
            mat(i,j) = json::value_to<double>(row[j]);
        }
    }

    return mat;
}

Eigen::VectorXd JsonParser::JsonArray2EigenVectorXd(const json::array &array){
    if(array.empty()){
        std::cout<<"[JsonParser::JsonArray2EigenVectorXd] The input array is empty!"<<std::endl;
        return Eigen::VectorXd();
    }

    const size_t size = static_cast<size_t>(array.size());

    Eigen::VectorXd vec(size);
    for(size_t i=0;i<size;i++){
        const auto& member = array[i];

        if(!member.is_double() && !member.is_int64() && !member.is_uint64()){
            throw std::invalid_argument("[JsonParser::JsonArray2EigenVectorXd] Invalid member of this array at"
                                        + std::to_string(i) + ", expected number! ");
        }

        try {
            vec(i) = json::value_to<double>(member);
        }  catch (const std::exception& e) {
            throw std::runtime_error("[JsonParser::JsonArray2EigenVectorXd] Failed to convert array["
                                     + std::to_string(i)
                                     + "] to double:"
                                     + e.what());
        }
    }

    return vec;
}

std::vector<std::string> JsonParser::JsonArray2StdVecStr(const json::array &array){
    if(array.empty()){
        std::cout<<"[JsonParser::JsonArray2StdVecStr] The input array is empty!"<<std::endl;
        return std::vector<std::string>();
    }

    std::vector<std::string> vec;
    vec.reserve(array.size());

    for(const auto& temp : array){
        if(!temp.is_string()){
            throw std::invalid_argument("[JsonParser::JsonArray2StdVecStr] The input array contains non-string element ");
        }
        vec.push_back(temp.as_string().c_str());
    }

    return vec;
}

std::vector<int> JsonParser::JsonArray2StdVecInt(const json::array &array){
    if(array.empty()){
        std::cout<<"[JsonParser::JsonArray2StdVecInt] The input array is empty!"<<std::endl;
        return std::vector<int>();
    }

    std::vector<int> vec;
    vec.reserve(array.size());

    for(size_t i = 0; i < array.size(); ++i){
        const auto& member = array[i];

        // 检查类型是否为 number（int64 / uint64 / double 都可）
        if(!member.is_double() && !member.is_int64() && !member.is_uint64()){
            throw std::invalid_argument(
                "[JsonParser::JsonArray2StdVecInt] Invalid member at index "
                + std::to_string(i) +
                ", expected number!"
            );
        }

        try {
            // 这里直接转换为 int
            vec.push_back(static_cast<int>(json::value_to<int>(member)));
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "[JsonParser::JsonArray2StdVecInt] Failed to convert array["
                + std::to_string(i)
                + "] to int: "
                + e.what()
            );
        }
    }

    return vec;
}
