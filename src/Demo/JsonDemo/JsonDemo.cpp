#include <JsonParser/JsonParser.hpp>
#include <source_path.h>

int main(int argc, char const *argv[])
{
    std::cout<<SOURCE_FILE_PATH<<std::endl;

    std::string filePath = static_cast<std::string>(SOURCE_FILE_PATH) + "/config/Teleoperate/Ti5Robot.json";

    JsonParser jsonParser(filePath);

    json::object temp = jsonParser.GetJsonObject();

    json::array baseOffset = temp["SolverConfig"].as_object()["BaseOffset"].as_array()[0].as_array();
//    std::cout<<baseOffset<<std::endl;
    std::cout<<
                JsonParser::JsonArray2EigenMatrixXd(baseOffset)
            <<std::endl;

    return 0;
}
