#include <pybind11/embed.h>
#include <iostream>
#include <source_path.h>
#include <FunctionLogger.hpp>
namespace py = pybind11;

class PybindDemo
{
public:
    PybindDemo(){}
    ~PybindDemo(){}

    void Demo(){
        try {
            py::eval_file(televuerModulePath+"/vuer_demo.py");
        }
        catch (const py::error_already_set &e) {
            std::cerr << "Python 出错: " << e.what() << std::endl;
        }
    }
private:
    void LoadModule(){

    }

    // module
    py::module_ televuer;

    // path
    const std::string sourcePath = SOURCE_FILE_PATH;
    const std::string televuerModulePath = sourcePath + "/utils/televuer/scripts";

};


int main() {

}


