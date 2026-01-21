#include <iostream>
#include <cstring>
#include <FunctionLogger.hpp>
#include <DataCollector/VisionProCollector.hpp>



int main() {
    std::string address = "tcp://127.0.0.1:5555";
    auto visionProCollector = VisionProCollector(address);
    visionProCollector.run();
    return 0;
}
