// FunctionLogger.hpp
#pragma once

#include <iostream>
#include <typeinfo>

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

// -------------------------
// 跨平台获取函数签名宏
// -------------------------
#if defined(__GNUC__) || defined(__clang__)
    #define FUNC_SIG __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
    #define FUNC_SIG __FUNCSIG__
#else
    #define FUNC_SIG __func__
#endif

// -------------------------
// RAII 日志类
// -------------------------
class FunctionLogger {
    std::string funcName;

    // 获取类名 + 函数名（跨平台）
    template<typename T>
    std::string getClassFuncName(T* obj, const char* func) {
        std::string className;

#if defined(__GNUC__) || defined(__clang__)
        int status;
        char* demangled = abi::__cxa_demangle(typeid(*obj).name(), nullptr, nullptr, &status);
        className = (status == 0) ? demangled : typeid(*obj).name();
        free(demangled);
#elif defined(_MSC_VER)
        className = typeid(*obj).name(); // MSVC 已返回可读类名
#else
        className = typeid(*obj).name();
#endif
        return className + "::" + func;
    }

public:
    // 构造函数：对象 + 函数名
    template<typename T>
    FunctionLogger(T* obj, const char* func) {
        funcName = getClassFuncName(obj, func);
        std::cout << "\n[ENTER] " << funcName << std::endl;
    }

    // 对于非类函数，也可以直接传函数名
    FunctionLogger(const char* func) : funcName(func) {
        std::cout << "\n[ENTER] " << funcName << std::endl;
    }

    ~FunctionLogger() {
        std::cout << "[EXIT] " << funcName << "\n" << std::endl;
    }
};

// -------------------------
// 宏定义：在函数内部直接使用
// -------------------------
#define LOG_FUNCTION FunctionLogger _log(this, FUNC_SIG)
#define LOG_FUNCTION_STATIC FunctionLogger _log(FUNC_SIG)
