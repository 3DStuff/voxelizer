#pragma once

#include <chrono>
#include <iostream>


namespace benchmark {
    class timer {
    private:
        std::string _info_message = "";
        std::chrono::time_point<std::chrono::steady_clock> _beg;
    
    public:
        timer(const std::string info = "") : _info_message(info) 
        {
            _beg = std::chrono::steady_clock::now();
        }
        
        ~timer()
        {
            auto end = std::chrono::steady_clock::now();
            std::cout << _info_message << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(end-_beg).count() << " ms" << std::endl;
        }

        void reset() {
            auto end = std::chrono::steady_clock::now();
            std::cout << _info_message << ": " << std::chrono::duration_cast<std::chrono::milliseconds>(end-_beg).count() << " ms" << std::endl;
            _beg = end;
        }
    };
};
