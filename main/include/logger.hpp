//Modified version of work started by Soulier and Tratnik - https://github.com/jsoulier/wildfire_simulator 

#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <utility>
#include <chrono>
#include <cadmium/core/logger/logger.hpp>

class Logger : public cadmium::Logger
{
public:

    Logger(const std::string& filepath,  std::chrono::system_clock::time_point start_time)
        : cadmium::Logger(), startTime(start_time)
        , filepath(filepath)
        , file() {}

    void start() override
    {
        file.open(filepath);
        //set column titles
        file << "time,x,y,ignited,willignite,simtime,ignitionTime,sigma,spreadDir,rateOfSpread" << std::endl;
    }

    void stop() override
    {
        file.close();
    }
    //override the normal logging outputs
    void logOutput(double time, long modelId, const std::string& modelName, const std::string& portName, const std::string& output) override {}
    void logState(double time, long modelId, const std::string& modelName, const std::string& state) override
    {
        //pull all of the data from the logging stream
        std::istringstream stream(state);
        std::string x, y, ignited, willIgnite, ignitionTime, sigma, spreadDir, rateOfSpread;
        std::getline(stream, x, ':');
        std::getline(stream, y, ':');
        std::getline(stream, ignited, ':');
        std::getline(stream, willIgnite, ':');
        std::getline(stream, ignitionTime, ':');
        std::getline(stream, sigma, ':');
        std::getline(stream, spreadDir, ':');
        std::getline(stream, rateOfSpread, ':');
        
        // if nothing has happened, do not make a log entry
        if (!std::stoi(ignited)&!std::stoi(willIgnite))
        {
            return;
        }
        using namespace std::chrono;
        
        //calculate the time 
        auto elapsed = duration_cast<system_clock::duration>(duration<double>(time));
        const std::time_t now = system_clock::to_time_t(startTime + elapsed);
        
        //write to the log file
        file << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
        file << ",";
        file << x;
        file << ",";
        file << y;
        file << ",";
        file << ignited;
        file << ",";
        file << willIgnite;
        file << ",";
        file << time;
        file << ",";
        file << ignitionTime;
        file << ",";
        file << sigma;
        file << ",";
        file << spreadDir;
        file << ",";
        file << rateOfSpread;
        file << std::endl;
    }

private:
    std::string filepath;
    std::ofstream file;
    std::chrono::system_clock::time_point startTime;
};