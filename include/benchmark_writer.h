#pragma once 
#include <string>
#include <sstream>
#include <fstream>

class BenchMarkWriter {
public:
    BenchMarkWriter() = delete;
    BenchMarkWriter(std::string path) : mPath(path) {}
    float mTimeLoadMesh = -1;
    float mTimeBuildOctree = -1;
    float mTimeCutMesh = -1;
    float mTimeCategorization = -1;
    float mTimeComplete = -1;
    std::string mPath = "";

    void writeToFile() {
        std::ofstream file;
        file.open(mPath);
        file << getJsonString();
        file.close();
    }

    std::string getJsonString() {
        std::stringstream ss;
        ss << "{" << std::endl;
        ss << "\"TimeLoadMesh\": " << mTimeLoadMesh << ", " << std::endl;
        ss << "\"TimeBuildOctree\": " << mTimeBuildOctree << ", " << std::endl;
        ss << "\"TimeCutMesh\": " << mTimeCutMesh << ", " << std::endl;
        ss << "\"TimeCategorization\": " << mTimeCategorization << ", " << std::endl;
        ss << "\"TimeComplete\": " << mTimeComplete << std::endl;
        ss << "}";
        return ss.str();
    }
};