#include "config.h"

Config::Config() : N(-1), searchParams(nullptr) {}

Config::Config(int numParams, float *paramArray) {
    N = numParams;
    searchParams = new float[N];

    for(int i = 0; i < N; ++i) {
       searchParams[i] = paramArray[i];
    }
}

Config::~Config() {
    if (searchParams) {
        delete[] searchParams;
    }
}

float Config::getParamValue(int i) const {
    return searchParams[i];
}

bool Config::getConfig() {
    std::string value;
    float angle;
    int distance;
    float weight;
    unsigned int steplimit;
    float loglevel;
    float curvatureHeuriscitWeight;
    double pivotRadius = 0;
    float decreaseDistance;
    int distanceMin;
    int numOfParentsToIncreaseRadius;
    bool postsmoother;
    
    // load from ros params
    return true;
}
