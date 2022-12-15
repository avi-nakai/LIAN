#ifndef MISSION_H
#define MISSION_H

#include "config.h"
#include "liansearch.h"
#include "map.h"
#include "search.h"
#include "searchresult.h"
#include "xmllogger.h"
#include "nav_msgs/msg/Path.hpp"
#include "geometry_msgs/PoseStamped.hpp"

#include <string>

class Mission {

public:
    Mission(const char* fName);
    ~Mission();

    bool getMap();
    bool getConfig();
    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();
    void saveSearchResultToPathMsg();
private:
    Map         map;
    Config      config;

    Search      *search;
    Logger      *logger;

    const char* fileName;

    SearchResult sr;
};

#endif

