#include "mission.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/Path.hpp"
#include "geometry_msgs/PoseStamped.hpp"
#include "tf2_ros.h"
#include <list>

Mission::Mission(const char* fName) : fileName(fName), search(nullptr), logger(nullptr) {}

Mission::~Mission() {
    delete search;
    delete logger;
}

bool Mission::getMap() {
    return map.getMap(fileName);
}

bool Mission::getConfig() {
    return config.getConfig(fileName);
}

void Mission::createSearch() {
    search = new LianSearch((float)config.getParamValue(CN_PT_AL),
                            (int)config.getParamValue(CN_PT_D),
                            (float)config.getParamValue(CN_PT_W),
                            (unsigned int)config.getParamValue(CN_PT_SL),
                            (float)config.getParamValue(CN_PT_CHW),
                            (bool)config.getParamValue(CN_PT_PS),
                            (float)config.getParamValue(CN_PT_DDF),
                            (int)config.getParamValue(CN_PT_DM),
                            (double)config.getParamValue(CN_PT_PC),
                            (int)config.getParamValue(CN_PT_NOP));
}

bool Mission::createLog() {
    if(config.getParamValue(CN_PT_LOGLVL) == CN_LOGLVL_LOW || config.getParamValue(CN_PT_LOGLVL) == CN_LOGLVL_HIGH ||
       config.getParamValue(CN_PT_LOGLVL) == CN_LOGLVL_MED || config.getParamValue(CN_PT_LOGLVL) == CN_LOGLVL_TINY ||
       config.getParamValue(CN_PT_LOGLVL) - CN_LOGLVL_ITER < 0.001) {
        logger = new XmlLogger(config.getParamValue(CN_PT_LOGLVL));
    } else if(config.getParamValue(CN_PT_LOGLVL) == CN_LOGLVL_NO) {
        logger = new XmlLogger(config.getParamValue(CN_PT_LOGLVL));
        return true;
    } else {
        std::cout << "'loglevel' is not correctly specified in input XML-file.\n";
        return false;
    }
    return logger->getLog(fileName);
}

void Mission::startSearch() {
    sr = search->startSearch(logger, map);
}

void Mission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "length_scaled=" << sr.pathlength * map.getCellSize() << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    logger->writeToLogSummary(sr.hppath, sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.pathlength * map.getCellSize(),
                              sr.time, sr.max_angle, sr.accum_angle, sr.sections);

    if (sr.pathfound) {
        logger->writeToLogPath(sr.lppath, sr.angles);
        logger->writeToLogMap(map,sr.lppath);
        logger->writeToLogHpLevel(sr.hppath);
    }
    logger->saveLog();
}

void Mission::saveSearchResultToPathMsg(){
    if (sr.pathfound) {

        nav_msgs::msg::Path lian_path;
        lian_path.header.stamp = node_->now();
        lian_path.header.frame_id = map;

        for(int node=0; node<len(sr.hppath);node++){

            geometry_msgs::msg::PoseStamped posestamped;

            tf2::Quaternion nodeQuaternion;
            nodeQuaternion.setRPY(0,0,sr.hppath[node].angle);

            posestamped.pose.position.x = sr.hppath[node].i;
            posestamped.pose.position.y = sr.hppath[node].j;
            posestamped.pose.orientation.z = nodeQuaternion.getZ();
            posestamped.pose.orientation.w = nodeQuaternion.getW();
            posestamped.header.stamp = node_->now();
            posestamped.header.frame_id = map;

            lian_path.push_back(posestamped);
        }     
    }

}