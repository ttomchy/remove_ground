//
// Created by hadoop on 17-4-13.
//

#ifndef DEPTH_CLUSTERING_SHOW_OBJECTS_MOOSMANN_H
#define DEPTH_CLUSTERING_SHOW_OBJECTS_MOOSMANN_H
#include<mutex>
struct test_define{
    cv::Mat key;
    int id;
};



static std::mutex m_mux;//全局互斥所
std::deque<test_define> image_buff;
std::deque<test_define> image_buff_before;
std::deque<test_define> image_buff_after;
const int size=100000;

//typedef
#endif //DEPTH_CLUSTERING_SHOW_OBJECTS_MOOSMANN_H
