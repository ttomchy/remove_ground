
#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <condition_variable>
#include <unistd.h>
#include "./depth_ground_remover.h"
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <algorithm>
#include<iostream>
#include <iterator>
#include <vector>
#include "utils/velodyne_utils.h"
#include "image_labelers/linear_image_labeler.h"
#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "utils/timer.h"


std::deque<int> m_data;
std::mutex m_mux;//全局互斥所
std::condition_variable m_cv;

void consume_thread(){

    sleep(10);

    while (true){

        std::unique_lock<std::mutex> lck(m_mux);
        if(m_data.empty()) {
            std::cout << "empty" << std::endl;
            lck.unlock();
            continue;
        }

        printf("consume %d\n", m_data.back());
        m_data.pop_back();
        lck.unlock();
    }
}

int main() {

    m_data= {1, 3, 5, 7,9,11,13,15,17,19,21,23,25};

    std::thread thread (&consume_thread);
    thread.join();
    //_counter ++;
    //std::cout<<"the value of _counter is "<<_counter<<std::endl;
    return  0;
}