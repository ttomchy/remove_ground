
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>

#include <qapplication.h>

#include <string>
#include <thread>
#include <ctime>
#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
#include "visualization/visualizer.h"
#include<mutex>
#include "tclap/CmdLine.h"
#include <deque>
#include <unistd.h>
#include "show_objects_moosmann.h"
#include <cstdlib>
test_define no_gnd_img,read_depth_img;

using std::string;
using std::to_string;
int i_depth_image=0;


int i_no_ground_before=0;
int i_no_ground_produced=0;
char image_name_consumer[6000];
int i_no_ground_consumer=0;

char image_name[size];
using namespace depth_clustering;

int min_cluster_size = 20;
int max_cluster_size = 100000;

int smooth_window_size = 5;
Radians ground_remove_angle = 9_deg;
//static std::unique_lock <std::mutex> lck(m_mux);
void ReadData(const Radians& angle_tollerance, const string& in_path /*,
              Visualizer* visualizer*/) {
    // delay reading for one second to allow GUI to load

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // now load the data
    std::cout<<" Now it runs in the product_thread OnNewObjectReceived function  in the ground_remove.cpp!"<<std::endl;

    fprintf(stderr, "INFO: running on Moosman data\n");

    auto image_reader =
            FolderReader(in_path, ".png", FolderReader::Order::SORTED);
    auto config_reader = FolderReader(in_path, "img.cfg");

    auto proj_params_ptr =
            ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

    for (const auto& path : image_reader.GetAllFilePaths()) {
       // std::unique_lock <std::mutex> lck(m_mux);
      //  lck.lock();
        m_mux.lock();
        auto depth_image = MatFromDepthPng(path);

        no_gnd_img.key=depth_image;
        image_buff.push_front(no_gnd_img);


        printf("product one image \n");
        printf("the size of queue %d\n",image_buff.size());
      //  sprintf(image_name, "%s%d%s", "depth_image_buff_produced", ++i_no_ground_produced, ".png");//保存的图片名
      //  cv::imwrite(image_name,no_gnd_img.key);
        m_mux.unlock();
        srand(time(0));
        sleep(rand()%3+1);
        printf("the value of the product_srand(time(0)) %d \n",rand()%10+1);


    }

}
void consume_thread(const Radians& angle_tollerance,const string& in_path ,Visualizer* visualizer){

    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_mux.lock();
    auto image_reader =
            FolderReader(in_path, ".png", FolderReader::Order::SORTED);
    auto config_reader = FolderReader(in_path, "img.cfg");
    auto proj_params_ptr =
            ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

    auto depth_ground_remover = DepthGroundRemover(
            *proj_params_ptr, ground_remove_angle, smooth_window_size);

    ImageBasedClusterer<LinearImageLabeler<>> clusterer(
            angle_tollerance, min_cluster_size, max_cluster_size);
    clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

    depth_ground_remover.AddClient(&clusterer);
    clusterer.AddClient(visualizer->object_clouds_client());



    while (true) {
        if (image_buff.empty()) {

            m_mux.unlock();
            continue;
        }
        read_depth_img = image_buff.back();
        printf("consume one image \n");

        sprintf(image_name_consumer, "%s%d%s", "depth_image_buff_consumer", ++i_no_ground_consumer, ".png");//保存的图片名
        cv::imwrite(image_name_consumer,read_depth_img.key);
        image_buff.pop_back();
        printf("the size of queue %d\n",image_buff.size());
        m_mux.unlock();
        srand(time(0));
        sleep(rand()%5+1);
        printf("the value of the consumer_srand(time(0)) %d \n",rand()%10+1);
/*
        auto cloud_ptr = Cloud::FromImage(read_depth_img.key, *proj_params_ptr);

        time_utils::Timer timer;

        visualizer->OnNewObjectReceived(*cloud_ptr, 0);
        depth_ground_remover.OnNewObjectReceived(*cloud_ptr, 0);



        auto current_millis = timer.measure(time_utils::Timer::Units::Milli);
        fprintf(stderr, "INFO: It took %lu ms to process and show everything.\n",
                current_millis);
        uint max_wait_time = 100;
        if (current_millis > max_wait_time) {
            continue;
        }
        auto time_to_wait = max_wait_time - current_millis;
        fprintf(stderr, "INFO: Waiting another %lu ms.\n", time_to_wait);

*/
    }
    i_no_ground_consumer=0;

}

double random(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

int main(int argc, char* argv[]) {
    TCLAP::CmdLine cmd(
            "Loads clouds from Frank Moosmann's data and clusters each of them.", ' ',
            "1.0");
    TCLAP::ValueArg<int> angle_arg(
            "", "angle",
            "Threshold angle. Below this value, the objects are separated", false, 10,
            "int");
    TCLAP::ValueArg<string> path_to_data_arg(
            "", "path", "Path to folder that stores the data", true, "", "string");

    cmd.add(angle_arg);
    cmd.add(path_to_data_arg);
    cmd.parse(argc, argv);

    Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());
    string in_path = path_to_data_arg.getValue();
    fprintf(stderr, "INFO: Reading from: %s \n", in_path.c_str());





    QApplication application(argc, argv);
    // visualizer should be created from a gui thread
    Visualizer visualizer;
    visualizer.show();

    // create and run loader thread
    std::cout<<" Now it runs before the load_thread ！"<<std::endl;
    std::thread loader_thread(ReadData , angle_tollerance, in_path /*, &visualizer*/);
    std::cout<<" Now it runs after the load_thread ！"<<std::endl;
    std::thread process_img(consume_thread, angle_tollerance,in_path,&visualizer);

    // if we close the qt application we will be here
    auto exit_code = application.exec();

    // join thread after the application is dead
    loader_thread.join();
    process_img.join();
    return exit_code;
}