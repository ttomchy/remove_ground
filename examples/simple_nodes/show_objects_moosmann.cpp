
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
#include <mutex>
#include "tclap/CmdLine.h"
#include <deque>
#include <unistd.h>
#include "show_objects_moosmann.h"
#include <cstdlib>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>

test_define no_gnd_img,read_depth_img ;

test_define read_origin_gnd_remove_depth_img ,read_me_gnd_remove_depth_img;
test_define no_ground_img;
using std::string;
using std::to_string;
int i_depth_image=0;

int i_no_ground_before=0;
int i_no_ground_produced=0;
char image_name_consumer[6000];
int i_no_ground_consumer=0;

int i_origin_remove_gnd_img=0;
int i_me_remove_gnd_img=0;

char image_name[size];
using namespace depth_clustering;

int min_cluster_size = 20;
int max_cluster_size = 100000;

int scan_i=0;
char scan_num[6000];
int smooth_window_size = 5;
Radians ground_remove_angle = 9_deg;

void ReadData(const Radians& angle_tollerance, const string& in_path
             ) {
    // delay reading for one second to allow GUI to load

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // now load the data
    std::cout<<" Now it runs in the product_thread OnNewObjectReceived function  in the ground_remove.cpp!"<<std::endl;

    fprintf(stderr, "INFO: running on Moosman data\n");
   // sleep(2);
    auto image_reader =
            FolderReader(in_path, ".png", FolderReader::Order::SORTED);
    auto config_reader = FolderReader(in_path, "img.cfg");

    auto proj_params_ptr =
            ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

    for (const auto& path : image_reader.GetAllFilePaths()) {\

        m_mux.lock();
        auto depth_image = MatFromDepthPng(path);
        no_gnd_img.key=depth_image;
        image_buff.push_front(no_gnd_img);

        int image_num=image_buff.size();

        ROS_WARN ("product one image ");
        ROS_WARN("the size of product queue %d ",image_num);
      //  sprintf(scan_num, "%s%d%s", ".//result//scan_image//scan", ++scan_i, ".png");
      //  cv::imwrite(scan_num,depth_image);
        m_mux.unlock();
    }

}
void consume_thread(const Radians& angle_tollerance,const string& in_path ,Visualizer* visualizer){

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
        int image_num= image_buff.size();

        image_buff.pop_back();
        ROS_ERROR("consume one image ");
        ROS_ERROR("the size of image_buff queue %d ",image_num);

        auto cloud_ptr = Cloud::FromImage(read_depth_img.key, *proj_params_ptr);

        /*
        if (image_buff_before.empty()) {
           std::cout<<" the image_buff_before is empty !!! "<<std::endl;
        }

        else {

            no_ground_img = image_buff_before.back();
            auto cloud_ptr_me = Cloud::FromImage(no_ground_img.key, *proj_params_ptr);

            //  auto cloud_ptr_me = Cloud::FromImage(read_depth_img.key, *proj_params_ptr);
        */
            time_utils::Timer timer;
            std::cout
                    << "Now it run in the show_objects_moosmann.cpp  before the visualizer->OnNewObjectReceived(*cloud_ptr, 0); "
                    << std::endl;
             visualizer->OnNewObjectReceived(*cloud_ptr, 0);

           // visualizer->OnNewObjectReceived(*cloud_ptr_me, 0);
            // sleep(1);
            // visualizer->DrawCloud(*cloud_ptr);
            /*
            for (const auto& point : cloud_ptr->points()) {
                // glVertex3f(point.x(),point.y(),point.z());//这里是画出所有的点云数据
                std::cout<<" the value of the point.x() is  "<< point.x()<<std::endl;
                std::cout<<" the value of the point.y() is  "<< point.y()<<std::endl;
                std::cout<<" the value of the point.z() is  "<< point.z()<<std::endl;

            }
            */
            std::cout
                    << "Now it run in the show_objects_moosmann.cpp  before the  depth_ground_remover.OnNewObjectReceived(*cloud_ptr, 0); "
                    << std::endl;
            depth_ground_remover.OnNewObjectReceived(*cloud_ptr, 0);
      //  }
    }
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

    fprintf(stderr, "INFO: The data is Reading from: %s \n", in_path.c_str());

    ROS_INFO("test");
    ROS_WARN("TEST");

    QApplication application(argc, argv);
    // visualizer should be created from a gui thread
    Visualizer visualizer;
    visualizer.show();
   // visualizer.show();



   // visualizer_me.show();



    // create and run loader thread
    std::cout<<" Now it runs before the load_thread ！"<<std::endl;
    std::thread loader_thread(ReadData , angle_tollerance, in_path );
    std::cout<<" Now it runs after the load_thread ！"<<std::endl;
    std::thread process_img(consume_thread, angle_tollerance,in_path,&visualizer);

   //std::thread img_cluster_thread(cluster_thread, angle_tollerance,in_path,&visualizer);
   //if we close the qt application we will be here
    auto exit_code = application.exec();

   //join thread after the application is dead
    loader_thread.join();
    process_img.join();
   //img_cluster_thread.join();
    return exit_code;

}