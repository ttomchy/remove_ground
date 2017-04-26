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
#include<thread>
#include <deque>
#include<condition_variable>
#include<mutex>
#include <unistd.h>
#include"show_objects_moosmann.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

test_define image_me,image_origin;

namespace depth_clustering {
using cv::Mat; 
using cv::DataType;
using std::to_string;
using time_utils::Timer;
const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);// opencv parameter 
const int SAME_OUTPUT_TYPE = -1;
int i = 0;
int i_txt=0;
int num_curlable=0;

int i_smooth=0;
int i_binary_self_before=0;
int i_binary_self_after=0;
int i_save_binary_img_ori=0;
int i_save_binary_img_me=0;
char img_name_before[6000];
char img_name_after[6000];
char save_binary_img_ori[6000];
char save_binary_img_me[6000];
char img_name_no_ground_image[6000];
char img_name_ground_image[6000];
int id_origin=0;
int id_me=0;
int i_no_ground_image=0;
int i_ground_image=0;
void DepthGroundRemover::OnNewObjectReceived(const Cloud& cloud,
                                             const int sender_id) {
  // this can be done even faster if we switch to column-major implementation
  // thus allowing us to load whole row in L1 cache 
  std::cout<<" Now it runs in the OnNewObjectReceived function  in the ground_remove.cpp!"<<std::endl;
  if (!cloud.projection_ptr()) {
    fprintf(stderr, "No projection in cloud. Skipping ground removal.\n");
    return; //
  }
  Cloud cloud_copy(cloud);

  const cv::Mat& depth_image =RepairDepth(cloud.projection_ptr()->depth_image());
  Timer total_timer;
  // _ground_remove_angle=0.15078_deg;
  auto angle_image = CreateAngleImage(depth_image);//刚开始会在这里进行执行程
  auto smoothed_image = ApplySavitskyGolaySmoothing(angle_image, _window_size);
  auto smoothed_image_me=bilateralFilter_me(angle_image);

 // auto no_ground_image =ZeroOutGround(depth_image, smoothed_image,
                                       // _ground_remove_angle);

  auto no_ground_image = ZeroOutGroundBFS(depth_image, smoothed_image,
                                          _ground_remove_angle, _window_size);
  auto NO_ground_image = NO_Ground_image(cloud.projection_ptr()->depth_image(), smoothed_image,
                     _ground_remove_angle);
 //  sprintf(img_name_no_ground_image, "%s%d%s", ".//result//no_ground_image//no_ground_image", ++i_no_ground_image, ".png");
 // cv::imwrite(img_name_no_ground_image,NO_ground_image);


  auto ground_image =Ground_image(cloud.projection_ptr()->depth_image(), smoothed_image,
                                        _ground_remove_angle);
  // sprintf(img_name_ground_image, "%s%d%s", ".//result//ground_image//ground_image", ++i_ground_image, ".png");
  // cv::imwrite(img_name_ground_image,ground_image);


  std::thread thread_three(&consume_print_one ,depth_image,smoothed_image,_ground_remove_angle);
  std::cout << " Now it run in before the thread_four function !"<<std::endl;
 // std::thread thread_four(&consume_print_two,depth_image,smoothed_image,_ground_remove_angle);
  std::thread thread_four(&consume_print_two,depth_image,smoothed_image_me,_ground_remove_angle);
  thread_three.join();
  thread_four.join();


  std::cout<<"  int the depth_ground_remove.cpp the value of  this->id() is :"<< this->id()<<std::endl;
  cloud_copy.projection_ptr()->depth_image() = no_ground_image;
 this->ShareDataWithAllClients(cloud_copy);
  _counter++;
  std::cout<<"the value of the _counter is :"<<_counter<<std::endl;

}

Mat DepthGroundRemover::ZeroOutGround(const cv::Mat& image,
                                      const cv::Mat& angle_image,
                                      const Radians& threshold) const {
  // TODO(igor): test if its enough to remove only values starting from the
  // botom pixel. I don't like removing all values based on a threshold.
  // But that's a start, so let's stick with it for now.
  std::cout<<" Now it runs in the ZeroOutGround function !"<<std::endl;
  Mat binary_self = Mat::zeros(image.rows,image.cols, CV_8UC1);
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (angle_image.at<float>(r, c) > threshold.val()) {
         // The value of the threshold.val() is 0.15708
          res.at<float>(r, c) = image.at<float>(r, c);
          binary_self.at<uchar>(r,c)=255;
      }
    }
  }
    return res;
}

Mat DepthGroundRemover::Ground_image(const cv::Mat& image,
                                      const cv::Mat& angle_image,
                                      const Radians& threshold) const {
    // TODO(igor): test if its enough to remove only values starting from the
    // botom pixel. I don't like removing all values based on a threshold.
    // But that's a start, so let's stick with it for now.
    std::cout<<" Now it runs in the ZeroOutGround function !"<<std::endl;
    Mat ground_res = cv::Mat::zeros(image.size(), CV_32F);
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (angle_image.at<float>(r, c) < threshold.val()) {
                // The value of the threshold.val() is 0.15708
                ground_res.at<float>(r, c) = image.at<float>(r, c);
            }
        }
    }
    return ground_res;
}


Mat DepthGroundRemover::NO_Ground_image(const cv::Mat& image,
                                     const cv::Mat& angle_image,
                                     const Radians& threshold) const {
    // TODO(igor): test if its enough to remove only values starting from the
    // botom pixel. I don't like removing all values based on a threshold.
    // But that's a start, so let's stick with it for now.
    std::cout<<" Now it runs in the ZeroOutGround function !"<<std::endl;


    Mat ground_res = cv::Mat::zeros(image.size(), CV_32F);
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (angle_image.at<float>(r, c) > threshold.val()) {
                // The value of the threshold.val() is 0.15708
                ground_res.at<float>(r, c) = image.at<float>(r, c);
            }
        }
    }
    return ground_res;
}


Mat DepthGroundRemover::ZeroOutGroundBFS(const cv::Mat& image,
                                         const cv::Mat& angle_image,
                                         const Radians& threshold,
                                         int kernel_size) const {
  std::cout<<" Now it runs in the ZeroOutGroundBFS function in the ground remove.cpp!"<<std::endl;
  std::cout<<" the value of the threshhold is : "<<threshold.val()<<std::endl;
  std::cout<<"the value of the kernel is  :"<<kernel_size<<std::endl;
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  std::cout<<"now it is before  the image_labeler(image, _params, threshold) in the ground_remove.cpp "<<kernel_size<<std::endl;
  LinearImageLabeler<> image_labeler(image, _params, threshold);
  std::cout<<"Now it runs after the image_labeler(image, _params, threshold) in the ground_remove.cpp"<<std::endl;
  SimpleDiff simple_diff_helper(&angle_image);
  std::cout<<"Now it run after  the simple_diff_helper(&angle_image)"<<std::endl;
  Radians start_thresh = 30_deg;
  for (int c = 0; c < image.cols; ++c) {
    // start at bottom pixels and do bfs
    int r = image.rows - 1;
    while (r > 0 && image.at<float>(r, c) < 0.001f){
      --r;
    }
    auto current_coord = PixelCoord(r, c);
    uint16_t current_label = image_labeler.LabelAt(current_coord);
    if (current_label > 0) {
      // this coord was already labeled, skip//如果这里的标签是1 的话说明早就已经是被标记过了
       continue;
    }
    std::cout<<"Now it runs before the mponent image_labeler.LabelOneComponent  in the ground remove.cpp"<<std::endl;
    std::cout<<"Now it turns into in the LabelOneComponent function  in the liner_image_lable .h"<<std::endl;
    image_labeler.LabelOneComponent(1, current_coord, &simple_diff_helper);//把所有1的区域连接起来,大部分的计算量在这里
    std::cout<<"Now it runs after the mponent image_labeler.LabelOneComponent  in the ground remove.cpp"<<std::endl;
  }
    std::cout<<"The value of num_curlable is :"<<num_curlable<<std::endl;
    num_curlable=0;

  auto label_image_ptr = image_labeler.GetLabelImage();//得到标签
  std::cout<<"Now it runs after the  image_labeler.GetLabelImage();  in the ground remove.cpp"<<std::endl;
  if (label_image_ptr->rows != res.rows || label_image_ptr->cols != res.cols) {
    fprintf(stderr, "ERROR: label image and res do not correspond.\n");
    return res;
  }
  kernel_size = std::max(kernel_size - 2, 3);
  Mat kernel = GetUniformKernel(kernel_size, CV_8U);
  Mat dilated = Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
  cv::dilate(*label_image_ptr, dilated, kernel);
  //在这里输出核函数的值
  Mat binary_self = Mat::zeros(image.rows,image.cols, CV_8UC1);

  for (int r = 0; r < dilated.rows; ++r) {
    for (int c = 0; c < dilated.cols; ++c) {
      if (dilated.at<uint16_t>(r, c) == 0) {
        // all unlabeled points are non-ground
          res.at<float>(r, c) = image.at<float>(r, c);
          binary_self.at<uchar>(r,c)=255;
      }
    }
  }
  return res;
}

  Mat DepthGroundRemover::RepairDepth(const Mat& no_ground_image, int step,//step is 5 , depth_threshold is 1.0f
                                    float depth_threshold) {
  std::cout<<" Now it runs in the RepairDepth function  parameter is const Mat& no_ground_image, int step, float depth_threshold!"<<std::endl;
  Mat inpainted_depth = no_ground_image.clone();
  int i_count=0;
  for (int c = 0; c < inpainted_depth.cols; ++c) {
    for (int r = 0; r < inpainted_depth.rows; ++r) {
      float& curr_depth = inpainted_depth.at<float>(r, c);
      //std::cout<<"the value of  curr_depth  is :"<<curr_depth <<std::endl;
      if (curr_depth < 0.001f) {
       //  std::cout<<"the value of r is"<<r<<std::endl;

        int counter = 0;
        float sum = 0.0f;
        for (int i = 1; i < step; ++i) {
          if (r - i < 0) {//前五列和最后5列都不进行处理？
             // std::cout<<"the value of r is"<<r<<std::endl;
             // std::cout<<"the value of i is"<<i<<std::endl;
            continue;
          }
          for (int j = 1; j < step; ++j) {
            if (r + j > inpainted_depth.rows - 1) {
              continue;
            }


            const float& prev = inpainted_depth.at<float>(r - i, c);
            const float& next = inpainted_depth.at<float>(r + j, c);
            if (prev > 0.001f && next > 0.001f &&
                fabs(prev - next) < depth_threshold) {

              sum += prev + next;
              counter += 2;

                i_count++;
            }
          }
        }
        if (counter > 0) {
          curr_depth = sum / counter;
        }
      }
    }
  }
  return inpainted_depth;
}

Mat DepthGroundRemover::RepairDepth(const Mat& depth_image) {
  std::cout<<" Now it runs in the RepairDepth function parameter is Mat& depth_image  !"<<std::endl;
  Mat kernel = GetUniformKernel(5);
  Mat inpainted_depth;  // init an empty smoothed image

  cv::filter2D(depth_image, inpainted_depth, SAME_OUTPUT_TYPE, kernel,
               ANCHOR_CENTER, 0, cv::BORDER_REFLECT101);
  Mat mask = depth_image > 0;
  depth_image.copyTo(inpainted_depth, mask);
  return inpainted_depth;
}


Mat DepthGroundRemover::CreateAngleImage(const Mat& depth_image) {
  std::cout<<" Now it runs in the CreateAngleImage function !"<<std::endl;
  Mat angle_image = Mat::zeros(depth_image.size(), DataType<float>::type);
  Mat x_mat = Mat::zeros(depth_image.size(), DataType<float>::type);
  Mat y_mat = Mat::zeros(depth_image.size(), DataType<float>::type);
  const auto& sines_vec = _params.RowAngleSines();//sin and cos functions
  const auto& cosines_vec = _params.RowAngleCosines();//it is defined in the parameter.cpp
  float dx, dy;


  x_mat.row(0) = depth_image.row(0) * cosines_vec[0];
  y_mat.row(0) = depth_image.row(0) * sines_vec[0];
  for (int r = 1; r < angle_image.rows; ++r) {
    x_mat.row(r) = depth_image.row(r) * cosines_vec[r];//转换为极坐标系的话x=r*cos(t);y =r*sin(t);
    y_mat.row(r) = depth_image.row(r) * sines_vec[r];
    for (int c = 0; c < angle_image.cols; ++c) {
      dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));// 不对啊我怎么感觉他在这里并没有把这种弧度性的角度进行转化啊
      dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));//把角度以弧度写入每个像素值
      angle_image.at<float>(r, c) = atan2(dy, dx);
     // std::cout<< "the value of the angle_image.at<float>(r, c) is "<<angle_image.at<float>(r, c) <<std::endl;
    }
  }
  return angle_image;
}

Mat DepthGroundRemover::GetSavitskyGolayKernel(int window_size) const {
  std::cout<<" Now it runs in the GetSavitskyGolayKernel function !"<<std::endl;
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
  if (!window_size_ok) {
    throw std::logic_error("bad window size");
  }
  // below are no magic constants. See Savitsky-golay filter.
  Mat kernel;
  switch (window_size) {
    case 5:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -3.0f;
      kernel.at<float>(0, 1) = 12.0f;
      kernel.at<float>(0, 2) = 17.0f;
      kernel.at<float>(0, 3) = 12.0f;
      kernel.at<float>(0, 4) = -3.0f;
      kernel /= 35.0f;
      return kernel;
    case 7:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -2.0f;
      kernel.at<float>(0, 1) = 3.0f;
      kernel.at<float>(0, 2) = 6.0f;
      kernel.at<float>(0, 3) = 7.0f;
      kernel.at<float>(0, 4) = 6.0f;
      kernel.at<float>(0, 5) = 3.0f;
      kernel.at<float>(0, 6) = -2.0f;
      kernel /= 21.0f;
      return kernel;
    case 9:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -21.0f;
      kernel.at<float>(0, 1) = 14.0f;
      kernel.at<float>(0, 2) = 39.0f;
      kernel.at<float>(0, 3) = 54.0f;
      kernel.at<float>(0, 4) = 59.0f;
      kernel.at<float>(0, 5) = 54.0f;
      kernel.at<float>(0, 6) = 39.0f;
      kernel.at<float>(0, 7) = 14.0f;
      kernel.at<float>(0, 8) = -21.0f;
      kernel /= 231.0f;
      return kernel;
    case 11:
      kernel = Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -36.0f;
      kernel.at<float>(0, 1) = 9.0f;
      kernel.at<float>(0, 2) = 44.0f;
      kernel.at<float>(0, 3) = 69.0f;
      kernel.at<float>(0, 4) = 84.0f;
      kernel.at<float>(0, 5) = 89.0f;
      kernel.at<float>(0, 6) = 84.0f;
      kernel.at<float>(0, 7) = 69.0f;
      kernel.at<float>(0, 8) = 44.0f;
      kernel.at<float>(0, 9) = 9.0f;
      kernel.at<float>(0, 10) = -36.0f;
      kernel /= 429.0f;
      return kernel;
  }
  return kernel;
}

Mat DepthGroundRemover::GetUniformKernel(int window_size, int type) const {
   std::cout<<" Now it runs in the GetUniformKernel function !"<<std::endl;
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  Mat kernel = Mat::zeros(window_size, 1, type);
  kernel.at<float>(0, 0) = 1;
  kernel.at<float>(window_size - 1, 0) = 1;
  kernel /= 2;
  return kernel; //在这里是自己构造的一个滤波器的kernel
}

Mat DepthGroundRemover::ApplySavitskyGolaySmoothing(const Mat& image,
                                                    int window_size) {
  std::cout<<" Now it runs in the ApplySavitskyGolaySmoothing function !"<<std::endl;
  Mat kernel = GetSavitskyGolayKernel(window_size);

  Mat smoothed_image;  // init an empty smoothed image
  cv::filter2D(image, smoothed_image, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER,
               0, cv::BORDER_REFLECT101);
  //sprintf(filename, "%s%d%s", "", ++i_txt, ".txt");//保存的图片名
  //WriteData(smoothed_image,filename);
  return smoothed_image;
}

Mat DepthGroundRemover::bilateralFilter_me(const Mat& image
                                                   ) {
    std::cout<<" Now it runs in the bilateralFilter_me function !"<<std::endl;
    Mat smoothed_image;
    bilateralFilter(image,smoothed_image,5,10.0,2.0);//这里滤波没什么效果，不明白
    return smoothed_image;
}
Radians DepthGroundRemover::GetLineAngle(const Mat& depth_image, int col,
                                         int row_curr, int row_neigh) {
  // compute inclination angle of the line given the depth of two pixels and
  // their position in the image. We use config to determine the needed angles
  // All following angles are in degrees
  std::cout<<" Now it runs in the GetLineAngle function !"<<std::endl;
  Radians current_angle;
  Radians neighbor_angle;
  current_angle = _params.AngleFromRow(row_curr);
  neighbor_angle = _params.AngleFromRow(row_neigh);
  // for easiness copy references to depth of current and neighbor positions
  const float& depth_current = depth_image.at<float>(row_curr, col);
  const float& depth_neighbor = depth_image.at<float>(row_neigh, col);
  if (depth_current < _eps || depth_neighbor < _eps) {
    // if either of these depth vales is close to zero this depth is not
    // reliable, so we will just report a 0 instead.
    return 0_deg;
  }
  auto x_current = depth_current * cos(current_angle.val());
  auto y_current = depth_current * sin(current_angle.val());
  auto x_neighbor = depth_neighbor * cos(neighbor_angle.val());
  auto y_neighbor = depth_neighbor * sin(neighbor_angle.val());
  auto dx = fabs(x_current - x_neighbor);
  auto dy = fabs(y_current - y_neighbor);
  auto angle = Radians::FromRadians(std::atan2(dy, dx));
  return angle;
}

/*----------------------------
* 功能 : 将 cv::Mat 数据写入到 .txt 文件
*----------------------------
* 函数 : WriteData
* 访问 : public
* 返回 : -1：打开文件失败；0：写入数据成功；1：矩阵为空
*
* 参数 : fileName    [in]    文件名
* 参数 : matData [in]    矩阵数据
*/

void DepthGroundRemover:: WriteData(cv::Mat& m, const char* filename)
{
    ofstream fout(filename);

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<float>(i,j)<<"\t";
        }
        fout<<endl;
    }

    fout.close();
}

void  consume_print_one( const cv::Mat& image,
                         const cv::Mat& angle_image,
                         const Radians& threshold){

    std::cout<<" Now it run in the thread_three function !"<<std::endl;
    m_mux.lock();
    auto no_ground_image =ZeroOutGround_two(image, angle_image,threshold);
     ++id_origin;
    image_origin.id=id_origin;
    std::cout<<" id_origin is:"<<id_origin<<std::endl;
    image_origin.key=no_ground_image;
    image_buff_before.push_front(image_origin);

    int image_num_ori=image_buff_before.size();
    ROS_ERROR("pusk_back one ZeroOutGround_two origin image ");
    ROS_ERROR("the size of  image_buff_before queue %d ",image_num_ori);

    std::cout<<"The value of the image_buff_before image queue is :"<< image_num_ori<<std::endl;
    //sprintf(img_name_after, "%s%d%s", ".//result//binary_self_consume_print_two_ori", ++i_binary_self_before, ".png");
   // cv::imwrite(img_name_after,image_origin.key);
    m_mux.unlock();
}

void consume_print_two( const cv::Mat& image,
                        const cv::Mat& angle_image,
                        const Radians& threshold){
    std::cout<<" Now it run in the thread_four function !"<<std::endl;
    m_mux.lock();
    auto no_ground_image_me =ZeroOutGround_me_two(image, angle_image,threshold);
    ++id_me;
    image_me.id=id_me;
    std::cout<<" id_me is:"<<id_me<<std::endl;
    image_me.key=no_ground_image_me;
    image_buff_after.push_front(image_me);
    int image_num_me=image_buff_after.size();
    ROS_ERROR("pusk_back ZeroOutGround_me_two me image ");
    ROS_ERROR("the size of image_buff_after image queue %d ",image_num_me);
    std::cout<<"The value of the image_buff_after  image queue is :"<< image_num_me<<std::endl;
   // sprintf(img_name_after, "%s%d%s", ".//result//binary_self_afterconsume_print_two", ++i_binary_self_after, ".png");
  //  cv::imwrite(img_name_after,image_me.key);
    m_mux.unlock();
}
cv::Mat ZeroOutGround_two(const cv::Mat& image,
                                      const cv::Mat& angle_image,
                                      const Radians& threshold)  {
    // TODO(igor): test if its enough to remove only values starting from the
    // botom pixel. I don't like removing all values based on a threshold.
    // But that's a start, so let's stick with it for now.
    std::cout<<" Now it runs in the ZeroOutGround two function !"<<std::endl;

    Mat binary_self_ori = Mat::zeros(image.rows,image.cols, CV_8UC1);
    Mat res = cv::Mat::zeros(image.size(), CV_32F);
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (angle_image.at<float>(r, c) > threshold.val()) {
                // The value of the threshold.val() is 0.15708
                res.at<float>(r, c) = image.at<float>(r, c);
                binary_self_ori.at<uchar>(r,c)=255;
            }

        }
    }
  //  sprintf(save_binary_img_ori, "%s%d%s", ".//result//binary_ground_removal//binary_ground_removal", ++i_save_binary_img_ori, ".png");//保存的图片名
  //  cv::imwrite(save_binary_img_ori,binary_self_ori);
    return res;

}

cv:: Mat ZeroOutGround_me_two(const cv::Mat& image,
                                         const cv::Mat& angle_image,
                                         const Radians& threshold)  {
    // TODO(igor): test if its enough to remove only values starting from the
    // botom pixel. I don't like removing all values based on a threshold.
    // But that's a start, so let's stick with it for now.
    std::cout<<" Now it runs in the ZeroOutGround_me_two function !"<<std::endl;

    Mat binary_self_me = Mat::zeros(image.rows,image.cols, CV_8UC1);
    Mat res = cv::Mat::zeros(image.size(), CV_32F);
    for (int r = 0; r < image.rows; ++r) {
        for (int c = 0; c < image.cols; ++c) {
            if (angle_image.at<float>(r, c) > threshold.val()) {
                // The value of the threshold.val() is 0.15708
                res.at<float>(r, c) = image.at<float>(r, c);
                binary_self_me.at<uchar>(r,c)=255;
            }
        }
    }

    Mat out;
    Mat out_origin = cv::Mat::zeros(image.size(), CV_32F);
    cv::blur(res,out_origin,cv::Size(3,3),cv::Point(-1,-1));
   /*
    cv::blur(binary_self_me,binary_self_me,cv::Size(3,3),cv::Point(-1,-1));
    Mat resutl;
    cv::(binary_self_me,binary_self_me,7);
    sprintf(save_binary_img_me, "%s%d%s", ".//result//proposed_ground_removal//proposed_ground_removal", ++i_save_binary_img_me, ".png");//保存的图片名
    cv::imwrite(save_binary_img_me,binary_self_me);
    */
    return res;
}

}  // namespace depth_clustering
