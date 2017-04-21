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

#ifndef SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
#define SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_

#include <chrono>
#include <ctime>
#include <map>
#include <opencv/cv.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/useful_typedefs.h"
#include "ground_removal/show_objects_moosmann.h"
#include "clusterers/abstract_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/cloud_projection.h"
char img_lable[6000];
int  i_lable=0;
test_define get_image_buff_before;
test_define get_image_buff_after;
namespace depth_clustering {

/**
 * @brief      Class for image based clusterer.
 *
 * @tparam     LabelerT  A Labeler class to be used for labeling.
 */
template <typename LabelerT>

class ImageBasedClusterer : public AbstractClusterer {
 public:
  using Receiver = AbstractClient<Cloud>;
  using Sender = AbstractSender<std::vector<Cloud>>;

  /**
   * @brief      Construct an image-based clusterer.
   *
   * @param[in]  angle_tollerance  The angle tollerance to separate objects
   * @param[in]  min_cluster_size  The minimum cluster size to send
   * @param[in]  max_cluster_size  The maximum cluster size to send
   */
  explicit ImageBasedClusterer(Radians angle_tollerance = 8_deg,
                               uint16_t min_cluster_size = 100,
                               uint16_t max_cluster_size = 25000)
      : AbstractClusterer(0.0, min_cluster_size, max_cluster_size),
        _counter(0),
        _angle_tollerance(angle_tollerance),
        _label_client{nullptr} {}

  virtual ~ImageBasedClusterer() {}

  /**
   * @brief      Sets the difference type.
   *
   * @param[in]  diff_type  The difference type
   */
  void SetDiffType(DiffFactory::DiffType diff_type) { _diff_type = diff_type; }

  /**
   * @brief      Sets the label image client.
   *
   * @param      client  The client to receive color images with labels
   */
  void SetLabelImageClient(AbstractClient<cv::Mat>* client) {
    this->_label_client = client;
  }

  /**
   * @brief      Gets called when clusterer receives a cloud to cluster
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */

  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override {

    std::cout<<"Now it runs int the image_lable_cluster.h on new object received"<<std::endl;
    // generate a projection from a point cloud
    if (!cloud.projection_ptr()) {
      fprintf(stderr, "ERROR: projection not initialized in cloud.\n");
      fprintf(stderr, "INFO: cannot label this cloud.\n");
      return;
    }

    //在这里的输入果然是已经去除掉地面的图像了
   // sprintf(img_lable, "%s%d%s", ".//result//image_lable//image_lable", ++i_lable, ".png");
   // cv::imwrite(img_lable,cloud.projection_ptr()->depth_image());

   // cluster_me( cloud);
    std::vector<Cloud> return_origin_cluster;
    std::vector<Cloud> return_me_cluster;
    if ((image_buff_before.empty())&(image_buff_after.empty())) {
      std::cout<<"one_of_the  image_buff_before or image_buff_after is empty!!! "<<std::endl;
     // continue;
    }
    else {
      get_image_buff_before = image_buff_before.back();
      image_buff_before.pop_back();
      std::cout << " the value of the get_image_buff_before.id is :" << get_image_buff_before.id << std::endl;

      get_image_buff_after = image_buff_after.back();
      image_buff_after.pop_back();
      std::cout << " the value of the get_image_buff_after.id is :" << get_image_buff_after.id << std::endl;

      if (get_image_buff_before.id == get_image_buff_after.id) {
        return_origin_cluster = cluster_me(cloud,get_image_buff_before.key);
        return_me_cluster = cluster_me(cloud,get_image_buff_after.key);
        //在这里是第二个模块聚类的输出结果　
        std::cout<<"int the image_based_clustered.h the value of  this->id() is :"<< this->id()<<std::endl;
        std::cout << "Now it runs in last of the image_based_clustered.cpp !!!" << std::endl;
        this->ShareDataWithAllClients(return_origin_cluster);
      }
    }
  }



std::vector <Cloud> cluster_me(const Cloud& cloud,const cv::Mat& depth_image){

  time_utils::Timer timer;
  std::cout<<"now it is runs before the  LabelerT image_labeler(cloud.projection_  in the image_based_clustered.h !!! "<<std::endl;
  LabelerT image_labeler(depth_image,
                         cloud.projection_ptr()->params(), _angle_tollerance);

  std::cout<<"now it is runs after the  LabelerT image_labeler(cloud.projection_  in the image_based_clustered.h !!! "<<std::endl;
  std::cout<<"now it is runs before the  ComputeLabels function in the image_based_clustered.h !!! "<<std::endl;
  image_labeler.ComputeLabels(_diff_type);
  std::cout<<"now it is runs after the  ComputeLabels function in the image_based_clustered.h !!! "<<std::endl;
  std::cout<<"now it is runs before the  image_labeler.GetLabelImage() function in the image_based_clustered.h !!! " <<std::endl;
  const cv::Mat* labels_ptr = image_labeler.GetLabelImage();
  std::cout<<"now it is runs after the  image_labeler.GetLabelImage() function in the image_based_clustered.h !!! " <<std::endl;
  fprintf(stderr, "INFO: image based labeling took: %lu us\n",
          timer.measure());
  std::cout<<"the value of the  _label_client is :!!! "<<_label_client <<std::endl;
  // send image to whoever wants to get it
  /*
  if (_label_client) {
    _label_client->OnNewObjectReceived(*labels_ptr, this->id());
  }
　　*/
  // create 3d clusters from image labels
  std::unordered_map<uint16_t, Cloud> clusters;
  for (int row = 0; row < labels_ptr->rows; ++row) {
    for (int col = 0; col < labels_ptr->cols; ++col) {
      const auto& point_container = cloud.projection_ptr()->at(row, col);
      if (point_container.IsEmpty()) {
        // this is ok, just continue, nothing interesting here, no points.
        continue;
      }
      uint16_t label = labels_ptr->at<uint16_t>(row, col);
      if (label < 1) {
        // this is a default label, skip
        continue;
      }
      for (const auto& point_idx : point_container.points()) {
        const auto& point = cloud.points()[point_idx];
        clusters[label].push_back(point);
      }
    }
  }


  //now send clusters to clients. They can't wait to get a new cloud.
  //现在开始发送给新的客户端， they  cann't wait to get the point .
  //auto labels_color = AbstractImageLabeler::LabelsToColor(*labels_ptr);


  _counter++;
  std::vector<Cloud> clusters_to_send;
  for (auto& kv : clusters) {
    // if (counter > num_clusters_max) { break; }
    auto& cluster = kv.second;
    if (cluster.size() < this->_min_cluster_size ||
        cluster.size() > this->_max_cluster_size) {
      // cluster is too small or too big, forget it.
      continue;
    }
    cluster.InitProjection(cloud.projection_ptr()->params());
    clusters_to_send.push_back(cluster);
  }
 return clusters_to_send;
}
private:
  int _counter;
  Radians _angle_tollerance;
  AbstractClient<cv::Mat>* _label_client;
  DiffFactory::DiffType _diff_type = DiffFactory::DiffType::NONE;
};

}  // namespace depth_clustering



#endif  // SRC_CLUSTERERS_IMAGE_BASED_CLUSTERER_H_
