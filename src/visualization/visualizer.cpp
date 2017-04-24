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

#include "./visualizer.h"
#include "clusterers/image_based_clusterer.h"
#include <chrono>
#include <string>
#include <vector>
#include <ctime>
#include <limits>
#include <algorithm>

namespace depth_clustering {

using std::string;
using std::vector;
using std::array;
using std::to_string;
using std::map;
using std::mutex;
using std::string;
using std::vector;
using std::thread;
using std::lock_guard;

static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
  _cloud_obj_storer.SetUpdateListener(this);

}

void Visualizer::draw() {
  std::cout<<"Now it runs in the Visualizer::draw() function in the visualize.cpp"<<std::endl;
  lock_guard<mutex> guard(_cloud_mutex);//c++新特点，会自动解锁
  DrawCloud(_cloud);//在这里画出所有的点云，从读取的图像哪里得到的深度图

  for (const auto& cluster : _cloud_obj_storer.object_clouds()) {
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());
    for (const auto& point : cluster.points()) {
      center = center + point.AsEigenVector();
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    }
    center /= cluster.size();
    if (min_point.x() < max_point.x()) {
      extent = max_point - min_point;
    }
    DrawCube(center, extent);//在这里是把障碍物的用方框画出来
  }

  for (const auto& cluster_me :_cloud_obj_storer.object_clouds_me()) {
        Eigen::Vector3f center_me = Eigen::Vector3f::Zero();
        Eigen::Vector3f extent_me = Eigen::Vector3f::Zero();
        Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest());
        Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max());
        for (const auto& point : cluster_me.points()) {
            center_me = center_me + point.AsEigenVector();
            min_point << std::min(min_point.x(), point.x()),
                    std::min(min_point.y(), point.y()),
                    std::min(min_point.z(), point.z());
            max_point << std::max(max_point.x(), point.x()),
                    std::max(max_point.y(), point.y()),
                    std::max(max_point.z(), point.z());
        }
      center_me /= cluster_me.size();
        if (min_point.x() < max_point.x()) {
            extent_me = max_point - min_point;
        }
        DrawCube_me(center_me, extent_me);//在这里是把障碍物的用方框画出来
  }

    std::cout<<"Now it runs in the end of Visualizer::draw() function in the visualize.cpp"<<std::endl;
}


void Visualizer::init() {
    setSceneRadius(100.0);
    camera()->showEntireScene();
    glDisable(GL_LIGHTING);
}

void Visualizer::DrawCloud(const Cloud& cloud) {
    glPushMatrix();
    glBegin(GL_POINTS);
    glColor3f(1.0f, 1.0f, 1.0f);
    for (const auto& point : cloud.points()) {
        glVertex3f(point.x(),point.y(),point.z());//这里是画出所有的点云数据
    }
    glEnd();
    glPopMatrix();
}
void Visualizer::DrawCube(const Eigen::Vector3f& center,
                      const Eigen::Vector3f& scale) {
  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(scale.x(), scale.y(), scale.z());
  float volume = scale.x() * scale.y() * scale.z();
  if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
   // glColor3f(0.0f, 0.2f, 0.9f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(4.0f);
  } else {
    glColor3f(0.3f, 0.3f, 0.3f);
   // glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(1.0f);
  }
  glBegin(GL_LINE_STRIP);

  // Bottom of Box
  glVertex3f(-0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  // Top of Box
  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, -0.5);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the Box

  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(-0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(0.5, 0.5, -0.5);

  glEnd();
  glPopMatrix();
}

void Visualizer::DrawCube_me(const Eigen::Vector3f& center,
                          const Eigen::Vector3f& scale) {
    glPushMatrix();
    glTranslatef(center.x(), center.y(), center.z());
    glScalef(scale.x(), scale.y(), scale.z());
    float volume = scale.x() * scale.y() * scale.z();
    if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
        glColor3f(0.0f, 0.2f, 0.9f);
       // glColor3f(1.0f, 0.0f, 0.0f);
        glLineWidth(4.0f);
    } else {
         glColor3f(0.3f, 0.3f, 0.3f);
        //glColor3f(0.0f, 1.0f, 0.0f);
        glLineWidth(1.0f);
    }
    glBegin(GL_LINE_STRIP);

    // Bottom of Box
    glVertex3f(-0.5, -0.5, -0.5);
    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(-0.5, -0.5, -0.5);

    // Top of Box
    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(-0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5, -0.5);
    glVertex3f(-0.5, 0.5, -0.5);

    glEnd();

    glBegin(GL_LINES);
    // For the Sides of the Box

    glVertex3f(-0.5, 0.5, -0.5);
    glVertex3f(-0.5, -0.5, -0.5);

    glVertex3f(-0.5, -0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);

    glVertex3f(0.5, -0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);

    glVertex3f(0.5, -0.5, -0.5);
    glVertex3f(0.5, 0.5, -0.5);

    glEnd();
    glPopMatrix();
}
    Visualizer::~Visualizer() {}

void Visualizer::OnNewObjectReceived(const Cloud& cloud, const int id) {
  std::cout<<"Now it runs in the Visualizer::OnNewObjectReceived function in the visualize.cpp"<<std::endl;
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;

}

void Visualizer::onUpdate() {

    std::cout<<" Now it returns the Visualizer::onUpdate() function in the visualizer.cpp"<<std::endl;
    this->update();
    std::cout<<" Now it run in the end of  the Visualizer::onUpdate() function in the visualizer.cpp"<<std::endl;
}

//在这里是不同的函数，不要弄错了
vector<Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  std::cout<<" Now it returns the _obj_clouds in the onUpdate function in the visualizer.cpp"<<std::endl;
  return _obj_clouds;
}

vector<Cloud> ObjectPtrStorer::object_clouds_me() const {
   // lock_guard<mutex> guard(_cluster_mutex);
    std::cout<<" Now it returns the _obj_clouds in the onUpdate function in the visualizer.cpp"<<std::endl;
    return _obj_clouds_me;
}


void ObjectPtrStorer::OnNewObjectReceived(const std::vector<Cloud>& clouds,
                                          const int id) {
  //看起来好像是在这里收到了那个聚类的输出信息
  std::cout<<"Now it runs in the ObjectPtrStorer::OnNewObjectReceived function in the visualize.cpp"<<std::endl;
  std::cout<<"the value of i_senter is ::"<<i_senter<<std::endl;
if(i_senter==0) {
    lock_guard <mutex> guard(_cluster_mutex);
    _obj_clouds.clear();
    _obj_clouds.reserve(clouds.size());
    for (const auto &obj : clouds) {
        _obj_clouds.push_back(obj);
    }
}
    if(i_senter==1) {
        i_senter=0;
        lock_guard <mutex> guard(_cluster_mutex);
        _obj_clouds_me.clear();
        _obj_clouds_me.reserve(clouds.size());
        for (const auto &obj_me : clouds) {
            _obj_clouds_me.push_back(obj_me);
        }
    }
 if (_update_listener) {
    _update_listener->onUpdate();//然后在这里就开始画图了
  }
}

}  // namespace depth_clustering
