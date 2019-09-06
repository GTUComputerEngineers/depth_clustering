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
#include "CommonLib.h"
#include "comBase.h"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/syslog_sink.h>

#include <string>
#include <vector>

namespace depth_clustering {

using std::array;
using std::string;
using std::to_string;
using std::vector;

using std::lock_guard;
using std::map;
using std::mutex;
using std::string;
using std::thread;
using std::unordered_map;
using std::vector;

static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent),
      AbstractClient<Cloud>(),
      _updated{false},
      m_publisher(true) {
  _cloud_obj_storer.SetUpdateListener(this);
  m_logger = spdlog::stdout_color_mt("Lidar");
  m_logger->set_level(spdlog::level::debug);
  std::string addr;
  addr.resize(50);
  sprintf(&addr.front(), zmqbase::PROC_CONNECTION.c_str(), "lidar_req_rep");
  m_server.connect(addr);
  m_logger->info("lidar_req_rep addr:{}", addr);
  addr.clear();
  addr.resize(50);
  sprintf(&addr.front(), zmqbase::PROC_CONNECTION.c_str(), "lidar_pub");
  m_publisher.connect(addr);
  m_logger->info("lidar_pub addr:{}", addr);
}

void Visualizer::draw() {
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud);
  std::vector<std::array<float, 5>> arr;
  for (const auto& kv : _cloud_obj_storer.object_clouds()) {
    const auto& cluster = kv.second;
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
    auto dist = sqrt(center.x() * center.x() + center.y() * center.y() +
                     center.z() * center.z());

    if (dist < 25) {
      std::array<float, 5> temp;
      temp[0] = center.x();
      temp[1] = center.y();
      temp[2] = center.z();
      temp[3] = dist;
      temp[4] = center.z();

      /*std::cout << "dist: " << dist << "-x: " << center.x()
                << "-y: " << center.y() << "-z: " << center.z() << std::endl;*/
      m_logger->info("dist:{}-x{}-y{}-z{}", dist, center.x(), center.y(),
                     center.z());
      DrawCube(center, extent);
      responseClusterInfo(center, dist);
    }
  }
  std::string msg = Common::lidar::create_clusters(arr);
  m_publisher.publish("lidar/data", msg);
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
    glVertex3f(point.x(), point.y(), point.z());
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
    glColor3f(0.0f, 0.2f, 0.9f);
    glLineWidth(4.0f);
  } else {
    glColor3f(0.3f, 0.3f, 0.3f);
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
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;
}

void Visualizer::onUpdate() { this->update(); }

unordered_map<uint16_t, Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  return _obj_clouds;
}

void Visualizer::responseClusterInfo(Eigen::Vector3f& center, float dist) {
  std::string msg;
  float x1, y1, x2, y2;
  if (m_server.recv(msg, 0)) {
    m_logger->info("Server connected.");
    if (Common::lidar::parse_lidar_req(msg, x1, y1, x2, y2)) {
      m_logger->info("Message parsed.");
      if ((x1 < center.x() && center.x() < x2) &&
          (y1 < center.y() && center.y() < y2)) {  // Control
        std::string rep = Common::lidar::create_lidar_rep(
            center.x(), center.y(), center.z(), dist, center.z());
        if (m_server.send(rep)) {
          m_logger->info("Response sent.");
        } else {
          m_logger->warn("Response did not send.");
        }
      }

    } else {
      m_logger->warn("Message did not parse.");
    }
  } else {
    m_logger->warn("Message did not receive.");
  }
}

void ObjectPtrStorer::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int id) {
  lock_guard<mutex> guard(_cluster_mutex);
  _obj_clouds = clouds;

  if (_update_listener) {
    _update_listener->onUpdate();
  }
}

}  // namespace depth_clustering
