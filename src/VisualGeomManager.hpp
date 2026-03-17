#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>  // C++17 for shared locking
#include "Eigen/Dense"
#include "Utils/VisualGeom.h"
#include "engine/engine_vis_visualize.h"

class VisualGeomManager {
 public:
  static VisualGeomManager* GetInstance() {
    static VisualGeomManager instance;
    return &instance;
  }

  void addVisualGeom(const std::string& id, const std::shared_ptr<VisualGeom<float>>& geom) {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    _visual_geoms[id] = geom;
  }

  void removeVisualGeom(const std::string& id) {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    _visual_geoms.erase(id);
  }

  void clearVisualGeoms() {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    // _visual_geoms.clear();
    std::erase_if(_visual_geoms, [](const auto& pair) { // require C++20
      return !pair.second->_is_static;
    });
  }

  // Clean up if geom size is 0
  void cleanInvalidGeoms() {
    std::lock_guard<std::shared_mutex> lock(_mutex);

    for (auto it = _visual_geoms.begin(); it != _visual_geoms.end(); ) {
      if ((it->second->_size.array() <= 0).any()) {
        it = _visual_geoms.erase(it);
      } else {
        ++it;
      }
    }
  }

  void updateGeoms(mjvScene& mj_scene) {
    cleanInvalidGeoms();
    
    {
      std::shared_lock<std::shared_mutex> readLock(_mutex);
      // auto& geoms = getVisualGeoms();

      for (const auto& [id, geom] : _visual_geoms) {
        if (mj_scene.ngeom >= mj_scene.maxgeom) {
          std::cerr << "Error: Exceeded Mujoco mj_scene geometry limit." << std::endl;
          return;
        }

        mjvGeom* new_geom = mj_scene.geoms + mj_scene.ngeom++;

        // Initialize the new geom, otherwise it will appear white
        mjv_initGeom(new_geom, mjGEOM_SPHERE, nullptr, nullptr, nullptr, nullptr);
        new_geom->type = geom->_type;
        std::copy(geom->_size.data(), geom->_size.data() + 3, new_geom->size);
        std::copy(geom->_pos.data(), geom->_pos.data() + 3, new_geom->pos);
        std::copy(geom->_mat.data(), geom->_mat.data() + 9, new_geom->mat);
        std::copy(geom->_rgba.data(), geom->_rgba.data() + 4, new_geom->rgba);
      }
    }

    // we should not need this, but now we need to cause otherwise will ocassionally show deleted geometries
    clearVisualGeoms();
  }

  std::map<std::string, std::shared_ptr<VisualGeom<float>>>& getVisualGeoms() {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    return _visual_geoms;
  }

  mutable std::shared_mutex _mutex;  // `shared_mutex` allows multiple readers, single writer

 private:
  VisualGeomManager() = default;
  ~VisualGeomManager() = default;

  std::map<std::string, std::shared_ptr<VisualGeom<float>>> _visual_geoms;
};
