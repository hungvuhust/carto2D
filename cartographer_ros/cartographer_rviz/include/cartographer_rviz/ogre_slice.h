

#ifndef CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_
#define CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Ogre.h"
#include "OgreManualObject.h"
#include "OgreMaterial.h"
#include "OgreQuaternion.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "OgreTexture.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"

namespace cartographer_rviz {

Ogre::Vector3    ToOgre(const Eigen::Vector3d &v);
Ogre::Quaternion ToOgre(const Eigen::Quaterniond &q);

// A class containing the Ogre code to visualize a slice texture of a submap.
// Member functions are expected to be called from the Ogre thread.
class OgreSlice {
public:
  // Attaches a node visualizing the submap 'id' to the 'submap_node' which is
  // expected to represent the submap frame.
  OgreSlice(
      const ::cartographer::mapping::SubmapId &id, int slice_id,
      Ogre::SceneManager *const scene_manager,
      Ogre::SceneNode *const    submap_node);
  ~OgreSlice();

  OgreSlice(const OgreSlice &)           = delete;
  OgreSlice &operator=(const OgreSlice &)= delete;

  // Updates the texture and pose of the submap using new data from
  // 'submap_texture'.
  void Update(const ::cartographer::io::SubmapTexture &submap_texture);

  // Changes the opacity of the submap to 'alpha'.
  void SetAlpha(float alpha);

  // Sets the local visibility of this slice.
  void SetVisibility(bool visibility);

  // Updates the SceneNode to be visible if the submap and this slice are
  // visible.
  void UpdateOgreNodeVisibility(bool submap_visibility);

private:
  // TODO(gaschler): Pack both ids into a struct.
  const ::cartographer::mapping::SubmapId id_;
  const int                               slice_id_;
  Ogre::SceneManager *const               scene_manager_;
  Ogre::SceneNode *const                  submap_node_;
  Ogre::SceneNode *const                  slice_node_;
  Ogre::ManualObject *const               manual_object_;
  Ogre::TexturePtr                        texture_;
  Ogre::MaterialPtr                       material_;
  bool                                    visibility_= true;
};

} // namespace cartographer_rviz

#endif // CARTOGRAPHER_RVIZ_SRC_OGRE_SLICE_H_
