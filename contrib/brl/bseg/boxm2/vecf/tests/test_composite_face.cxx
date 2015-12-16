//:
// \file
// \author J.L. Mundy
// \date 03/12/15


#include <testlib/testlib_test.h>
#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include "../boxm2_vecf_composite_face_params.h"
#include "../boxm2_vecf_composite_face_scene.h"
#include <vgl/vgl_point_3d.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>

#define BUILD_TEST_COMPOSITE_FACE
void test_composite_face()
{
#ifdef BUILD_TEST_COMPOSITE_FACE
  vcl_string scene_dir = "c:/Users/mundy/VisionSystems/Janus/experiments/vector_flow/composite_face/";
  vcl_string scene_paths = scene_dir + "face_scene_paths.txt";
  if(!vul_file::exists(scene_paths))
    {
      vcl_cout<<"scene paths file " << scene_paths << " does not exist"<<vcl_endl;
      return;
    }
  boxm2_vecf_composite_face_scene cfscene(scene_paths);
  //boxm2_cache::instance()->write_to_disk();

  //  vcl_string target_scene_path = scene_dir + "target_face.xml";
    vcl_string target_scene_path = scene_dir + "../cranium/target_cranium_8.xml";

  if(!vul_file::exists(target_scene_path))
    {
      vcl_cout<<"target scene file) does not exist"<<vcl_endl;
      return;
    }
  boxm2_scene_sptr target = new boxm2_scene(target_scene_path);
  if(!target)
    return;
  cfscene.map_to_target(target);

  boxm2_cache::instance()->write_to_disk();


#endif
}
TESTMAIN( test_composite_face );

