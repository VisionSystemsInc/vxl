//:
// \file
// \author J.L. Mundy
// \date 03/12/15


#include <testlib/testlib_test.h>
#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include "../boxm2_vecf_skin_params.h"
#include "../boxm2_vecf_skin_scene.h"
#include <vgl/vgl_point_3d.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>

#define BUILD_TEST_SKIN
void test_skin()
{
#ifdef BUILD_TEST_SKIN
  vcl_string pc_dir = "c:/Users/mundy/VisionSystems/Janus/RelevantPapers/FacialMusclesExpression/skull/";
  vcl_string scene_dir = "c:/Users/mundy/VisionSystems/Janus/experiments/vector_flow/skin/";
  vcl_string skin_scene_path = scene_dir + "skin.xml";
  if(!vul_file::exists(skin_scene_path))
    {
      vcl_cout<<"skin scene file does not exist"<<vcl_endl;
      return;
    }
  vcl_string skin_geo_path = scene_dir + "skin_geo.txt";
  if(!vul_file::exists(skin_geo_path))
    {
      vcl_cout<<"skin geo file does not exist"<<vcl_endl;
      return;
    }
  // boxm2_vecf_skin_scene skscene(skin_scene_path, skin_geo_path);
  boxm2_vecf_skin_scene skscene(skin_scene_path);
  boxm2_scene_sptr scene = skscene.scene();
  if(!scene)
    return;
   boxm2_cache::instance()->write_to_disk();


#if 0  
  vcl_string skin_pc_path = pc_dir + "skin_trans.txt";
  vcl_ofstream ostr(skin_pc_path.c_str());
  if(!ostr){
    vcl_cout << "couldn't open output stream for " << skin_pc_path << "\n";
    return;
  }
  skscene.export_point_cloud(ostr);
  ostr.close();

  vcl_string target_scene_path = scene_dir + "target_skin_8.xml";

  if(!vul_file::exists(target_scene_path))
    {
      vcl_cout<<"target scene file) does not exist"<<vcl_endl;
      return;
    }
  boxm2_scene_sptr target = new boxm2_scene(target_scene_path);
  if(!target)
    return;
  crscene.map_to_target(target);

  boxm2_cache::instance()->write_to_disk();
#endif
#endif
}
TESTMAIN( test_skin );
 
