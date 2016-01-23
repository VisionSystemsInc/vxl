//:
// \file
// \author J.L. Mundy
// \date 03/12/15


#include <testlib/testlib_test.h>
#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include "../boxm2_vecf_fat_pocket.h"
#include "../boxm2_vecf_fat_pocket_params.h"
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>

#define BUILD_TEST_FAT_POCKET
void test_fat_pocket()
{
#ifdef BUILD_TEST_FAT_POCKET
  vcl_string scene_base_path = "D:/VisionSystems/Janus/experiments/vector_flow/fat_pocket/";
  vcl_string geo_base_path = "D:/VisionSystems/Janus/RelevantPapers/FacialMusclesExpression/fat_pocket/";
  vcl_string param_path = geo_base_path + "fat_pocket_params.txt";
  vcl_string geo_path = scene_base_path+ "fat_pocket_geo.txt";
  vcl_string rand_pts_path = geo_base_path + "random_pocket_points.txt";
  boxm2_vecf_fat_pocket fpoc(geo_path);
  vgl_pointset_3d<double> ptset = fpoc.random_pointset(100000);
  vcl_ofstream ostr(rand_pts_path.c_str());
  if(ostr){
    ostr << ptset;
    ostr.close();
  }else
   vcl_cout << "couldn't open " << rand_pts_path << '\n';

#endif
}
TESTMAIN( test_fat_pocket);

