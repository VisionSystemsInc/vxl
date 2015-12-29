//:
// \file
// \author J.L. Mundy
// \date 03/12/15


#include <testlib/testlib_test.h>
#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include "../boxm2_vecf_mouth.h"
#include "../boxm2_vecf_mandible_params.h"
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>

#define BUILD_TEST_MOUTH
void test_mouth()
{
#ifdef BUILD_TEST_MOUTH
  vcl_string pc_dir = "c:/Users/mundy/VisionSystems/Janus/RelevantPapers/FacialMusclesExpression/skull/";
  vcl_string scene_dir = "c:/Users/mundy/VisionSystems/Janus/experiments/vector_flow/mouth/";
  vcl_string mouth_geo_path = scene_dir + "mouth_geo.txt";
  //vcl_string mouth_pt_path = pc_dir + "skin-trans-sampled-mandible-coupling-tooth-contour.txt";
  vcl_string mouth_pt_path = pc_dir + "skin-trans-sampled-mandible-coupling-lip-contour.txt";
  vcl_string mouth_display_pc_path = pc_dir + "mouth_open_v2.txt";
  if(!vul_file::exists(mouth_pt_path))
    {
      vcl_cout<<"mouth knot file does not exist"<<vcl_endl;
      return;
    }
  vgl_pointset_3d<double> ptset;
  vcl_ifstream istr(mouth_pt_path.c_str());
  if(!istr)
    return;
  istr >> ptset;
  istr.close();
  boxm2_vecf_mouth mouth(ptset);
  boxm2_vecf_mandible_params params;
  params.jaw_opening_angle_rad_ = 0.35;
  mouth.set_mandible_params(params);
  vgl_point_3d<double> pt_sup(-3.456, -60.0, 90.459);
  vgl_point_3d<double> pt_inf(-3.518, -69.144, 88.131);
  vgl_vector_3d<double> v = pt_sup-pt_inf;
  vgl_point_3d<double> bad_pt = pt_inf + 0.3*v;
  bool badin = mouth.in(bad_pt);
  unsigned n_pts = 1000;
  vgl_pointset_3d<double> mouth_pts = mouth.random_pointset(n_pts);
  vcl_ofstream ostr(mouth_display_pc_path.c_str());
  if(!ostr)
    return;
  ostr << mouth_pts;
  ostr.close();
  params.jaw_opening_angle_rad_ = 0.0;
  mouth.set_mandible_params(params);
#endif
}
TESTMAIN( test_mouth);

