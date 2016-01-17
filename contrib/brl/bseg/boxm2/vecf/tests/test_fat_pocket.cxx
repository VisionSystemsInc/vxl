//:
// \file
// \author J.L. Mundy
// \date 03/12/15


#include <testlib/testlib_test.h>
#include <vcl_fstream.h>
#include <vul/vul_timer.h>
#include "../boxm2_vecf_fat_pocket.h"
#include "../boxm2_vecf_mandible_params.h"
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <boxm2/boxm2_scene.h>
#include <boxm2/io/boxm2_cache.h>

#define BUILD_TEST_FAT_POCKET
void test_fat_pocket()
{
#ifdef BUILD_TEST_FAT_POCKET
  vcl_string base_path = "d:/VisionSystems/Janus/RelevantPapers/FacialMusclesExpression/";
  vcl_string display_random_plane_path = base_path + "fat_pocket/random_fat_pocket_plane_pts.txt";
  vcl_string extracted_ptset_path = base_path + "fat_pocket/extracted_fat_pocket_pts.txt";
  vgl_vector_3d<double> normal(-0.73911, 0.100747, -0.666008);
  vgl_point_3d<double> origin(27.0,-10.0,81.75);
  vcl_vector<vgl_point_2d<double > > knots;
  vgl_point_2d<double> p00(4.806863675505915, 6.582528807225162);
  vgl_point_2d<double> p01(11.01628093103361, 10.276435637911343);
  vgl_point_2d<double> p02(32.74932515259732, 15.152382902522577);
  vgl_point_2d<double> p03(57.214391510270254,17.81199050140143);
  vgl_point_2d<double> p04(64.66569966821164, 14.118083670715249);
  vgl_point_2d<double> p05(68.76391915507942,  6.878033374857261);
  vgl_point_2d<double> p06(70.99931160246183, -5.533535243434039);
  vgl_point_2d<double> p07(72.48957323405011, -16.61523357209593);
  vgl_point_2d<double> p08(72.98638920548085,-22.673154780442236);
  vgl_point_2d<double> p09(71.86869298178964, -33.016147098515525);
  vgl_point_2d<double> p10(71.24762644682524, -39.81277431745044);
  vgl_point_2d<double> p11(68.26710318364867,-45.27981937091249);
  vgl_point_2d<double> p12(62.43030722082923, -53.554080244991184);
  vgl_point_2d<double> p13(56.34501013094244, -58.13458943216029);
  vgl_point_2d<double> p14(50.756529012486396, -61.68073289733209);
  vgl_point_2d<double> p15(44.91973304966695, -64.78360842935743);
  vgl_point_2d<double> p16(37.468424891725554, -66.70451001764768);
  vgl_point_2d<double> p17(29.768801889420764,-66.55668016194333);
  vgl_point_2d<double> p18(24.801201023225186, -66.26124208450119);
  vgl_point_2d<double> p19(19.585285312666212, -65.2268763625039);
  vgl_point_2d<double> p20(14.245119038573563,-60.35092909789266);
  vgl_point_2d<double> p21(11.38884633893068, -54.29300788954636);
  vgl_point_2d<double> p22(7.538997581237495, -40.25604225059692);
  vgl_point_2d<double> p23(3.8133435022667994,-24.594056368732495);
  vgl_point_2d<double> p24(0.8328202390902426, -10.70492058548739);
  vgl_point_2d<double> p25(-0.2848759846009665, -3.760463510848136);
  vgl_point_2d<double> p26(-0.03648662715599116,-0.06649018997197387);
  vgl_point_2d<double> p27(2.8198419572980784,  4.218425775978408);

  knots.push_back(p00);   knots.push_back(p01);   knots.push_back(p02);
  knots.push_back(p03);   knots.push_back(p04);   knots.push_back(p05);
  knots.push_back(p06);   knots.push_back(p07);   knots.push_back(p08);
  knots.push_back(p09);   knots.push_back(p10);   knots.push_back(p11);
  knots.push_back(p12);   knots.push_back(p13);   knots.push_back(p14);
  knots.push_back(p15);   knots.push_back(p16);   knots.push_back(p17);
  knots.push_back(p18);   knots.push_back(p19);   knots.push_back(p20);
  knots.push_back(p21);   knots.push_back(p22);   knots.push_back(p23);
  knots.push_back(p24);   knots.push_back(p25);   knots.push_back(p26);
  knots.push_back(p27);  

  boxm2_vecf_fat_pocket fpoc(knots, normal, origin);

  unsigned npts = 100000;
  vgl_pointset_3d<double> rand_pts = fpoc.random_planar_pointset(npts);
  vcl_ofstream rostr(display_random_plane_path.c_str());
  if(!rostr)
    return;
  rostr << rand_pts;
  rostr.close();

  vcl_string skin_pt_path = base_path + "skull/skin_orig_appearance_sampled-r10-s1.05-t10.txt";
  if(!vul_file::exists(skin_pt_path))
    {
      vcl_cout<<"skin point cloud file does not exist"<<vcl_endl;
      return;
    }
  vgl_pointset_3d<double> skin_ptset;
  vcl_ifstream istr(skin_pt_path.c_str());
  if(!istr)
    return;
  istr >> skin_ptset;
  istr.close();
  
  vgl_pointset_3d<double> fat_pocket_pts;
#if 1   // extract potential fat pocket points
   fat_pocket_pts = fpoc.extract_pocket_pts(skin_ptset, 75.0);
  vcl_ofstream ostr(extracted_ptset_path.c_str());
  if(!ostr)
    return;
  ostr << fat_pocket_pts;
  ostr.close();
#else
  vcl_ifstream istr(extracted_ptset_path.c_str());
  if(!istr)
    return;
  istr >> fat_pocket_pts;
  istr.close();
#endif
#endif
}
TESTMAIN( test_fat_pocket);

