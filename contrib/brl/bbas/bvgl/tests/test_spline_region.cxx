//:
// \file
#include <testlib/testlib_test.h>
#include <bvgl/bvgl_spline_region_3d.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
#define TEST_SPLINE_REGION 1
//: Test changes
static void test_spline_region()
{
#if TEST_SPLINE_REGION
  std::string base_dir = "c:/Users/mundy/VisionSystems/Janus/RelevantPapers/FacialMusclesExpression/skull/";
  vgl_point_3d<double> p0(-27.048786163330,-58.756767272949,61.351360321045);
  vgl_point_3d<double> p1(-22.989215850830,-61.527729034424,69.345726013184);
  vgl_point_3d<double> p2(-16.756101608276,-63.819145202637,72.460670471191);
  vgl_point_3d<double> p3(-10.471982955933,-66.513549804688,75.632919311523);
  vgl_point_3d<double> p4(-4.012704849243,-65.970741271973,77.426780700684);
  vgl_point_3d<double> p5(1.456362962723,-65.779235839844,77.438545227051);
  vgl_point_3d<double> p6(9.006361961365,-64.448524475098,76.230621337891);
  vgl_point_3d<double> p7(14.902037620544,-64.017341613770,73.427680969238);
  vgl_point_3d<double> p8(22.068965911865,-61.970176696777,69.302963256836);
  vgl_point_3d<double> p9(25.956886291504,-59.957836151123,62.066261291504);
  std::vector<vgl_point_3d<double> > knots;
  knots.push_back(p0); knots.push_back(p1); knots.push_back(p2); knots.push_back(p3);
  knots.push_back(p4); knots.push_back(p5); knots.push_back(p6); knots.push_back(p7);
  knots.push_back(p8); knots.push_back(p9);
  bvgl_spline_region_3d<double> spl_reg(knots, 0.5);
  vgl_point_3d<double> p_pos(0.0, -40.0, 65.0);
  spl_reg.set_point_positive(p_pos);
  vgl_point_3d<double> p3d(0.0, -65.8559, 77.0);
  bool inside = spl_reg.in(p3d);
  double sd;
  vgl_point_3d<double> psd(1.08,-60.0 , 68.39);
  bool good = spl_reg.signed_distance(psd, sd);
  if(good)
    std::cout << "signed distance " << sd << '\n';
  vgl_point_3d<double> cent = spl_reg.centroid();
  std::cout << "centroid " << cent << '\n';
  vgl_pointset_3d<double> ptset = spl_reg.random_pointset(1000);
  std::string pt_path = base_dir + "lower_mouth_plane.txt";
  std::ofstream ostr(pt_path.c_str());
  if(ostr){
    ostr << ptset;
    ostr.close();
  }
#endif
}

TESTMAIN( test_spline_region );
