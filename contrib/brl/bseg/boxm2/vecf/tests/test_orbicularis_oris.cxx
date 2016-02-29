// This is contrib/brl/bseg/boxm2/vecf/test/test_orbicularis_oris.cxx
//:
// \file
// \author Yi Dong
// \date January, 2016

#include <testlib/testlib_test.h>
#include <boxm2/vecf/boxm2_vecf_orbicularis_oris_params.h>
#include <boxm2/vecf/boxm2_vecf_orbicularis_oris.h>


bool test_orbicularis_oris_params()
{
  boxm2_vecf_orbicularis_oris_params pr;
  pr.principal_eigenvector_1_.set(1.0, 1.0, 1.0);
  pr.principal_eigenvector_2_.set(2.0, 0.0, 1.0);
  pr.lambda_ = 0.8;
  pr.gamma_ = 0.5;
  pr.planar_surface_dist_thresh_ = 2.0;
  pr.tilt_angle_in_deg_ = 1.0;
  pr.scale_factor_ = 1.0;
  vcl_string param_file = "./boxm2_vecf_orbicularis_oris_params.txt";
  vcl_ofstream ostr(param_file);
  ostr << pr;
  ostr.close();
  // read in
  boxm2_vecf_orbicularis_oris_params pr_in;
  vcl_ifstream istr(param_file);
  istr >> pr_in;
  istr.close();
  bool good = pr_in.principal_eigenvector_1_ == pr.principal_eigenvector_1_;
  good     &= pr_in.principal_eigenvector_2_ == pr.principal_eigenvector_2_;
  good     &= vcl_fabs(pr_in.lambda_ - pr.lambda_) < 1E-5;
  good     &= vcl_fabs(pr_in.gamma_ - pr.gamma_) < 1E-5;
  return good;
}

bool test_orbicularis_oris_doformation(vcl_string const& geometry_file, vcl_string const& param_file)
{
  vcl_string base_dir = vul_file::dirname(geometry_file);
  boxm2_vecf_orbicularis_oris oom(geometry_file);
  vcl_cout << "number of knots along axis: " << oom.axis().knots().size() << vcl_endl;
  vcl_cout << "number of cross sections: " << oom.n_cross_sections() << vcl_endl;

  // set the parameters
  boxm2_vecf_orbicularis_oris_params pr;
  vcl_ifstream istr(param_file);
  istr >> pr;
  istr.close();

  vcl_string init_axis_wrl  = base_dir + "/initial_axis.wrl";
  vcl_string init_pts_wrl   = base_dir + "/initial_pts.wrl";
  vcl_string init_cross_wrl = base_dir + "/initial_cross.wrl";
  vcl_string init_axis_file = base_dir + "/initial_axis.txt";
  vcl_string init_pts_file  = base_dir + "/initial_pts.txt";

  vcl_ofstream aostr_init(init_axis_file.c_str());
  oom.display_axis_spline_ascii(aostr_init);
  vcl_ofstream postr_init(init_pts_file.c_str());
  oom.display_cross_section_pointsets_ascii(postr_init);
  oom.display_axis_spline(vcl_ofstream(init_axis_wrl.c_str()));
  oom.display_cross_section_pointsets(vcl_ofstream(init_pts_wrl.c_str()));
  oom.display_cross_section_planes(vcl_ofstream(init_cross_wrl.c_str()));


  // test scale operation
  // enlarge
  boxm2_vecf_orbicularis_oris_params pr_scale = pr;
  pr_scale.scale_factor_ = 1.5;
  oom.set_params(pr_scale);
  boxm2_vecf_orbicularis_oris oom_large = oom.scale();

  vcl_string large_axis_wrl  = base_dir + "/enlarge_axis.wrl";
  vcl_string large_cross_wrl = base_dir + "/enlarge_cross.wrl";
  vcl_string large_pts_wrl   = base_dir + "/enlarge_pts.wrl";
  vcl_string large_axis_file = base_dir + "/enlarge_axis.txt";
  vcl_string large_pts_file  = base_dir + "/enlarge_pts.txt";
  oom_large.display_axis_spline_ascii(vcl_ofstream(large_axis_file.c_str()), 0, 0, 255);
  oom_large.display_cross_section_pointsets_ascii(vcl_ofstream(large_pts_file.c_str()), 0, 0, 255);
  oom_large.display_axis_spline(vcl_ofstream(large_axis_wrl.c_str()));
  oom_large.display_cross_section_planes(vcl_ofstream(large_cross_wrl.c_str()));
  oom_large.display_cross_section_pointsets(vcl_ofstream(large_pts_wrl.c_str()));

  // shrink
  pr_scale.scale_factor_ = 0.5;
  oom.set_params(pr_scale);
  boxm2_vecf_orbicularis_oris oom_shrink = oom.scale();

  vcl_string shrink_axis_wrl  = base_dir + "/shrink_axis.wrl";
  vcl_string shrink_cross_wrl = base_dir + "/shrink_cross.wrl";
  vcl_string shrink_pts_wrl   = base_dir + "/shrink_pts.wrl";
  vcl_string shrink_axis_file = base_dir + "/shrink_axis.txt";
  vcl_string shrink_pts_file  = base_dir + "/shrink_pts.txt";
  oom_shrink.display_axis_spline_ascii(vcl_ofstream(shrink_axis_file.c_str()), 255, 0, 0);
  oom_shrink.display_cross_section_pointsets_ascii(vcl_ofstream(shrink_pts_file.c_str()), 255, 0, 0);
  oom_shrink.display_axis_spline(vcl_ofstream(shrink_axis_wrl.c_str()));
  oom_shrink.display_cross_section_planes(vcl_ofstream(shrink_cross_wrl.c_str()));
  oom_shrink.display_cross_section_pointsets(vcl_ofstream(shrink_pts_wrl.c_str()));

  // test tilt operation
  // open
  boxm2_vecf_orbicularis_oris_params pr_tilt = pr;
  pr_tilt.tilt_angle_in_deg_ = 25.0;
  oom.set_params(pr_tilt);
  boxm2_vecf_orbicularis_oris oom_open = oom.tilt();

  vcl_string open_axis_wrl  = base_dir + "/mouth_open_25_deg_axis.wrl";
  vcl_string open_cross_wrl = base_dir + "/mouth_open_25_deg_cross.wrl";
  vcl_string open_pts_wrl   = base_dir + "/mouth_open_25_deg_pts.wrl";
  vcl_string open_axis_file = base_dir + "/mouth_open_25_deg_axis.txt";
  vcl_string open_pts_file  = base_dir + "/mouth_open_25_deg_pts.txt";
  oom_open.display_axis_spline_ascii(vcl_ofstream(open_axis_file.c_str()), 0, 255, 255);
  oom_open.display_cross_section_pointsets_ascii(vcl_ofstream(open_pts_file.c_str()), 0, 255, 255);
  oom_open.display_axis_spline(vcl_ofstream(open_axis_wrl.c_str()));
  oom_open.display_cross_section_planes(vcl_ofstream(open_cross_wrl.c_str()));
  oom_open.display_cross_section_pointsets(vcl_ofstream(open_pts_wrl.c_str()));

  // close
  pr_tilt.tilt_angle_in_deg_ = -20.0;
  oom.set_params(pr_tilt);
  boxm2_vecf_orbicularis_oris oom_close = oom.tilt();

  vcl_string close_axis_wrl  = base_dir + "/mouth_close_20_deg_axis.wrl";
  vcl_string close_cross_wrl = base_dir + "/mouth_close_20_deg_cross.wrl";
  vcl_string close_pts_wrl   = base_dir + "/mouth_close_20_deg_pts.wrl";
  vcl_string close_axis_file = base_dir + "/mouth_close_20_deg_axis.txt";
  vcl_string close_pts_file  = base_dir + "/mouth_close_20_deg_pts.txt";
  oom_close.display_axis_spline_ascii(vcl_ofstream(close_axis_file.c_str()), 138, 43, 226);
  oom_close.display_cross_section_pointsets_ascii(vcl_ofstream(close_pts_file.c_str()), 138, 43, 226);
  oom_close.display_axis_spline(vcl_ofstream(close_axis_wrl.c_str()));
  oom_close.display_cross_section_planes(vcl_ofstream(close_cross_wrl.c_str()));
  oom_close.display_cross_section_pointsets(vcl_ofstream(close_pts_wrl.c_str()));

  // test pucker deformation
  oom.set_params(pr);
  boxm2_vecf_orbicularis_oris oom_deformed = oom.circular_deform();

  vcl_string out_axis_vrml  = base_dir + "/pucker_axis.wrl";
  vcl_string out_pts_vrml   = base_dir + "/pucker_pts.wrl";
  vcl_string out_cross_vrml = base_dir + "/pucker_cross.wrl";
  vcl_string def_axis_file  = base_dir + "/pucker_axis.txt";
  vcl_string def_pts_file   = base_dir + "/pucker_pts.txt";

  vcl_ofstream aostr_vrml(out_axis_vrml.c_str());
  oom_deformed.display_axis_spline(aostr_vrml);
  vcl_ofstream postr_vrml(out_pts_vrml.c_str());
  oom_deformed.display_cross_section_pointsets(postr_vrml);
  vcl_ofstream costr_vrml(out_cross_vrml.c_str());
  oom_deformed.display_cross_section_planes(costr_vrml);
  vcl_ofstream aostr_def(def_axis_file.c_str());
  oom_deformed.display_axis_spline_ascii(aostr_def, 255, 255, 0);
  vcl_ofstream postr_def(def_pts_file.c_str());
  oom_deformed.display_cross_section_pointsets_ascii(postr_def, 255, 255, 0);


  return true;
}

void test_orbicularis_oris()
{
  bool success = test_orbicularis_oris_params();
  TEST("orbicular oris parameter" , success, true);

  vcl_string param_file = "./boxm2_vecf_orbicularis_oris_params.txt";
  vcl_string geometry_file = "D:/work/janus/Orbicularis_Oris/expt/geometry_file.txt";
  if ( vul_file::exists(param_file) && vul_file::exists(geometry_file)) {
    success = test_orbicularis_oris_doformation(geometry_file, param_file);
    TEST("orbicular oris deformation", success, true);
  }
  else {
    vcl_cout << "Missing input geometry file: " << geometry_file << ", test boxm2_vecf_orbicularis_oris ignored." << vcl_endl;
  }
  return;
}

TESTMAIN( test_orbicularis_oris );