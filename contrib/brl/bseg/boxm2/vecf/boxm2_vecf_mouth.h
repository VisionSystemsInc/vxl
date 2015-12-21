#ifndef boxm2_vecf_mouth_h_
#define boxm2_vecf_mouth_h_
//:
// \file
// \brief  The mouth component of the head
//
// \author J.L. Mundy
// \date   20 Dec 2015
//
#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vcl_vector.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <bvgl/bvgl_spline_region_3d.h>
#include "boxm2_vecf_mandible_params.h"
class boxm2_vecf_mouth{
 public:
  boxm2_vecf_mouth(){}
  boxm2_vecf_mouth(vcl_string const& geometry_file){};

  boxm2_vecf_mouth(vcl_vector<vgl_point_3d<double> >  const& knots);
  boxm2_vecf_mouth(vgl_pointset_3d<double>  const& ptset);

  void read_mouth(vcl_istream& istr);

  void set_mandible_params(boxm2_vecf_mandible_params const& mand_params);

  vgl_box_3d<double> bounding_box() const;

  bool in(vgl_point_3d<double> const& pt) const;
  
  //:for debug purposes
  void display_vrml(vcl_ofstream& ostr) const;
  void rotate_inf();
  vgl_pointset_3d<double> random_pointset(unsigned n_pts) const;
 private:

  bvgl_spline_region_3d<double> sup_;
  bvgl_spline_region_3d<double> inf_;
  boxm2_vecf_mandible_params mand_params_;
  vgl_rotation_3d<double> rot_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_mouth const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_mouth& pr);

#endif// boxm2_vecf_mouth
