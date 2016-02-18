#ifndef boxm2_vecf_middle_fat_pocket_params_h_
#define boxm2_vecf_middle_fat_pocket_params_h_
//:
// \file
// \brief  Parameters for the middle_fat_pocket model
//
// \author J.L. Mundy
// \date   30 December 2015
//
#include "boxm2_vecf_articulated_params.h"
#include <vcl_iostream.h>
#include <vgl/vgl_vector_3d.h>
#include <vnl/vnl_vector_fixed.h>
class boxm2_vecf_middle_fat_pocket_params : public boxm2_vecf_articulated_params{
 public:
 boxm2_vecf_middle_fat_pocket_params(): origin_(vgl_point_3d<double>(26.14, -7.45, 72.1)),
    normal_(vgl_vector_3d<double>(0.73911, -0.2, 0.666008)),
    //    principal_eigenvector_(vgl_vector_3d<double>(0.55,0.68,-0.42)), lambda_(1.0), gamma_(0.2), tolerance_(0.5),
    principal_eigenvector_(vgl_vector_3d<double>(0.55,0.68,-0.42)), lambda_(1.0), gamma_(0.8), principal_offset_(10.0), tolerance_(0.5),
    max_norm_distance_(25.0), scale_at_midpt_(0.85), scale_at_max_(0.5),middle_fat_pocket_intensity_(static_cast<unsigned char>(50)){
  }
 vgl_point_3d<double> origin_;
 vgl_vector_3d<double> normal_;
 vgl_vector_3d<double> principal_eigenvector_;
 double principal_offset_;
 double lambda_;
 double gamma_;
 double tolerance_;
 double max_norm_distance_;
 double scale_at_midpt_;
 double scale_at_max_;
 unsigned char middle_fat_pocket_intensity_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_middle_fat_pocket_params const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_middle_fat_pocket_params& pr);

#endif// boxm2_vecf_middle_fat_pocket_params
