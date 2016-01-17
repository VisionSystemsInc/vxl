#ifndef boxm2_vecf_fat_pocket_params_h_
#define boxm2_vecf_fat_pocket_params_h_
//:
// \file
// \brief  Parameters for the fat_pocket model
//
// \author J.L. Mundy
// \date   30 December 2015
//
#include "boxm2_vecf_articulated_params.h"
#include <vcl_iostream.h>
#include <vgl/vgl_vector_3d.h>
#include <vnl/vnl_vector_fixed.h>
class boxm2_vecf_fat_pocket_params : public boxm2_vecf_articulated_params{
 public:
 boxm2_vecf_fat_pocket_params(): offset_(vgl_vector_3d<double>(0.0, 0.0, 0.0)){
  }
 vgl_vector_3d<double> offset_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_fat_pocket_params const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_fat_pocket_params& pr);

#endif// boxm2_vecf_fat_pocket_params
