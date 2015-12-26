#ifndef boxm2_vecf_skin_params_h_
#define boxm2_vecf_skin_params_h_
//:
// \file
// \brief  Parameters for the skin model
//
// \author J.L. Mundy
// \date   3 November 2015
//
#include "boxm2_vecf_articulated_params.h"
#include <vcl_iostream.h>
#include <vgl/vgl_vector_3d.h>
class boxm2_vecf_skin_params : public boxm2_vecf_articulated_params{
 public:

 boxm2_vecf_skin_params():  offset_(vgl_vector_3d<double>(0.0, 0.0, 0.0)), skin_intensity_(static_cast<unsigned char>(130)){
  }

 boxm2_vecf_skin_params(unsigned char skin_intensity):skin_intensity_(skin_intensity){
  }
  unsigned char skin_intensity_;
  vgl_vector_3d<double> offset_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_skin_params const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_skin_params& pr);

#endif// boxm2_vecf_skin_params
