#ifndef boxm2_vecf_composite_face_params_h_
#define boxm2_vecf_composite_face_params_h_
//:
// \file
// \brief  Parameters for the composite_face model
//
// \author J.L. Mundy
// \date   3 November 2015
//
#include "boxm2_vecf_articulated_params.h"
#include <vcl_iostream.h>
#include <vgl/vgl_vector_3d.h>
#include "boxm2_vecf_mandible_params.h"
#include "boxm2_vecf_cranium_params.h"
class boxm2_vecf_composite_face_params : public boxm2_vecf_articulated_params{
 public:

 boxm2_vecf_composite_face_params():  offset_(vgl_vector_3d<double>(0.0, 0.0, 0.0)), composite_face_intensity_((unsigned char)(200)){
  }

 boxm2_vecf_composite_face_params(unsigned char composite_face_intensity):composite_face_intensity_(composite_face_intensity){
  }
  unsigned char composite_face_intensity_;
  vgl_vector_3d<double> offset_;
  boxm2_vecf_mandible_params mandible_params_;
  boxm2_vecf_cranium_params cranium_params_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_composite_face_params const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_composite_face_params& pr);

#endif// boxm2_vecf_composite_face_params
