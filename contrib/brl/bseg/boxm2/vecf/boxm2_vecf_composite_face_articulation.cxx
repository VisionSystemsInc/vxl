#include "boxm2_vecf_composite_face_articulation.h"
#include <vcl_fstream.h>
void boxm2_vecf_composite_face_articulation::initialize(){

  //======================= default articulation =====================
  vcl_vector<boxm2_vecf_composite_face_params> default_articulation;
  // initial params has the member trans_ that is an initial guess to
  // the global affine transformation between the source and target scenes
  boxm2_vecf_composite_face_params params = initial_params_;
  params.skin_params_.skin_is_transparent_ = true;
  params.mouth_params_.show_mouth_region_ = true;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.05;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.1;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.15;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.2;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.25;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.3;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.35;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.4;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.45;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.5;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.45;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.35;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.3;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.25;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.15;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.1;
  default_articulation.push_back(params);
  params.mandible_params_.jaw_opening_angle_rad_ = 0.05;
  default_articulation.push_back(params);
  play_sequence_map_["default"] = default_articulation;
  this->set_play_sequence("default") ;
}
