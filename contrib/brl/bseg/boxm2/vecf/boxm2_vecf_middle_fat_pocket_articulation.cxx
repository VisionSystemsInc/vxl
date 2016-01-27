#include "boxm2_vecf_middle_fat_pocket_articulation.h"

boxm2_vecf_middle_fat_pocket_articulation::boxm2_vecf_middle_fat_pocket_articulation(){

  //======================= default articulation =====================
  vcl_vector<boxm2_vecf_middle_fat_pocket_params> default_articulation;
  boxm2_vecf_middle_fat_pocket_params  params;
  default_articulation.push_back(params);
  params.lambda_ = 0.85;
  default_articulation.push_back(params);
  play_sequence_map_["default"] = default_articulation;
  this->set_play_sequence("default") ;
}
