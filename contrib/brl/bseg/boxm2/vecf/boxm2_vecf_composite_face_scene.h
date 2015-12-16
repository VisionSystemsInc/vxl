#ifndef boxm2_vecf_composite_face_scene_h_
#define boxm2_vecf_composite_face_scene_h_

#include <vcl_string.h>

#include <boxm2/boxm2_scene.h>
#include "boxm2_vecf_mandible_scene.h"
#include "boxm2_vecf_cranium_scene.h"
#include "boxm2_vecf_skin_scene.h"
#include "boxm2_vecf_articulated_params.h"
#include "boxm2_vecf_composite_face_params.h"
#include "boxm2_vecf_articulated_scene.h"

class boxm2_vecf_composite_face_scene : public boxm2_vecf_articulated_scene{
public:
  enum comp_type {MANDIBLE, CRANIUM, SKIN, NO_TYPE};
  boxm2_vecf_composite_face_scene(vcl_string const& face_scene_paths);

  void map_to_target(boxm2_scene_sptr target);

  bool set_params(boxm2_vecf_articulated_params const& params);
  boxm2_vecf_composite_face_params const& get_params() const {return params_;}

  //: find the inverse vector field for unrefined target block centers
  virtual void inverse_vector_field_unrefined(boxm2_scene_sptr target_scene);

  //: refine target cells to match the refinement level of the source block
  virtual int prerefine_target_sub_block(vgl_point_3d<int> const& sub_block_index);

  virtual bool inverse_vector_field(vgl_point_3d<double> const& target_pt, vgl_vector_3d<double>& inv_vf) const {return false;}
  virtual bool apply_vector_field(cell_info const& target_cell, vgl_vector_3d<double> const& inv_vf){ return false;}

  //: compute an inverse vector field for rotation of skin
  void inverse_vector_field(vcl_vector<vgl_vector_3d<double> >& vfield, vcl_vector<vcl_string>& type) const;

  //: apply the vector field
  void apply_vector_field_to_target(vcl_vector<vgl_vector_3d<double> > const& vfield, vcl_vector<vcl_string> const& type);

  //: get the cell centers
  void extract_target_cell_centers();
private:
  boxm2_vecf_composite_face_params params_;
  boxm2_vecf_articulated_scene_sptr mandible_;
  boxm2_vecf_articulated_scene_sptr cranium_;
  boxm2_vecf_articulated_scene_sptr skin_;
  vcl_vector<cell_info> target_cell_centers_;       // cell centers in the refined target block
};

#endif
