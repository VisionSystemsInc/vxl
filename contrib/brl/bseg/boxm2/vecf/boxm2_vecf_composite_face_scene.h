#ifndef boxm2_vecf_composite_face_scene_h_
#define boxm2_vecf_composite_face_scene_h_

#include <vcl_string.h>
#include <vgl/vgl_box_3d.h>
#include <boxm2/boxm2_scene.h>
#include "boxm2_vecf_mandible_scene.h"
#include "boxm2_vecf_cranium_scene.h"
#include "boxm2_vecf_skin_scene.h"
#include "boxm2_vecf_articulated_params.h"
#include "boxm2_vecf_composite_face_params.h"
#include "boxm2_vecf_articulated_scene.h"
#include "boxm2_vecf_mouth.h"
struct unrefined_cell_info{
unrefined_cell_info():linear_index_(0), ix_(0), iy_(0), iz_(0){}
  unsigned linear_index_;
  unsigned ix_;
  unsigned iy_;
  unsigned iz_;
  vgl_point_3d<double> pt_;
};
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
  // old version now deprecated - remove at some point
  virtual int prerefine_target_sub_block(vgl_point_3d<int> const& sub_block_index);
  // current version
  virtual int prerefine_target_sub_block(vgl_point_3d<double> const& sub_block_pt, unsigned pt_index);

  //implemented on parent but specialized here
  virtual void prerefine_target(boxm2_scene_sptr target_scene);

  virtual bool inverse_vector_field(vgl_point_3d<double> const& target_pt, vgl_vector_3d<double>& inv_vf) const {return false;}
  virtual bool apply_vector_field(cell_info const& target_cell, vgl_vector_3d<double> const& inv_vf){ return false;}

  //: compute an inverse vector field for rotation of skin
  void inverse_vector_field(vcl_vector<vgl_vector_3d<double> >& vfield, vcl_vector<vcl_string>& type) const;

  //: apply the vector field
  void apply_vector_field_to_target(vcl_vector<vgl_vector_3d<double> > const& vfield, vcl_vector<vcl_string> const& type);

  //: get the cell centers
  void extract_target_cell_centers();

  //: compute target box from point cloud
  // the point cloud is a target to compare with source
  void compute_target_box( vcl_string const& pc_path);
  vgl_box_3d<double> target_box() const {return target_box_;}
  // for debugging purposes
  void set_target_box(vgl_box_3d<double> const& target_box){ target_box_ = target_box;}

  //: construct the target scene from the bounding box  
  boxm2_scene_sptr construct_target_scene(vcl_string const& scene_dir,vcl_string const& scene_name, vcl_string const& data_path, double sub_block_len,
                                         bool save_scene_xml = true);

 private:
  void extract_unrefined_cell_info();
  vcl_vector<unrefined_cell_info> unrefined_cell_info_;
  vgl_box_3d<double> target_box_;
  vgl_box_3d<double> coupling_box_;
  boxm2_vecf_mouth mouth_geo_;
  boxm2_vecf_composite_face_params params_;
  boxm2_vecf_articulated_scene_sptr mandible_;
  boxm2_vecf_articulated_scene_sptr cranium_;
  boxm2_vecf_articulated_scene_sptr skin_;
  vcl_vector<cell_info> target_cell_centers_;       // cell centers in the refined target block
};

#endif
