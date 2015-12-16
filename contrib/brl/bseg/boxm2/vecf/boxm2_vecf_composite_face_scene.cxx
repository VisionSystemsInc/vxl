#include "boxm2_vecf_composite_face_scene.h"
#include <vul/vul_file.h>
#include <vul/vul_timer.h>
#include <vcl_fstream.h>
boxm2_vecf_composite_face_scene::boxm2_vecf_composite_face_scene(vcl_string const& face_scene_paths){
  vcl_ifstream istr(face_scene_paths.c_str());
  // construct paths to component scene xml files
  vcl_map<vcl_string, vcl_string> scene_path_map;
  vcl_string component, path;
  while(istr >> component >> path)
    scene_path_map[component] = path;

  vcl_string base_path, mandible_path, cranium_path, skin_path;

  vcl_map<vcl_string, vcl_string>::iterator pit;

  pit = scene_path_map.find("base_path");
  if(pit == scene_path_map.end()){
    vcl_cout << "FATAL - base_path not defined\n";
    return;
  }else
    base_path = pit->second;
  
  pit = scene_path_map.find("mandible_path");
  if(pit == scene_path_map.end()){
    vcl_cout << "FATAL - mandible_path not defined\n";
    return;
  }else
    mandible_path = base_path + pit->second;
  if(!vul_file::exists(mandible_path)){
    vcl_cout << "FATAL - " << mandible_path << " does not exist\n";
    return;
  }
  pit = scene_path_map.find("cranium_path");
  if(pit == scene_path_map.end()){
    vcl_cout << "FATAL - cranium_path not defined\n";
    return;
  }else
    cranium_path = base_path + pit->second;

  if(!vul_file::exists(cranium_path)){
    vcl_cout << "FATAL - " << cranium_path << " does not exist\n";
    return;
  }
  pit = scene_path_map.find("skin_path");
  if(pit == scene_path_map.end()){
    vcl_cout << "FATAL - skin_path not defined\n";
    return;
  }else
    skin_path = base_path + pit->second;

  if(!vul_file::exists(skin_path)){
    vcl_cout << "FATAL - " << skin_path << " does not exist\n";
    return;
  }
  //load the scenes
  mandible_ = new boxm2_vecf_mandible_scene(mandible_path);


  cranium_ = new boxm2_vecf_cranium_scene(cranium_path);
#if 0
  skin_ = new boxm2_vecf_skin_scene(skin_path);

#endif
  skin_ = 0;
}

//: compute the inverse vector field 
void boxm2_vecf_composite_face_scene::inverse_vector_field(vcl_vector<vgl_vector_3d<double> >& vfield, vcl_vector<vcl_string>& type) const{
  vul_timer t;
  //the target cell centers. the vector field could potentially be defined at all target points
  unsigned nt = static_cast<unsigned>(target_cell_centers_.size());
  vfield.resize(nt, vgl_vector_3d<double>(0.0, 0.0, 0.0));// initialized to 0
  type.resize(nt, "invalid");
  unsigned mandible_cnt = 0, skin_cnt = 0, cranium_cnt = 0;
  for(unsigned i = 0; i<nt; ++i){
    vgl_vector_3d<double> mandible_inv_vf, cranium_inv_vf, skin_inv_vf; //need to iterate over components here (left off)
    bool mandible_valid=false, cranium_valid=false, skin_valid=false;
    mandible_valid = mandible_->inverse_vector_field(target_cell_centers_[i].cell_center_,mandible_inv_vf);
    cranium_valid = cranium_->inverse_vector_field(target_cell_centers_[i].cell_center_, cranium_inv_vf);
    ///skin_valid = skin_->inverse_vector_field(target_cell_centers_[i].cell_center_, skin_inv_vf);
    bool not_valid = !mandible_valid&&!cranium_valid&&!skin_valid;
    if(not_valid){
      continue;
    }
    if(cranium_valid){
      type[i]="cranium";
      vfield[i].set(cranium_inv_vf.x(), cranium_inv_vf.y(), cranium_inv_vf.z());
      cranium_cnt++;
    }else if(mandible_valid){
      type[i]="mandible";
      vfield[i].set(mandible_inv_vf.x(), mandible_inv_vf.y(), mandible_inv_vf.z());
      mandible_cnt++;
    }else if(skin_valid){
      type[i]="skin";
      vfield[i].set(skin_inv_vf.x(), skin_inv_vf.y(), skin_inv_vf.z());
      skin_cnt++;
    }
  }
  vcl_cout << "computed " << mandible_cnt << " mandible pts "<< skin_cnt << " skin pts and " 
          << cranium_cnt << " cranium pts out of " << nt << " for face vector field in " << t.real()/1000.0 << " sec.\n";
}

void boxm2_vecf_composite_face_scene::map_to_target(boxm2_scene_sptr target){
  vul_timer t;
  if(!target_data_extracted_)
    this->extract_target_block_data(target);
  // compute inverse vector field for prerefining the target
  mandible_->extract_target_block_data(target);
  mandible_->inverse_vector_field_unrefined(target);

  cranium_->extract_target_block_data(target);
  cranium_->inverse_vector_field_unrefined(target);
#if 0
  skin_->extract_target_block_data(target);
  skin_->inverse_vector_field_unrefined(target);
#endif
  this->prerefine_target(target);
  this->extract_target_block_data(target);
  mandible_->extract_target_block_data(target);

  cranium_->extract_target_block_data(target);
#if 0
  skin_->extract_target_block_data(target);
#endif
  this->extract_target_cell_centers();
  vcl_vector<vgl_vector_3d<double> > vfield;
  vcl_vector<vcl_string> type;
  this->inverse_vector_field(vfield,  type);
  this->apply_vector_field_to_target(vfield, type);
}

bool boxm2_vecf_composite_face_scene::set_params(boxm2_vecf_articulated_params const& params){
  return true;
}

void boxm2_vecf_composite_face_scene::inverse_vector_field_unrefined(boxm2_scene_sptr target_scene){
  
}

int boxm2_vecf_composite_face_scene::prerefine_target_sub_block(vgl_point_3d<int> const& sub_block_index){
  int max_depth = -1;
  int depth_mandible = mandible_->prerefine_target_sub_block(sub_block_index);
  if(depth_mandible>max_depth)
    max_depth = depth_mandible;

  int depth_cranium = cranium_->prerefine_target_sub_block(sub_block_index);
  if(depth_cranium>max_depth)
    max_depth = depth_cranium;
#if 0
  int depth_skin = skin_->prerefine_target_sub_block(sub_block_index);
  if(depth_skin>max_depth)
    max_depth = depth_skin;
#endif
  return max_depth;
}

void boxm2_vecf_composite_face_scene::extract_target_cell_centers(){
  vgl_box_3d<double> target_bb = target_blk_->bounding_box_global();
  //vgl_box_3d<double> target_bb = cranium_->source_bounding_box();
  target_cell_centers_ = target_blk_->cells_in_box(target_bb);
}


void boxm2_vecf_composite_face_scene::apply_vector_field_to_target(vcl_vector<vgl_vector_3d<double> > const& vf, vcl_vector<vcl_string> const& type){
  boxm2_data_traits<BOXM2_ALPHA>::datatype alpha = 0.0f;
  unsigned n_valid = 0;
  unsigned n = static_cast<unsigned>(vf.size());
  for(unsigned j = 0; j<n; ++j){
    const vcl_string& t = type[j];
    bool fail = true;
    if(t == "cranium"){
     fail = !cranium_->apply_vector_field(target_cell_centers_[j], vf[j]);
    }else if(t == "mandible"){
      fail = !mandible_->apply_vector_field(target_cell_centers_[j], vf[j]);
    }
    if(t == "invalid" || fail){
      unsigned tindx = target_cell_centers_[j].data_index_;
      alpha = 0.0f;//default to no occlusion
      target_alpha_data_[tindx] = alpha;
      continue;
    }else
      n_valid++;
  }
  vcl_cout << "mapped "  << n_valid << " source points out of " << n << " total points\n";
}

