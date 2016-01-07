#include "boxm2_vecf_composite_face_scene.h"
#include <vcl_fstream.h>
#include <vul/vul_file.h>
#include <vul/vul_timer.h>
#include <vcl_fstream.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_bounding_box.h>
#include <vgl/algo/vgl_h_matrix_3d.h>
#include <boxm2/cpp/algo/boxm2_refine_block_multi_data.h>
boxm2_vecf_composite_face_scene::boxm2_vecf_composite_face_scene(vcl_string const& face_scene_paths){
  //points defining the jaw coupled vector field bounding box in the source scene
  //the vector field from the mandible is propagated to all skin voxels within the box
  // unless invalid as defined by the articulated mouth region
  vgl_point_3d<double> p0(-82.0,-60.0, 0.0);  vgl_point_3d<double> p1(82.0,-60.0, 0.0);
  vgl_point_3d<double> p2(-56.0,-150.0,0.0);  vgl_point_3d<double> p3(56.0,-150.0,0.0);
  vgl_point_3d<double> p4(0,-60.,120.0);
  coupling_box_.add(p0); coupling_box_.add(p1); coupling_box_.add(p2); coupling_box_.add(p3); coupling_box_.add(p4);

  vcl_ifstream istr(face_scene_paths.c_str());
  // construct paths to component scene xml files
  vcl_map<vcl_string, vcl_string> scene_path_map;
  vcl_string component, path;
  while(istr >> component >> path)
    scene_path_map[component] = path;

  vcl_string base_path, mandible_path, cranium_path, skin_path, mouth_path, param_path;

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

  pit = scene_path_map.find("mouth_path");
  if(pit == scene_path_map.end()){
    vcl_cout << "FATAL - mouth_path not defined\n";
    return;
  }else
    mouth_path = base_path + pit->second;

  if(!vul_file::exists(mouth_path)){
    vcl_cout << "FATAL - " << mouth_path << " does not exist\n";
    return;
  }

  pit = scene_path_map.find("param_path");
  if(pit != scene_path_map.end()){
    param_path = pit->second;
    if(!vul_file::exists(param_path)){
      vcl_cout << "FATAL - " << param_path << " does not exist\n";
      return;
    }
    vcl_ifstream pistr(param_path.c_str());
    pistr >> params_;
  }

  //load the scenes
  mandible_ = new boxm2_vecf_mandible_scene(mandible_path);
  cranium_ = new boxm2_vecf_cranium_scene(cranium_path);
  skin_ = new boxm2_vecf_skin_scene(skin_path);

  vgl_pointset_3d<double> ptset;
  vcl_ifstream mstr(mouth_path.c_str());
  if(!mstr)
    return;
  mstr >> ptset;
  mstr.close();
  mouth_geo_ = boxm2_vecf_mouth(ptset);
}
bool  boxm2_vecf_composite_face_scene::inverse_vector_field(vgl_point_3d<double> const& target_pt, vgl_vector_3d<double>& inv_vf, vcl_string& anatomy_type) const{
  vgl_vector_3d<double> mandible_inv_vf, cranium_inv_vf, skin_inv_vf;
  bool mandible_valid=false, cranium_valid=false, skin_valid=false, in_mouth = false;
  if(mandible_)
    mandible_valid = mandible_->inverse_vector_field(target_pt, mandible_inv_vf);
  if(cranium_)
    cranium_valid = cranium_->inverse_vector_field(target_pt, cranium_inv_vf);
  if(skin_){
    if(coupling_box_.contains(target_pt)){
      skin_valid = mandible_->coupled_vector_field(target_pt, skin_inv_vf);
      in_mouth = mouth_geo_.in(target_pt);
    }else
      skin_valid = skin_->inverse_vector_field(target_pt, skin_inv_vf);
  }
  bool not_valid = (!mandible_valid&&!cranium_valid&&!skin_valid);
  if(not_valid){
    return false;
  }
  bool mouth_skin_invalid = skin_valid&&in_mouth;
  if(mouth_skin_invalid){
    anatomy_type = "mouth";
    inv_vf.set(0.0, 0.0, 0.0);
  }else if(mandible_valid){
    anatomy_type="mandible";
    inv_vf.set(mandible_inv_vf.x(), mandible_inv_vf.y(), mandible_inv_vf.z());
  }else if(skin_valid){
    anatomy_type="skin";
    inv_vf.set(skin_inv_vf.x(), skin_inv_vf.y(), skin_inv_vf.z());
  }else if(cranium_valid){
    anatomy_type="cranium";
    inv_vf.set(cranium_inv_vf.x(), cranium_inv_vf.y(), cranium_inv_vf.z());
  }
  return true;
}
//: compute the inverse vector field, first undoing the affine map to the target
void  boxm2_vecf_composite_face_scene::inverse_vector_field(vcl_vector<vgl_vector_3d<double> >& vfield, vcl_vector<vcl_string>& type) const{
  vul_timer t;

  //the target cell centers. the vector field could potentially be defined at all target points
  unsigned nt = static_cast<unsigned>(target_cell_centers_.size());
  vfield.resize(nt, vgl_vector_3d<double>(0.0, 0.0, 0.0));// initialized to 0
  type.resize(nt, "invalid");
  unsigned mandible_cnt = 0, skin_cnt = 0, cranium_cnt = 0;
  for(unsigned i = 0; i<nt; ++i){
    vgl_vector_3d<double> inv_vf;
    vcl_string anatomy_type;
    const vgl_point_3d<double>& p_inv = target_cell_centers_[i].cell_center_;
    bool valid = this->inverse_vector_field(p_inv, inv_vf, anatomy_type);
    if(!valid)
      continue;
    vfield[i].set(inv_vf.x(), inv_vf.y(), inv_vf.z());
    type[i]=anatomy_type;
#if 0
    vgl_vector_3d<double> mandible_inv_vf, cranium_inv_vf, skin_inv_vf; //need to iterate over components here (left off)
    bool mandible_valid=false, cranium_valid=false, skin_valid=false, in_mouth = false;
    if(mandible_)
      mandible_valid = mandible_->inverse_vector_field(p_inv, mandible_inv_vf);
    if(cranium_)
      cranium_valid = cranium_->inverse_vector_field(p_inv, cranium_inv_vf);
    if(skin_){
      if(coupling_box_.contains(p_inv)){
        skin_valid = mandible_->coupled_vector_field(p_inv, skin_inv_vf);
        in_mouth = mouth_geo_.in(p_inv);
      }else
        skin_valid = skin_->inverse_vector_field(p_inv, skin_inv_vf);
        }

    bool not_valid = (!mandible_valid&&!cranium_valid&&!skin_valid);
    if(not_valid){
      continue;
    }
    bool mouth_skin_invalid = skin_valid&&in_mouth;
    if(mouth_skin_invalid){
      type[i] = "mouth";
      vfield[i].set(0.0, 0.0, 0.0);
    }else if(mandible_valid){
      type[i]="mandible";
      vfield[i].set(mandible_inv_vf.x(), mandible_inv_vf.y(), mandible_inv_vf.z());
      mandible_cnt++;
    }else if(skin_valid){
      type[i]="skin";
      vfield[i].set(skin_inv_vf.x(), skin_inv_vf.y(), skin_inv_vf.z());
      skin_cnt++;
    }else if(cranium_valid){
      type[i]="cranium";
      vfield[i].set(cranium_inv_vf.x(), cranium_inv_vf.y(), cranium_inv_vf.z());
      cranium_cnt++;
    }
#endif
  }
  //  vcl_cout << "computed " << mandible_cnt << " mandible pts "<< skin_cnt << " skin pts and " 
  //          << cranium_cnt << " cranium pts out of " << nt << " for face vector field in " << t.real()/1000.0 << " sec.\n";
}
void boxm2_vecf_composite_face_scene::extract_unrefined_cell_info(){
  if(!target_blk_){
    vcl_cout << "FATAL! - NULL target block\n";
    return;
  }
  // inverse mapping for global affine face transformation to a specific subject
  const vgl_h_matrix_3d<double>& Ainv = params_.trans_.get_inverse();

  //iterate through the trees of the target. At this point they are unrefined
  unrefined_cell_info_.resize(targ_n_.x()*targ_n_.y()*targ_n_.z());
  for(unsigned ix = 0; ix<targ_n_.x(); ++ix){
    for(unsigned iy = 0; iy<targ_n_.y(); ++iy){
      for(unsigned iz = 0; iz<targ_n_.z(); ++iz){
        double x = targ_origin_.x() + ix*targ_dims_.x();
        double y = targ_origin_.y() + iy*targ_dims_.y();
        double z = targ_origin_.z() + iz*targ_dims_.z();
        vgl_point_3d<double> p(x, y, z);
        unsigned lindex = static_cast<unsigned>(target_linear_index(ix, iy, iz));
        unrefined_cell_info cinf;
        cinf.linear_index_ = lindex;
        cinf.ix_ = ix; cinf.iy_ = iy; cinf.iz_ = iz;
        cinf.pt_=Ainv*p;
        unrefined_cell_info_[lindex]=cinf;
      }
    }
   }
}
void boxm2_vecf_composite_face_scene::map_to_target(boxm2_scene_sptr target){
  vul_timer t;
  static bool first = true;
  if(first){
  if(!target_data_extracted_)
    this->extract_target_block_data(target);

  this->extract_unrefined_cell_info();//on articulated_scene
  vcl_vector<vgl_point_3d<double> > tgt_pts;
  for(vcl_vector<unrefined_cell_info>::iterator cit = unrefined_cell_info_.begin();
      cit != unrefined_cell_info_.end(); ++cit){
    tgt_pts.push_back(cit->pt_);
  }

  // compute inverse vector field for prerefining the target
  if(mandible_){
   // mandible_->extract_target_block_data(target);
    mandible_->inverse_vector_field_unrefined(tgt_pts);
  }
  if(cranium_){
   // cranium_->extract_target_block_data(target);
   cranium_->inverse_vector_field_unrefined(tgt_pts);
  }
  if(skin_){
    //skin_->extract_target_block_data(target);
    skin_->inverse_vector_field_unrefined(tgt_pts);
  }

  this->prerefine_target(target);
  this->extract_target_block_data(target);

  if(mandible_)
     mandible_->extract_target_block_data(target);
  if(cranium_)
    cranium_->extract_target_block_data(target);
  if(skin_)
    skin_->extract_target_block_data(target);
  first = false;
  }
  this->extract_target_cell_centers();
  vcl_vector<vgl_vector_3d<double> > vfield;
  vcl_vector<vcl_string> type;
  this->inverse_vector_field(vfield,  type);
  this->apply_vector_field_to_target(vfield, type);
}

bool boxm2_vecf_composite_face_scene::set_params(boxm2_vecf_articulated_params const& params){
  try{
    boxm2_vecf_composite_face_params const& params_ref = dynamic_cast<boxm2_vecf_composite_face_params const &>(params);
    params_ =boxm2_vecf_composite_face_params(params_ref);
  }catch(std::exception e){
    vcl_cout<<" Can't downcast to composite_face parameters! PARAMATER ASSIGNMENT PHAILED!"<<vcl_endl;
    return false;
  }
  if(skin_)
    skin_->set_params(params_.skin_params_);
  if(mandible_){
    mandible_->set_params( params_.mandible_params_);
    mouth_geo_.set_mandible_params(params_.mandible_params_);
    mouth_geo_.set_params(params_.mouth_params_);
    vcl_cout << "======> Set jaw angle " << params_.mandible_params_.jaw_opening_angle_rad_ << '\n';
  }
  return true;
}

void boxm2_vecf_composite_face_scene::inverse_vector_field_unrefined(vcl_vector<vgl_point_3d<double> > const& unrefined_target_pts){
}

int boxm2_vecf_composite_face_scene::prerefine_target_sub_block(vgl_point_3d<double> const& sub_block_pt, unsigned pt_index){
  int max_depth = -1;
  if(mandible_){
    int depth_mandible = mandible_->prerefine_target_sub_block(sub_block_pt, pt_index);
    if(depth_mandible>max_depth)
      max_depth = depth_mandible;
  }
  if(cranium_){
    int depth_cranium = cranium_->prerefine_target_sub_block(sub_block_pt, pt_index);
    if(depth_cranium>max_depth)
      max_depth = depth_cranium;
  }
  if(skin_){
    int depth_skin = skin_->prerefine_target_sub_block(sub_block_pt, pt_index);
    if(depth_skin>max_depth)
      max_depth = depth_skin;
  }
  return max_depth;
}

void boxm2_vecf_composite_face_scene::compute_target_box( vcl_string const& pc_path){
  vcl_ifstream istr(pc_path.c_str());
  if(!istr){
    vcl_cout << "FATAL - can't open point cloud path " << pc_path << '\n';
    return;
  }
  vgl_pointset_3d<double> ptset;
  istr >> ptset;
  target_box_ = vgl_bounding_box(ptset);
}

boxm2_scene_sptr boxm2_vecf_composite_face_scene::
construct_target_scene(vcl_string const& scene_dir,vcl_string const& scene_name, vcl_string const& data_path,
                      double sub_block_len, bool save_scene_xml){
  if(target_box_.is_empty())
    return 0;
  vcl_vector<vcl_string> prefixes;
  prefixes.push_back("boxm2_mog3_grey");
  prefixes.push_back("boxm2_num_obs");
  boxm2_scene_sptr tscene = new boxm2_scene(scene_dir, scene_name, data_path, prefixes, target_box_, sub_block_len);
  if(save_scene_xml)
    tscene->save_scene();
  return tscene;
}

void boxm2_vecf_composite_face_scene::extract_target_cell_centers(){
  vgl_box_3d<double> target_bb = target_blk_->bounding_box_global();
  target_cell_centers_ = target_blk_->cells_in_box(target_bb);
  const vgl_h_matrix_3d<double>& Ainv = params_.trans_.get_inverse();
  for(vcl_vector<cell_info>::iterator cit =target_cell_centers_.begin();
      cit!=target_cell_centers_.end(); ++cit){
    const vgl_point_3d<double>& c =     cit->cell_center_;
    cit->cell_center_ = Ainv*c;
  }
}


void boxm2_vecf_composite_face_scene::apply_vector_field_to_target(vcl_vector<vgl_vector_3d<double> > const& vf, vcl_vector<vcl_string> const& type){
  boxm2_data_traits<BOXM2_ALPHA>::datatype alpha = 0.0f;
  boxm2_data_traits<BOXM2_MOG3_GREY>::datatype app;
  unsigned n_valid = 0;
  unsigned n = static_cast<unsigned>(vf.size());
  bool show_mouth = params_.mouth_params_.show_mouth_region_;
  for(unsigned j = 0; j<n; ++j){
    const vcl_string& t = type[j];
    bool fail = true;
    if(t == "mouth"){
      app[0]= static_cast<unsigned char>(0.0);
      unsigned tindx = target_cell_centers_[j].data_index_;
      alpha = 0.0f;//default to no occlusion
      if(show_mouth)
        alpha = 10.0f;
      target_alpha_data_[tindx] = alpha;
      target_app_data_[tindx]=app;
      continue;
    }else if(t == "mandible"){
      fail = !mandible_->apply_vector_field(target_cell_centers_[j], vf[j]);
    }else if(t == "cranium"){
      fail = !cranium_->apply_vector_field(target_cell_centers_[j], vf[j]);
    }else if(t == "skin"){
      fail = !skin_->apply_vector_field(target_cell_centers_[j], vf[j]);
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

