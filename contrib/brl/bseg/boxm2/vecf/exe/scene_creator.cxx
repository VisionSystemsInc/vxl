#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include "../boxm2_vecf_composite_face_scene.h"
int main(int argc, char ** argv)
{

  vul_arg_info_list arglist;
  vul_arg<vcl_string> subject_base_dir_path(arglist, "-sdir", "subject base directory", "");
  vul_arg<vcl_string> idstr(arglist, "-sid", "subject id string", "");
  vul_arg<vcl_string> avg_face_scene_paths(arglist, "-fpath", "face scene paths file path","");
  vul_arg<double>     sub_block_length(arglist, "-len", "sub block edge length", 1.0);
  arglist.parse(argc, argv, false);

  vcl_string base_dir = subject_base_dir_path();
  if(base_dir == ""){
    vcl_cout << "Must have a base directory - fatal!\n";
    return -1;
  }
  vcl_string id = idstr();
  if(id == ""){
    vcl_cout << "Must have an id string - fatal!\n";
    return -1;
  }
  vcl_string avg_face_paths = avg_face_scene_paths();
  if(avg_face_paths == ""){
    vcl_cout << "Must have a path to average face scene paths - fatal!\n";
    return -1;
  }

  //  sub-directory named by id
  vcl_string p_dir =  base_dir + id + "/";
  vcl_string pc_path = p_dir + id + "_trans_face.txt";
  vcl_string params_path = p_dir + id+ "_face_params.txt";
  vcl_string data_dir_name = "target_model";
  vcl_string data_directory = p_dir + data_dir_name;
  if(!vul_file::exists(data_directory))
          if(!vul_file::make_directory(data_directory)){
                  vcl_cout << "couldn't make directory " << data_directory << '\n';
                  return -1;
          }

  if(!vul_file::exists(pc_path)){
    vcl_cout << "path doesn't exist for point cloud data " << pc_path << '\n';
    return -1;
  }
  if(!vul_file::exists(params_path)){
    vcl_cout << "path doesn't exist for fitted subject parameters " << pc_path << '\n';
    return -1;
  }
  if(!vul_file::exists(avg_face_paths)){
    vcl_cout << "average face scene path doesn't exist  " << avg_face_paths << '\n';
    return -1;
  }

  vcl_string xml_name = id;

  boxm2_vecf_composite_face_scene cfs(avg_face_paths);

  boxm2_vecf_composite_face_params params;
  vcl_ifstream pistr(params_path.c_str());
  pistr >> params;

  cfs.set_params(params);

  cfs.compute_target_box(pc_path);

  vgl_box_3d<double> tgt_box = cfs.target_box();
  vcl_cout << "scene bounds " << tgt_box << '\n';

  double sb_len = sub_block_length();

  boxm2_scene_sptr tscene = cfs.construct_target_scene(p_dir, xml_name, data_dir_name, sb_len);
  if(!tscene){
    vcl_cout << "Coulnd construct target scene" << '\n';
    return -1;
  }
  vcl_cout << *tscene << '\n';

  return 0;
}
