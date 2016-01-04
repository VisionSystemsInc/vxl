#include <vcl_fstream.h>
#include <vcl_string.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include "../boxm2_vecf_fit_face.h"
int main(int argc, char ** argv)
{
  
  vul_arg_info_list arglist;
  vul_arg<vcl_string> base_dir_path(arglist, "-sdir", "Base directory for subjects", "");
  vul_arg<vcl_string> idstr(arglist, "-sid", " subject id string", "");

  arglist.parse(argc, argv, false);

  vcl_string base_dir = base_dir_path();

  if(base_dir == ""){
    vcl_cout << "Must have a base directory - fatal!\n";
    return -1;
  }
  vcl_string id = idstr();
  if(id == ""){
    vcl_cout << "Must have an id string - fatal!\n";
    return -1;
  }
  //  sub-directory named by id
  vcl_string p_dir =  base_dir + id + "/";

  vcl_string anchor_path = p_dir + id + "_face_anchors.txt";

  boxm2_vecf_fit_face fs;
  bool good = true;

  good = fs.read_anchor_file(anchor_path);
  if(!good){
    vcl_cout << "failed to read left anchor file\n";
    return -1;
  }
  good = fs.compute_auxillary_points();
  if(!good){
    vcl_cout << "failed to compute auxillary points\n";
    return -1;
  } 

  good = fs.set_trans();
  if(!good){
        vcl_cout << "failed to compute face transform\n";
    return -1;
  } 
  vcl_string param_path = base_dir + id + "/" + id + "_face_params.txt";
  vcl_ofstream ostr(param_path.c_str());
  if(ostr){
    ostr << fs.params();
    ostr.close();
  }else
    vcl_cout << "WARNING - not able to open param path " << param_path << '\n' << vcl_flush;
  
  vcl_string source_path = base_dir + "skull/skin_orig_appearance_sampled-r10-s1.05-t10.txt";
  vcl_string target_path = base_dir + id + "/" + id + "_trans_face.txt";
  good = fs.transform_face(source_path, target_path);
  if(good)
    return 0;
  return -1;
}

 
