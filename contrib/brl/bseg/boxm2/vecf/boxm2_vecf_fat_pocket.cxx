#include "boxm2_vecf_fat_pocket.h"
#include <vcl_cstdlib.h>

boxm2_vecf_fat_pocket::boxm2_vecf_fat_pocket(vcl_string const& geometry_file){
}
boxm2_vecf_fat_pocket::boxm2_vecf_fat_pocket(vcl_vector<vgl_point_2d<double> > const& knots, vgl_vector_3d<double> const& normal, vgl_point_3d<double> const& origin):spline_plane_(bvgl_spline_region_3d<double>(knots, normal, origin, 0.5)){
}


vgl_box_3d<double> boxm2_vecf_fat_pocket::nominal_bounding_box(double norm_distance) const{
  vgl_box_3d<double> bb = spline_plane_.bounding_box_3d();
  vgl_point_3d<double> c = spline_plane_.centroid();
  vgl_vector_3d<double> n = spline_plane_.normal();
  vgl_point_3d<double> p_normal = c+(norm_distance*n);
  bb.add(p_normal);
  return bb;
}

vgl_pointset_3d<double> boxm2_vecf_fat_pocket::extract_pocket_pts(vgl_pointset_3d<double> const& ptset,
                                                                  double norm_distance) const{
  vgl_pointset_3d<double> ret;
  vgl_box_3d<double> nbb = this->nominal_bounding_box(norm_distance);
  unsigned npts = ptset.npts();
  for(unsigned i=0; i<npts; ++i){
    const vgl_point_3d<double>& p = ptset.p(i);
    if(!nbb.contains(p))
      continue;
    if(ptset.has_normals())
      ret.add_point_with_normal(p, ptset.n(i));
    else
      ret.add_point(p);
  }
  return ret;
}
