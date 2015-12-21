#include "boxm2_vecf_mouth.h"
#include <vcl_cstdlib.h>

boxm2_vecf_mouth::boxm2_vecf_mouth(vcl_vector<vgl_point_3d<double> > const& knots){
  sup_ = bvgl_spline_region_3d<double>(knots);
  inf_ = bvgl_spline_region_3d<double>(knots);
  // set the sense of the normals so that points inside the mouth
  // have positive signed distances from both region planes
  vgl_point_3d<double> cent_sup = sup_.centroid();
  vgl_point_3d<double> cent_inf = cent_sup;
  cent_sup.set(cent_sup.x(), cent_sup.y()-1.0, cent_sup.z());
  cent_inf.set(cent_inf.x(), cent_inf.y()+1.0, cent_inf.z());
  sup_.set_point_positive(cent_sup);
  inf_.set_point_positive(cent_inf);
}

boxm2_vecf_mouth::boxm2_vecf_mouth(vgl_pointset_3d<double>  const& ptset){
  *this = boxm2_vecf_mouth(ptset.points());
}

void boxm2_vecf_mouth::rotate_inf(){
  vcl_vector<vgl_point_3d<double> > knots = sup_.knots();
  // rotate the knots of the sup rather than current inf knots
  // thus an invariant starting point before rotation
  vcl_vector<vgl_point_3d<double> > rot_knots;
  for(vcl_vector<vgl_point_3d<double> >::iterator kit = knots.begin();
      kit != knots.end(); ++kit)
    rot_knots.push_back(rot_*(*kit));
  inf_ = bvgl_spline_region_3d<double>(rot_knots);
  vgl_point_3d<double> cent_sup = sup_.centroid();
  inf_.set_point_positive(cent_sup);
}

void boxm2_vecf_mouth::set_mandible_params(boxm2_vecf_mandible_params const& mand_params){
  mand_params_ = mand_params;
  vnl_vector_fixed<double,3> X(1.0, 0.0, 0.0);
  vnl_quaternion<double> Q(X,mand_params_.jaw_opening_angle_rad_);
  rot_ = vgl_rotation_3d<double>(Q);
  this->rotate_inf();
}

vgl_box_3d<double> boxm2_vecf_mouth::bounding_box() const{
  vcl_vector<vgl_point_3d<double> > sup_knots = sup_.knots();
  vcl_vector<vgl_point_3d<double> > inf_knots = inf_.knots();
  vgl_box_3d<double> ret;
  for(vcl_vector<vgl_point_3d<double> >::iterator kit = sup_knots.begin();
      kit != sup_knots.end(); ++kit)
    ret.add(*kit);
  for(vcl_vector<vgl_point_3d<double> >::iterator kit = inf_knots.begin();
      kit != inf_knots.end(); ++kit)
    ret.add(*kit);
  return ret;
}

bool boxm2_vecf_mouth::in(vgl_point_3d<double> const& pt) const{
  double ds, di;
  bool sup_sd_valid = sup_.signed_distance(pt, ds);
  bool inf_sd_valid = inf_.signed_distance(pt, di);
 
  return sup_sd_valid&&inf_sd_valid&&ds>=0.0&&di>=0.0;
}
vgl_pointset_3d<double> boxm2_vecf_mouth::random_pointset(unsigned n_pts) const{
  // add sup and inf random pointsets
  vgl_pointset_3d<double> pts = sup_.random_pointset(n_pts);
  vgl_pointset_3d<double> inf_pts = inf_.random_pointset(n_pts);
  pts.append_pointset(inf_pts);
  vgl_box_3d<double> bb = this->bounding_box();
  // generate interior points 
  unsigned n_req = 100*n_pts, niter =0;
  double xmin = bb.min_x(), xmax = bb.max_x();
  double ymin = bb.min_y(), ymax = bb.max_y();
  double zmin = bb.min_z(), zmax = bb.max_z();
  while(n_req>0 && niter < 10000*n_pts){
    double x = (xmax-xmin)*(static_cast<double>(vcl_rand())/static_cast<double>(RAND_MAX)) + xmin;
    double y = (ymax-ymin)*(static_cast<double>(vcl_rand())/static_cast<double>(RAND_MAX)) + ymin;
    double z = (zmax-zmin)*(static_cast<double>(vcl_rand())/static_cast<double>(RAND_MAX)) + zmin;
    vgl_point_3d<double> p(x, y, z);
    if(this->in(p)){
      if(p.x()>0.0)
        pts.add_point_with_normal(p, vgl_vector_3d<double>(-1.0, 0.0, 0.0));
      else
        pts.add_point_with_normal(p, vgl_vector_3d<double>(-1.0, 0.0, 0.0));
      n_req--;
    }else niter++;
  }
  if(n_req !=0)
    vcl_cout << "Warning! Insufficient number of points " << pts.npts() << " instead of " << n_pts << '\n';

  return pts;
}
