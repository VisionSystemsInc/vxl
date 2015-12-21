// This is brl/bbas/bvgl/bvgl_spline_region_3d.txx
#ifndef bvgl_spline_region_3d_txx_
#define bvgl_spline_region_3d_txx_
#include <vcl_cstdlib.h>
#include "bvgl_spline_region_3d.h"
#include <vgl/algo/vgl_fit_plane_3d.h>
#include <vgl/vgl_closest_point.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_box_2d.h>
#include <vcl_cmath.h>
#include <vcl_limits.h>
template <class Type>
bvgl_spline_region_3d<Type>::bvgl_spline_region_3d(vcl_vector<vgl_point_3d<Type> > const& knots):
  tolerance_(Type(0.5)){
  vgl_fit_plane_3d<Type> fitter;
  fitter.clear();
  Type limit = static_cast<Type>(1);
  for(vcl_vector<vgl_point_3d<Type> >::const_iterator pit = knots.begin();
      pit != knots.end(); ++pit)
    fitter.add_point(pit->x(), pit->y(), pit->z());
  double temp = fitter.fit(&vcl_cout);
  bool success =  vcl_sqrt(temp)< limit;
  if(!success){
    vcl_cout << "Fitting error is " << vcl_sqrt(temp) << " exceeds the limit " << limit << '\n';
    vcl_cout << " Could not fit knots to a plane -- FATAL! \n";
    return;
  }
  plane_ = vgl_plane_3d<Type>(fitter.get_plane());
  unit_normal_ = plane_.normal();
  vcl_vector<vgl_point_3d<Type> > planar_knots;
  for(vcl_vector<vgl_point_3d<Type> >::const_iterator pit = knots.begin();
      pit != knots.end(); ++pit){
    vgl_point_3d<Type> cp = vgl_closest_point(plane_, *pit);
    planar_knots.push_back(cp);
  }
  spline_3d_ = vgl_cubic_spline_3d<Type>(planar_knots, Type(0.5), true);

  // planar polygon approximation to the spline
  poly_2d_.new_sheet();
  for(Type t = Type(0); t<=spline_3d_.max_t(); t+=tolerance_){
    vgl_point_3d<Type> p3d = spline_3d_(t);
    vgl_point_2d<Type> p2d;
    if(!plane_.plane_coords(p3d, p2d, tolerance_*Type(0.1)))
      continue;
    poly_2d_.push_back(p2d);
  }
}
template <class Type>
bvgl_spline_region_3d<Type>::bvgl_spline_region_3d(vgl_pointset_3d<Type> const& ptset){
  vcl_vector<vgl_point_3d<Type> > knots = ptset.points();
  *this = bvgl_spline_region_3d<Type>(knots);
}


template <class Type>
void bvgl_spline_region_3d<Type>::set_point_positive(vgl_point_3d<Type> const& p_pos){
  vgl_point_3d<Type> cp = vgl_closest_point(plane_, p_pos);
  vgl_vector_3d<Type> v = p_pos-cp;
  Type dp = dot_product(v, unit_normal_);
  if(dp<0){
    plane_.set(-plane_.a(), -plane_.b(), -plane_.c(), -plane_.d());
    unit_normal_ = -unit_normal_;
  }
}

template <class Type>
bool bvgl_spline_region_3d<Type>::in(vgl_point_3d<Type> const& p3d) const{
  vgl_point_2d<Type> p2d;
  if(!plane_.plane_coords(p3d, p2d, tolerance_*Type(0.5)))
    return false;
  return poly_2d_.contains(p2d);
}    
template <class Type>
bool bvgl_spline_region_3d<Type>::signed_distance(vgl_point_3d<Type> const& p, Type& dist) const{
  vgl_point_3d<Type> cp = vgl_closest_point(plane_, p);
  if(!in(cp)){
    dist = vcl_numeric_limits<Type>::max();
    return false;
  }
  vgl_vector_3d<Type> v = p-cp;
  dist = dot_product(v, unit_normal_);
  return true;
}

template <class Type>       
vgl_point_3d<Type> bvgl_spline_region_3d<Type>::centroid() const{
  Type cx=Type(0), cy=Type(0), cz=Type(0);
  vcl_vector<vgl_point_3d<Type> > pts = spline_3d_.knots();
  if(pts.size()==0)
    return vgl_point_3d<Type>(cx, cy, cz);
  double n = Type(0);
  for(vcl_vector<vgl_point_3d<Type> >::iterator pit = pts.begin();
      pit != pts.end(); ++pit, n+=Type(1)){
    cx += pit->x(); cy += pit->y(); cz += pit->z();
  }
  cx /= n; cy /= n; cz /= n;
  return vgl_point_3d<Type>(cx, cy, cz);
}

template <class Type>       
vgl_pointset_3d<Type> bvgl_spline_region_3d<Type>::random_pointset(unsigned n_pts) const{
  // get a bounding box for the planar polygon
  vgl_box_2d<Type> bb;
  vcl_vector<vgl_point_2d<Type> > verts = poly_2d_[0];
  unsigned n = static_cast<unsigned>(verts.size());
  vcl_vector<vgl_point_2d<Type> > pts;
  for(unsigned i = 0; i<n; ++i){
    const vgl_point_2d<Type> &p_i = verts[i];
    pts.push_back(p_i); // poly pts are part of the set
    bb.add(p_i);
  }
  unsigned n_req = n_pts - n, niter = 0;
  Type xmin = bb.min_x(), xmax = bb.max_x();
  Type ymin = bb.min_y(), ymax = bb.max_y();
  while(n_req>0 && niter < 100*n_pts){
    Type x = (xmax-xmin)*(static_cast<Type>(vcl_rand())/static_cast<Type>(RAND_MAX)) + xmin;
    Type y = (ymax-ymin)*(static_cast<Type>(vcl_rand())/static_cast<Type>(RAND_MAX)) + ymin;
    if(poly_2d_.contains(x, y)){
      pts.push_back(vgl_point_2d<Type>(x,y));
      n_req--;
    }else niter++;
  }
  if(n_req !=0)
    vcl_cout << "Warning! Insufficient number of points " << pts.size() << " instead of " << n_pts << '\n';
  //map the 2-d points back to the 3-d space
  vgl_pointset_3d<Type> ret;
  for(vcl_vector<vgl_point_2d<Type> >::iterator pit = pts.begin();
      pit != pts.end(); ++pit){
    vgl_point_3d<Type> p3d = plane_.world_coords(*pit);
    ret.add_point_with_normal(p3d, unit_normal_);
  }
  return ret;
}
#undef BVGL_SPLINE_REGION_3D_INSTANTIATE
#define BVGL_SPLINE_REGION_3D_INSTANTIATE(Type) \
template class bvgl_spline_region_3d<Type >;\
template vcl_ostream& operator<<(vcl_ostream&, bvgl_spline_region_3d<Type > const& p);\
template vcl_istream& operator>>(vcl_istream&, bvgl_spline_region_3d<Type >& p)

#endif // bvgl_spline_region_3d_txx_
