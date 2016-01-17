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
void bvgl_spline_region_3d<Type>::plane_to_world(Type u, Type v, vgl_point_3d<Type>& p3d) const{
  Type x = u*u_vec_.x() + origin_.x();
  Type y = v*v_vec_.y() + origin_.y();
  Type z = -(plane_.a()*x + plane_.b()*y + plane_.d())/plane_.c();
  p3d.set(x, y, z);
}
template <class Type>
bool bvgl_spline_region_3d<Type>::world_to_plane(vgl_point_3d<Type> p3d, Type& u, Type& v) const{
  u = Type(0); v = Type(0);
  vgl_point_3d<Type> cp = vgl_closest_point(plane_, p3d);
  Type len = (p3d-cp).length();
  if(len>tolerance_)
    return false;
  vgl_vector_3d<Type> del = p3d-origin_;
  u = dot_product(del, u_vec_);
  v = dot_product(del, v_vec_);
  return true;
}

template <class Type>
bvgl_spline_region_3d<Type>::bvgl_spline_region_3d(vcl_vector<vgl_point_3d<Type> > const& knots, Type tolerance):
  tolerance_(tolerance){
  if(knots.size()<3){
          vcl_cout << "FATAL - two few points to construct spline region\n";
          return;
  }
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
  vgl_point_3d<Type> cp = vgl_closest_point(plane_, knots[0]);
  origin_.set(cp.x(), cp.y(), cp.z());
  plane_.plane_coord_vectors(u_vec_, v_vec_);

  vcl_vector<vgl_point_3d<Type> > planar_knots;
  for(vcl_vector<vgl_point_3d<Type> >::const_iterator pit = knots.begin();
      pit != knots.end(); ++pit){
    vgl_point_3d<Type> cp = vgl_closest_point(plane_, *pit);
    planar_knots.push_back(cp);
  }
  spline_3d_ = vgl_cubic_spline_3d<Type>(planar_knots, Type(0.5), true);
  // normally the sense of the plane vectors doesn't matter
  // as they are used in both projecting to and from the plane
  // it is convenient to have them in the positive half space
  if(u_vec_.x()< Type(0))
    u_vec_ = -u_vec_;
  if(v_vec_.y()< Type(0))
    v_vec_ = -v_vec_;
  vgl_vector_3d<Type> newn = cross_product(u_vec_,v_vec_);
  Type len = newn.length();
  newn/=len;
  unit_normal_ = newn;
  unit_normal_/=length(newn);
  plane_ = vgl_plane_3d<Type>(unit_normal_,origin_);

  // planar polygon approximation to the spline
  poly_2d_.new_sheet();
  for(Type t = Type(0); t<=spline_3d_.max_t(); t+=tolerance_){
    vgl_point_3d<Type> p3d = spline_3d_(t);
    Type u, v;
    if(!this->world_to_plane(p3d, u, v))
      continue;
    vgl_point_2d<Type> p2d(u, v);
    poly_2d_.push_back(p2d);
  }
}
template <class Type>
bvgl_spline_region_3d<Type>::bvgl_spline_region_3d(vgl_pointset_3d<Type> const& ptset, Type tolerance){
  vcl_vector<vgl_point_3d<Type> > knots = ptset.points();
  *this = bvgl_spline_region_3d<Type>(knots, tolerance);
}

template <class Type>
bvgl_spline_region_3d<Type>::bvgl_spline_region_3d(vcl_vector<vgl_point_2d<Type> > const& knots_2d, vgl_vector_3d<Type> const& normal,
                                                   vgl_point_3d<Type> const& origin, Type tolerance):tolerance_(tolerance), origin_(origin){
  bool closed = true;
  spline_2d_.set_knots(knots_2d, closed);
  spline_2d_.set_s(Type(0.5));
  Type len = normal.length();
  unit_normal_ = normal/len;
  plane_ = vgl_plane_3d<Type>(unit_normal_, origin_);
  // initially assume that the normal should point into the positive coordinate half space
  // the user can flip the diretions later if desired.
  vgl_point_3d<Type> p_pos = origin_ + vgl_vector_3d<Type>(1.0e15,1.0e15, 1.0e15);
  this->set_point_positive(p_pos);
  plane_.plane_coord_vectors(u_vec_, v_vec_);
  // normally the sense of the plane vectors doesn't matter
  // as they are used in both projecting to and from the plane
  if(u_vec_.x()< Type(0))
    u_vec_ = -u_vec_;
  if(v_vec_.y()< Type(0))
    v_vec_ = -v_vec_;
  // construct 2-d poly approximation
  poly_2d_.new_sheet();
  for(Type t = Type(0); t<=spline_2d_.max_t(); t+=tolerance_)
    poly_2d_.push_back(vgl_point_2d<Type>(spline_2d_(t)));

  vcl_vector<vgl_point_3d<double> > knots_3d;
  for(vcl_vector<vgl_point_2d<Type> >::const_iterator kit = knots_2d.begin();
      kit != knots_2d.end(); ++kit){
    vgl_point_3d<Type> p3d;
    this->plane_to_world(kit->x(), kit->y(), p3d);
    knots_3d.push_back(p3d);
  }
  spline_3d_.set_knots(knots_3d, closed);
  spline_3d_.set_s(Type(0.5));
}
template <class Type>
void bvgl_spline_region_3d<Type>::set_point_positive(vgl_point_3d<Type> const& p_pos){
  vgl_point_3d<Type> cp = vgl_closest_point(plane_, p_pos);
  vgl_vector_3d<Type> v = p_pos-cp;
  Type dp = dot_product(v, unit_normal_);
  if(dp<0){
    plane_.set(-plane_.a(), -plane_.b(), -plane_.c(), -plane_.d());
    unit_normal_ = -unit_normal_;
    u_vec_ = -u_vec_;
    v_vec_ = -v_vec_;
  }
}

template <class Type>
bool bvgl_spline_region_3d<Type>::in(vgl_point_3d<Type> const& p3d) const{
  Type u, v;
  if(!world_to_plane(p3d, u, v))
    return false;

  vgl_point_2d<Type> p2d(u, v);
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
vgl_point_3d<Type> bvgl_spline_region_3d<Type>::operator () (Type t) const{
  vgl_point_2d<Type> p2d = spline_2d_(t);
  vgl_point_3d<Type> ret;
  this->plane_to_world(p2d.x(), p2d.y(), ret);
  return ret;
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
bvgl_spline_region_3d<Type> bvgl_spline_region_3d<Type>::scale(Type s, vgl_vector_3d<Type>const& v) const{
  vgl_point_3d<double> c = this->centroid();
  vcl_vector<vgl_point_3d<Type> > pts = spline_3d_.knots(), scaled_pts;
  for(vcl_vector<vgl_point_3d<Type> >::iterator kit = pts.begin();
      kit != pts.end(); ++kit){
    vgl_vector_3d<Type> vs = ((*kit)-c)*s;
    vgl_point_3d<Type> ps = c+vs;
    // now translate
    scaled_pts.push_back(ps + v);
  }    
  return bvgl_spline_region_3d<Type>(scaled_pts, tolerance_);
}

template <class Type>       
vgl_box_2d<Type> bvgl_spline_region_3d<Type>::bounding_box_2d() const{
  vgl_box_2d<Type> bb;
  vcl_vector<vgl_point_2d<Type> > verts = poly_2d_[0];
  unsigned n = static_cast<unsigned>(verts.size());
  vcl_vector<vgl_point_2d<Type> > pts;
  for(unsigned i = 0; i<n; ++i){
    const vgl_point_2d<Type> &p_i = verts[i];
    bb.add(p_i);
  }
  return bb;
}

template <class Type>       
vgl_box_3d<Type> bvgl_spline_region_3d<Type>::bounding_box_3d() const{
  vgl_box_2d<Type> bb = this->bounding_box_2d();
  vgl_point_2d<Type> pmin = bb.min_point(), pmax = bb.max_point();
  vgl_point_3d<Type> pmin_3d, pmax_3d;
  this->plane_to_world(pmin.x(), pmin.y(), pmin_3d);
  this->plane_to_world(pmax.x(), pmax.y(), pmax_3d);
  vgl_box_3d<Type> ret;
  ret.add(pmin_3d); ret.add(pmax_3d);
  return ret;
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
    vgl_point_3d<Type> p3d;
    this->plane_to_world(pit->x(), pit->y(), p3d);
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
