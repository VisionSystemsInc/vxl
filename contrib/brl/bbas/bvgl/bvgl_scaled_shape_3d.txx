// This is brl/bbas/bvgl/bvgl_scaled_shape_3d.txx
#ifndef bvgl_scaled_shape_3d_txx_
#define bvgl_scaled_shape_3d_txx_
#include <vcl_cstdlib.h>
#include "bvgl_scaled_shape_3d.h"
#include <vcl_cmath.h>
#include <vcl_limits.h>

template <class Type>
Type bvgl_scaled_shape_3d<Type>::linear_scale(Type w) const{
  // how does the scale of the base vary with the normal parameter
  // for now assume linear range of 0.1-1.0
  Type s = Type(1)+ w*(scale_at_max_-Type(1));
  if(s > Type(1))
    return Type(1);
  if(s<Type(0.1))
    return Type(0.1);
  return s;
}
template <class Type>
Type bvgl_scaled_shape_3d<Type>::quadratic_scale(Type w) const{
  // how does the scale of the base vary with the normal parameter
  // for now assume quadratic range of 0.1-1.0
  Type s1 = Type(4)*scale_at_midpt_-scale_at_max_-Type(3);
  Type s2 = Type(2)*(Type(1)+scale_at_max_)-Type(4)*scale_at_midpt_;;
  Type s = Type(1)+ w*s1 + w*w*s2;
  if(s > Type(1))
    return Type(1);
  if(s<Type(0.1))
    return Type(0.1);
  return s;
}
template <class Type>
void bvgl_scaled_shape_3d<Type>::compute_cross_sections(){
  vgl_vector_3d<Type> n = base_.normal();
  for(Type nd = Type(0); nd<=max_norm_distance_; nd += tolerance_){
    vgl_vector_3d<Type> v = nd*n;
    Type w = nd/max_norm_distance_;
    Type s = Type(1);
    if(stype_ == LINEAR)
      s = this->linear_scale(w);
    else
      s = this->quadratic_scale(w);
    cross_sections_.push_back(base_.scale(s,v));
  }
}
template <class Type>
bool bvgl_scaled_shape_3d<Type>::nearest_cross_section_index(vgl_point_3d<Type> const& p3d, unsigned& index) const{
  // assume for the time being that the base is the maximum cross section
  index = 0;
  Type dist;
  if(!base_.signed_distance(p3d, dist))
    return false;
  if(dist<-tolerance_)
    return false;
  if(dist>max_norm_distance_)
    return false;
  Type dindex = vcl_floor(dist/tolerance_);
  index = static_cast<unsigned>(dindex);
  unsigned n =static_cast<unsigned>(cross_sections_.size()); 
  if(index >= n)
    index = n-1;
  return true;
}

template <class Type>
bool bvgl_scaled_shape_3d<Type>::in(vgl_point_3d<Type> const& p3d) const{
  unsigned indx;
  if(!nearest_cross_section_index(p3d, indx))
    return false;
  return cross_sections_[indx].in(p3d);
}

template <class Type>
bool bvgl_scaled_shape_3d<Type>::signed_distance(vgl_point_3d<Type> const& p, Type& dist) const{
  dist = Type(0);
  return false;
}

template <class Type>
vgl_point_3d<Type> bvgl_scaled_shape_3d<Type>::operator () (Type u, Type v) const{
  return vgl_point_3d<Type>(Type(0), Type(0), Type(0));
}

template <class Type>
vgl_point_3d<Type> bvgl_scaled_shape_3d<Type>::centroid() const{
  Type cx=Type(0), cy=Type(0), cz=Type(0);
  return vgl_point_3d<Type>(cx, cy, cz);
}

template <class Type>       
vgl_box_3d<Type> bvgl_scaled_shape_3d<Type>::bounding_box_3d() const{
  vgl_box_3d<Type> bb = base_.bounding_box_3d();
  // assume the scale monotonically decreases so bounding box is defined by the base
  vgl_vector_3d<Type> nv = max_norm_distance_*base_.normal();
  vcl_vector<vgl_point_3d<Type> > knots = base_.knots();
  for(vcl_vector<vgl_point_3d<Type> >::iterator kit = knots.begin();
      kit != knots.end(); ++kit){
    vgl_point_3d<Type> p = (*kit) + nv;
    bb.add(p);
  }
    return bb;
}

template <class Type>       
vgl_pointset_3d<Type> bvgl_scaled_shape_3d<Type>::random_pointset(unsigned n_pts) const{
  vgl_vector_3d<Type> norm = base_.normal();
  vgl_pointset_3d<Type> ret;
  vcl_vector<vgl_point_3d<Type> > pts;
  vgl_box_3d<Type> bb = this->bounding_box_3d();
  Type xmin = bb.min_x(), xmax = bb.max_x();
  Type ymin = bb.min_y(), ymax = bb.max_y();
  Type zmin = bb.min_z(), zmax = bb.max_z();
  unsigned n_req = n_pts, niter = 0;
  while(n_req>0 && niter < 100*n_pts){
    Type x = (xmax-xmin)*(static_cast<Type>(vcl_rand())/static_cast<Type>(RAND_MAX)) + xmin;
    Type y = (ymax-ymin)*(static_cast<Type>(vcl_rand())/static_cast<Type>(RAND_MAX)) + ymin;
    Type z = (zmax-zmin)*(static_cast<Type>(vcl_rand())/static_cast<Type>(RAND_MAX)) + zmin;
    vgl_point_3d<Type> p(x, y, z);
    if(this->in(p)){
      pts.push_back(vgl_point_3d<Type>(x,y,z));
      n_req--;
    }else niter++;
  }
  if(n_req !=0)
    vcl_cout << "Warning! Insufficient number of points " << pts.size() << " instead of " << n_pts << '\n';
  //map the 2-d points back to the 3-d space

  for(vcl_vector<vgl_point_3d<Type> >::iterator pit = pts.begin();
      pit != pts.end(); ++pit){
    ret.add_point_with_normal(*pit,norm);
  }
  return ret;
}
#undef BVGL_SCALED_SHAPE_3D_INSTANTIATE
#define BVGL_SCALED_SHAPE_3D_INSTANTIATE(Type) \
template class bvgl_scaled_shape_3d<Type >;\
template vcl_ostream& operator<<(vcl_ostream&, bvgl_scaled_shape_3d<Type > const& p);\
template vcl_istream& operator>>(vcl_istream&, bvgl_scaled_shape_3d<Type >& p)

#endif // bvgl_scaled_shape_3d_txx_
