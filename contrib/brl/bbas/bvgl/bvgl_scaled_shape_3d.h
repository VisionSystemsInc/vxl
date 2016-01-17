// This is brl/bbas/bvgl/bvgl_scaled_shape_3d.h
#ifndef bvgl_scaled_shape_3d_h_
#define bvgl_scaled_shape_3d_h_
//:
// \file
// \brief A 3-d plane surface bounded by a cubic spline lying in the plane
// \author December 20, 2015 J.L. Mundy
//
#include "bvgl_spline_region_3d.h"
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vcl_vector.h>
#include <vcl_iosfwd.h>

template <class Type>
class bvgl_scaled_shape_3d
{
 public:
  enum scale_type {LINEAR, QUADRATIC};
  //: Default constructor (creates empty scaled_shape_3d)
 bvgl_scaled_shape_3d():tolerance_(Type(0.5)), scale_at_midpt_(Type(1)), scale_at_max_(Type(1)), max_norm_distance_(Type(0)), stype_(LINEAR){};

  //: Construct using spline knots (must be closed curve)
 bvgl_scaled_shape_3d(bvgl_spline_region_3d<Type> const& region, Type max_norm_distance, Type scale_at_max, Type tolerance):
  base_(region), max_norm_distance_(max_norm_distance), scale_at_max_(scale_at_max),scale_at_midpt_(Type(0.5)*scale_at_max_),
    tolerance_(tolerance), stype_(LINEAR){
    this->compute_cross_sections();
    vcl_cout << cross_sections_.size() << '\n';
  }
 bvgl_scaled_shape_3d(bvgl_spline_region_3d<Type> const& region, Type max_norm_distance, Type scale_at_midpt,Type scale_at_max, Type tolerance):
  base_(region), max_norm_distance_(max_norm_distance), scale_at_midpt_(scale_at_midpt), scale_at_max_(scale_at_max),
    tolerance_(tolerance), stype_(QUADRATIC){
    this->compute_cross_sections();
    vcl_cout << cross_sections_.size() << '\n';
  }
  Type linear_scale(Type w) const;
  Type quadratic_scale(Type w) const;
  
  // parametric description of the surface
  vgl_point_3d<Type> operator () (Type u, Type v) const;

    //: is a point inside the 3-d region (on the plane and within or on the spline boundary
  bool in(vgl_point_3d<Type> const& p3d) const;

  //: signed distance to the surface
  bool signed_distance(vgl_point_3d<Type> const& p, Type& dist) const;
  
  //: centroid of the region
  vgl_point_3d<Type> centroid() const;

  // returns false if the point is not within tolerance of any cross section
  bool nearest_cross_section_index(vgl_point_3d<Type> const& p3d, unsigned& index) const;

  vgl_box_3d<Type> bounding_box_3d() const;

  // for debug purposes
  // generate a random poinset drawn from the region
  vgl_pointset_3d<Type> random_pointset(unsigned n_pts) const;
  vgl_pointset_3d<Type> base_random_pointset(unsigned n_pts) const{
    return base_.random_pointset(n_pts);
  }
 private:
  scale_type stype_;
  void compute_cross_sections();
  Type tolerance_;
  bvgl_spline_region_3d<Type> base_;
  vcl_vector<bvgl_spline_region_3d<Type> > cross_sections_;
  Type max_norm_distance_;
  Type scale_at_midpt_;
  Type scale_at_max_;
};

template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, bvgl_scaled_shape_3d<Type> const& p);

template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  bvgl_scaled_shape_3d<Type>& p);

#define BVGL_SCALED_SHAPE_3D(T) extern "please include bbas/bvgl/bvgl_scaled_shape_3d.txx first"

#endif // bvgl_scaled_shape_3d_h_
