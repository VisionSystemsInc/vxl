// This is brl/bbas/bvgl/bvgl_spline_region_3d.h
#ifndef bvgl_spline_region_3d_h_
#define bvgl_spline_region_3d_h_
//:
// \file
// \brief A 3-d plane surface bounded by a cubic spline
// \author December 20, 2015 J.L. Mundy
//

#include <vgl/vgl_cubic_spline_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_point_3d.h>
#include <vcl_vector.h>
#include <vcl_iosfwd.h>

template <class Type>
class bvgl_spline_region_3d
{
 public:

  //: Default constructor (creates empty spline_region_3d)
 bvgl_spline_region_3d():tolerance_(Type(0.5)){};

  //: Construct using spline knots (must be closed curve)
  bvgl_spline_region_3d(vcl_vector<vgl_point_3d<Type> > const& knots);

  //: Construct using spline knots as a pointset
  bvgl_spline_region_3d(vgl_pointset_3d<Type> const& ptset);

  //: set point positve, the plane is oriented so the specified point has a positive distance.
  void set_point_positive(vgl_point_3d<Type> const& p_pos);

  //: is a point inside the region (on the plane and within or on the spline boundary
  bool in(vgl_point_3d<Type> const& p3d) const;

  //: signed distance. If the closest planar is ::in return true, otherwise false
  bool signed_distance(vgl_point_3d<Type> const& p, Type& dist) const;
  //: centroid of the region
  vgl_point_3d<Type> centroid() const;
  vcl_vector<vgl_point_3d<Type> > knots() const {return spline_3d_.knots();}
  
  // for debug purposes
  // generate a random poinset drawn from the region
  vgl_pointset_3d<Type> random_pointset(unsigned n_pts) const;
 private:
  Type tolerance_;
  vgl_polygon<Type> poly_2d_;  // to test inside
  vgl_cubic_spline_3d<Type> spline_3d_;
  vgl_plane_3d<Type> plane_;
  // contained in plane but for efficiency cache unit vector
  vgl_vector_3d<Type> unit_normal_; 
};

template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, bvgl_spline_region_3d<Type> const& p);

//: Read box from stream
template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  bvgl_spline_region_3d<Type>& p);

#define BVGL_SPLINE_REGION_3D(T) extern "please include bbas/bvgl/bvgl_spline_region_3d.txx first"

#endif // bvgl_spline_region_3d_h_
