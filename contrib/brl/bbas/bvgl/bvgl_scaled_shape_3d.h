// This is brl/bbas/bvgl/bvgl_scaled_shape_3d.h
#ifndef bvgl_scaled_shape_3d_h_
#define bvgl_scaled_shape_3d_h_
//:
// \file
// \brief A 3-d plane surface bounded by a cubic spline lying in the plane
// \author December 20, 2015 J.L. Mundy
//
// this shape is a form of generalized cylinder and may be folded into the
// gen_cylinder class at a later stage.
#include "bvgl_spline_region_3d.h"
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
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
  
  //: return the base spline region
  bvgl_spline_region_3d<Type> base() const{ return base_;}

  //:bounds on u, v parameters. Currently assumes that the base is the largest cross section
  // and the axis is the base plane normal. 
  void uv_bounds(Type& u_min, Type& u_max, Type& v_min, Type& v_max) const;

  // parametric description of the volume
  bool operator () (Type u, Type v, Type w, vgl_point_3d<Type>& p) const;

  //: is a point inside the 3-d region (on the plane and within or on the spline boundary
  bool in(vgl_point_3d<Type> const& p3d) const;

  //: the closest point in the volume
  vgl_point_3d<Type> closest_point(vgl_point_3d<Type> const& p) const;

 //: centroid of the region
  vgl_point_3d<Type> centroid() const;

  //: The volume of the region
  Type volume() const;

  //: deform *this shape.
  // Two of the deformation axes, L1 and L2, lie in the base plane and L3 is along the plane normal
  // Specifying L1 determines the in plane rotation needed to align the L1 direction with the u plane axis
  bvgl_scaled_shape_3d<Type> deform(Type lambda, Type gamma, vgl_vector_3d<Type> const& L1) const;
                                    
  //: set deformation parameters
  void set_lambda(Type lambda){lambda_=lambda;}//principal eigenvalue l1
  //exponent factor for second eigenvalue, i.e. l2 = lambda_^gamma, l3 = 1/(l1*l2)
  void set_gamma(Type gamma){gamma_ = gamma;}
  void set_principal_eigenvector(vgl_vector_3d<Type> const& L1){L1_ = L1;}
  void apply_parameters_to_cross_sections();

  //: inverse vector field corresponding to the deformation
  bool inverse_vector_field(vgl_point_3d<Type> const& p, vgl_vector_3d<Type>& inv) const;

  // returns false if the point is not within tolerance of any cross section
  bool nearest_cross_section_index(vgl_point_3d<Type> const& p3d, unsigned& index) const;

  vgl_box_3d<Type> bounding_box() const;

  Type distance(vgl_point_3d<Type> const& p) const;
  
  // for debug purposes
  // generate a random poinset drawn from the region
  vgl_pointset_3d<Type> random_pointset(unsigned n_pts) const;
  vgl_pointset_3d<Type> base_random_pointset(unsigned n_pts) const{
    return base_.random_pointset(n_pts);
  }
  //:Accessors
  const vcl_vector<bvgl_spline_region_3d<Type> >& cross_sections() const {return cross_sections_;}
  void set_cross_sections(vcl_vector<bvgl_spline_region_3d<Type> >const& csects) {cross_sections_ = csects;}

  Type max_norm_distance() const {return max_norm_distance_;}
  void set_max_norm_distance(Type max_nd){max_norm_distance_ = max_nd;}

 private:
  // deformation parameters
  Type lambda_; 
  Type gamma_;  
  vgl_vector_3d<Type> L1_;
  Type max_nd_;
  // nominal attributes
  scale_type stype_;
  Type tolerance_;
  bvgl_spline_region_3d<Type> base_;
  vcl_vector<bvgl_spline_region_3d<Type> > cross_sections_;
  Type max_norm_distance_;
  Type scale_at_midpt_;
  Type scale_at_max_;
  // private functions
  void compute_cross_sections();
};

template <class Type>
vcl_ostream&  operator<<(vcl_ostream& s, bvgl_scaled_shape_3d<Type> const& p);

template <class Type>
vcl_istream&  operator>>(vcl_istream& is,  bvgl_scaled_shape_3d<Type>& p);

#define BVGL_SCALED_SHAPE_3D(T) extern "please include bbas/bvgl/bvgl_scaled_shape_3d.txx first"

#endif // bvgl_scaled_shape_3d_h_
