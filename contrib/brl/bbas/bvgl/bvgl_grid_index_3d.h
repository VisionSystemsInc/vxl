#ifndef bvgl_grid_index_3d_h_
#define bvgl_grid_index_3d_h_
//:
// \file
// \brief  A simple grid index to store a pointset, optionally with normals and optional scalar value
//
// \author J.L. Mundy
// \date   6 November 2015
//
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_cassert.h>
#include <vcl_vector.h>
#include <vcl_string.h>
#include <vcl_map.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_box_3d.h>
#include <vbl/vbl_array_3d.h>
class bvgl_grid_index_3d{
 public:
 bvgl_grid_index_3d(): has_scalars_(false),has_normals_(false),
    nx_(0), ny_(0), nz_(0), delta_x_(1.0), delta_y_(1.0), delta_z_(1.0), npts_(0), thresh_(1.0){}
  bvgl_grid_index_3d(unsigned nx, unsigned ny, unsigned nz, vgl_pointset_3d<double> ptset, double thresh = 1.0);
  bvgl_grid_index_3d(unsigned nx, unsigned ny, unsigned nz, vgl_pointset_3d<double> ptset, vcl_vector<double> scalars, double thresh = 1.0);

  void index(vgl_point_3d<double> const& p, unsigned& ix, unsigned& iy, unsigned& iz) const;

  //: geometric queries
  //: the point contained in the grid closest to p or optionally to the normal plane passing through the closest point
  bool closest_point(vgl_point_3d<double> const& p, vgl_point_3d<double>& pc) const;
  bool closest_point(vgl_point_3d<double> const& p, vgl_point_3d<double>& pc, double& scalar) const;
  //: the distance from p to the closest point or optionally its normal plane
  double distance(vgl_point_3d<double> const& p) const;
  double distance(vgl_point_3d<double> const& p, double& scalar) const;
  //: accessors
  vgl_box_3d<double> bounding_box() const {return bbox_;}
  unsigned npts() const {return npts_;}
  bool has_normals() const {return has_normals_;}
  bool has_scalars() const {return has_scalars_;}
 protected:
  bool has_scalars_;
  bool has_normals_;
  vbl_array_3d<vcl_vector<vgl_point_3d<double > > >   p_grid_;
  vbl_array_3d<vcl_vector<vgl_vector_3d<double > > >  n_grid_;//optional normal grid
  vbl_array_3d<vcl_vector<double> >  s_grid_;//optional scalar grid
  vgl_box_3d<double> bbox_;
  double xmin_;
  double ymin_;
  double zmin_;
  double delta_x_;
  double delta_y_;
  double delta_z_;
  unsigned nx_;
  unsigned ny_;
  unsigned nz_;
  unsigned npts_;
  double thresh_;
};
#endif// bvgl_grid_index_3d
