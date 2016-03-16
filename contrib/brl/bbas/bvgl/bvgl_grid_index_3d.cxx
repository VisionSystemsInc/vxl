#include "bvgl_grid_index_3d.h"
#include <vgl/vgl_bounding_box.h>
#include <vgl/vgl_closest_point.h>
#include <vgl/vgl_plane_3d.h>

#include <vcl_compiler.h>
#include <iostream>


void
bvgl_grid_index_3d
::index(vgl_point_3d<double> const& p, unsigned& ix, unsigned& iy, unsigned& iz) const
{
  ix = static_cast<unsigned>(p.x()/delta_x_);
  iy = static_cast<unsigned>(p.y()/delta_y_);
  iz = static_cast<unsigned>(p.z()/delta_z_);
  if(ix>=nx_)
    ix = nx_-1;
  if(iy>=ny_)
    iy = ny_-1;
  if(iz>=nz_)
    iz = nz_-1;
}

bvgl_grid_index_3d::bvgl_grid_index_3d(unsigned nx, unsigned ny, unsigned nz, vgl_pointset_3d<double> ptset, double thresh):
  nx_(nx), ny_(ny), nz_(nz), has_normals_(ptset.has_normals()), npts_(0), thresh_(thresh), has_scalars_(false){
  p_grid_ = vbl_array_3d<std::vector<vgl_point_3d<double> > >(nx, ny, nz);
  if(has_normals_)
    n_grid_ = vbl_array_3d<std::vector<vgl_vector_3d<double> > >(nx, ny, nz);
  bbox_ = vgl_bounding_box(ptset);
  xmin_ = bbox_.min_x();
  ymin_ = bbox_.min_y();
  zmin_ = bbox_.min_z();
  double dnx = static_cast<double>(nx), dny = static_cast<double>(ny), dnz = static_cast<double>(nz);
  delta_x_ = bbox_.width()/dnx;
  delta_y_ = bbox_.height()/dny;
  delta_z_ = bbox_.depth()/dnz;
  unsigned ix =0, iy=0, iz =0;
  unsigned npts = ptset.npts();
  for (unsigned int i = 0; i<npts; ++i)
    {
    vgl_point_3d<double> pi = ptset.p(i);
    this->index(pi, ix, iy, iz);
    p_grid_[ix][iy][iz].push_back(pi);
    npts_++;
    if(has_normals_)
      n_grid_[ix][iy][iz].push_back(ptset.n(i));
    }
}
bvgl_grid_index_3d::bvgl_grid_index_3d(unsigned nx, unsigned ny, unsigned nz,
                                       vgl_pointset_3d<double> ptset, std::vector<double> scalars, double thresh):
  nx_(nx), ny_(ny), nz_(nz), has_normals_(ptset.has_normals()), npts_(0), thresh_(thresh), has_scalars_(true){
  p_grid_ = vbl_array_3d<std::vector<vgl_point_3d<double> > >(nx, ny, nz);
  s_grid_ = vbl_array_3d<std::vector<double> >(nx, ny, nz);
  if(has_normals_)
    n_grid_ = vbl_array_3d<std::vector<vgl_vector_3d<double> > >(nx, ny, nz);
  bbox_ = vgl_bounding_box(ptset);
  xmin_ = bbox_.min_x();
  ymin_ = bbox_.min_y();
  zmin_ = bbox_.min_z();
  double dnx = static_cast<double>(nx), dny = static_cast<double>(ny), dnz = static_cast<double>(nz);
  delta_x_ = bbox_.width()/dnx;
  delta_y_ = bbox_.height()/dny;
  delta_z_ = bbox_.depth()/dnz;
  unsigned ix =0, iy=0, iz =0;
  unsigned npts = ptset.npts();
  for(unsigned i = 0; i<npts; ++i){
    vgl_point_3d<double> pi = ptset.p(i);
    this->index(pi, ix, iy, iz);
    p_grid_[ix][iy][iz].push_back(pi);
    npts_++;
    if(has_normals_)
      n_grid_[ix][iy][iz].push_back(ptset.n(i));
    s_grid_[ix][iy][iz].push_back(scalars[i]);
  }
}

//: the point contained in the grid closest to p or optionally to the normal plane passing through the closest point
bool bvgl_grid_index_3d::closest_point(vgl_point_3d<double> const& p, vgl_point_3d<double>& pc, double& scalar) const{
  scalar = 0.0;
  if(!npts_)
    return false;
  unsigned ix =0, iy=0, iz =0;
  this->index(p, ix, iy, iz);
  unsigned ixs = ix, ixe = ix;
  unsigned iys = iy, iye = iy;
  unsigned izs = iz, ize = iz;
  if(ix>=1) ixs = ix-1;
  if(iy>=1) iys = iy-1;
  if(iz>=1) izs = iz-1;
  if(ix<(nx_-1)) ixe = ix+1;
  if(iy<(ny_-1)) iye = iy+1;
  if(iz<(nz_-1)) ize = iz+1;
  double dmin = std::numeric_limits<double>::max();
  unsigned ixc = 0, iyc = 0,  izc = 0, ipc = 0;
  bool found = false;
  for(unsigned i = ixs; i<=ixe; ++i)
    for(unsigned j = iys; j<=iye; ++j)
      for(unsigned k = izs; k<=ize; ++k){
        unsigned np = static_cast<unsigned>(p_grid_[i][j][k].size());
        if(!np) continue;
        for(unsigned ip = 0; ip<np; ++ip){
          vgl_point_3d<double> pp = p_grid_[i][j][k][ip];
          double d = (pp-p).length();
          if(d<dmin){
            dmin = d; ixc = i; iyc = j; izc = k; ipc = ip; found = true;
          }
        }
      }
  if(!found)
    return false;
  vgl_point_3d<double> pmin = p_grid_[ixc][iyc][izc][ipc];
  if(!has_normals_){
    pc = pmin;
    if(this->has_scalars())
    scalar = s_grid_[ixc][iyc][izc][ipc];
    return true;
  }
  vgl_vector_3d<double> nc = n_grid_[ixc][iyc][izc][ipc];
  vgl_plane_3d<double> pl(nc, pmin);
  pc = vgl_closest_point(pl, p);
  if((pc-pmin).length()>thresh_)
    pc = pmin;
  if(this->has_scalars())
    scalar = s_grid_[ixc][iyc][izc][ipc];
  return true;
}
//: the distance from p to the closest point or optionally its normal plane
bool bvgl_grid_index_3d::closest_point(vgl_point_3d<double> const& p, vgl_point_3d<double>& pc) const{
  double scalar = 0.0;
  return this->closest_point(p, pc, scalar);
}
double bvgl_grid_index_3d::distance(vgl_point_3d<double> const& p, double& scalar) const{
  vgl_point_3d<double> pc;
  bool good = this->closest_point(p, pc, scalar);
  if(!good)
    return std::numeric_limits<double>::max();
  return (p-pc).length();
}
//: the distance from p to the closest point or optionally its normal plane
double bvgl_grid_index_3d::distance(vgl_point_3d<double> const& p) const{
  double scalar = 0.0;
  return this->distance(p, scalar);
}

