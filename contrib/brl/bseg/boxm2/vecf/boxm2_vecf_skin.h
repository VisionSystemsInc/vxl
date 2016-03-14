#ifndef boxm2_vecf_skin_h_
#define boxm2_vecf_skin_h_
//:
// \file
// \brief  The skin component of the head
//
// \author J.L. Mundy
// \date   4 Dec 2015
//
#include <vector>
#include <string>
#include <vcl_iostream.h>
#include <vgl/vgl_pointset_3d.h>
#include <vgl/vgl_box_3d.h>
#include <bvgl/bvgl_grid_index_3d.h>
class boxm2_vecf_skin{
 public:
 boxm2_vecf_skin(): nbins_(25), has_appearance_(false){}
 boxm2_vecf_skin(std::string const& geometry_file, unsigned nbins = 25);

 boxm2_vecf_skin(vgl_pointset_3d<double> const& ptset,unsigned nbins = 25): ptset_(ptset), nbins_(nbins), has_appearance_(false){}

 void read_skin(vcl_istream& istr, unsigned comma_count=2);

 vgl_box_3d<double> bounding_box() const {return index_.bounding_box();}
 double surface_distance(vgl_point_3d<double> const& p) const {return index_.surface_distance(p);}
 double surface_distance(vgl_point_3d<double> const& p, double& apc) const {return index_.surface_distance(p, apc);}
 //: the functor operator for surface distance. dist_thresh is the distance a closest point on the normal plane
 // can be away from the closest point in the cross-section pointset.
 double operator() (vgl_point_3d<double> p) const{ return index_.surface_distance(p);}
 bool has_appearance() const { return index_.has_scalars();}

 //:for debug purposes
 void display_vrml(vcl_ofstream& ostr) const;

 private:
 unsigned nbins_;
 bool has_appearance_;
 bvgl_grid_index_3d index_;
 vgl_pointset_3d<double> ptset_;
 std::vector<double> appearance_;
};
vcl_ostream&  operator << (vcl_ostream& s, boxm2_vecf_skin const& pr);
vcl_istream&  operator >> (vcl_istream& s, boxm2_vecf_skin& pr);

#endif// boxm2_vecf_skin
