#include <testlib/testlib_register.h>

DECLARE(test_bvgl_changes);
DECLARE(test_bvgl_volume_of_intersection);
DECLARE(test_bvgl_ray_pyramid);
DECLARE(test_bvgl_intersection);
DECLARE(test_bvgl_labelme_parser);
DECLARE(test_gen_cylinder);
DECLARE(test_grid_index_3d);
DECLARE(test_spline_region);
DECLARE(test_poly_region);

void
register_tests()
{
  REGISTER(test_bvgl_changes);
  REGISTER(test_bvgl_volume_of_intersection);
  REGISTER(test_bvgl_ray_pyramid);
  REGISTER(test_bvgl_intersection);
  REGISTER(test_bvgl_labelme_parser);
  REGISTER(test_gen_cylinder);
  REGISTER(test_grid_index_3d);
  REGISTER(test_spline_region);
  REGISTER(test_poly_region);
}

DEFINE_MAIN;
