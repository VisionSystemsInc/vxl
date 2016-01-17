# brl/bbas/bvgl/tests/CMakeLists.txt

SET(bvgl_test_sources
  test_driver.cxx
  test_bvgl_changes.cxx
  test_bvgl_volume_of_intersection.cxx
  test_bvgl_ray_pyramid.cxx
  test_bvgl_intersection.cxx
  test_gen_cylinder.cxx
  test_grid_index_3d.cxx
  test_spline_region.cxx
  test_poly_region.cxx
  test_scaled_shape.cxx
)

#Make sure expat library is found
INCLUDE( ${BRL_MODULE_PATH}/FindEXPAT.cmake )
IF(EXPAT_FOUND)
  SET(bvgl_test_sources ${bvgl_test_sources} test_bvgl_labelme_parser.cxx)
ENDIF(EXPAT_FOUND)

ADD_EXECUTABLE(bvgl_test_all ${bvgl_test_sources})
TARGET_LINK_LIBRARIES(bvgl_test_all bvgl vgl vnl testlib)

ADD_TEST(bvgl_test_changes                ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_bvgl_changes)
ADD_TEST(bvgl_test_volume_of_intersection ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_bvgl_volume_of_intersection)
ADD_TEST(bvgl_test_ray_pyramid            ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_bvgl_ray_pyramid)
ADD_TEST(bvgl_test_intersection           ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_bvgl_intersection)
ADD_TEST(bvgl_test_gen_cylinder           ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_gen_cylinder)	
ADD_TEST(bvgl_test_grid_index_3d          ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_grid_index_3d)	
ADD_TEST(bvgl_test_spline_region          ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_spline_region)
ADD_TEST(bvgl_test_poly_region            ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_poly_region)
ADD_TEST(bvgl_test_scaled_shape           ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_scaled_shape)             

IF(EXPAT_FOUND)
  ADD_TEST(bvgl_test_labelme_parser       ${EXECUTABLE_OUTPUT_PATH}/bvgl_test_all test_bvgl_labelme_parser)
ENDIF(EXPAT_FOUND)

ADD_EXECUTABLE( bvgl_test_include test_include.cxx )
TARGET_LINK_LIBRARIES( bvgl_test_include bvgl )
ADD_EXECUTABLE( bvgl_test_template_include test_template_include.cxx )
TARGET_LINK_LIBRARIES( bvgl_test_template_include bvgl )
