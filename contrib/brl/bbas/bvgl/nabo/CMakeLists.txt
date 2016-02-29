# Kd tree and k nearest neighbor library adapted from nabo to use vnl
# nabo was written by Stephane Magnenat
# brl/bbas/bvgl/nabo/CMakeLists.txt

doxygen_add_library(contrib/brl/bbas/bvgl/nabo
  DEPENDS core/vnl
  PACKAGE contrib/brl
  )

INCLUDE_DIRECTORIES(${BRL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${BRL_INCLUDE_DIR}/bbas )
INCLUDE_DIRECTORIES(${GEL_INCLUDE_DIR})

SET(bvgl_sources
 bvgl_nabo_index_heap.h bvgl_nabo.h bvgl_nabo_private.h 
 bvgl_nabo_kdd_tree.cxx bvgl_nabo.cxx bvgl_nabo_brute_force.cxx
)
AUX_SOURCE_DIRECTORY(Templates bvgl_sources)


#Create NABO library
ADD_LIBRARY(bvgl_nabo ${bvgl_sources})
INSTALL_TARGETS(/lib bvgl_nabo)
INSTALL_NOBASE_HEADER_FILES(/include/vxl/contrib/brl/bbas/bvgl/nabo ${bvgl_sources})
TARGET_LINK_LIBRARIES(bvgl_nabo vnl)

