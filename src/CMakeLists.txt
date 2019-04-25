project(myVOsum)

ADD_EXECUTABLE( main main.cpp) # always test cmake function first

INCLUDE_DIRECTORIES ("/usr/include/eigen3/")

# # SET("PCL_DIR" "/home/jin/Third_party_packages/pcl-pcl-1.9.1/build")
# SET(PCL_DIR "/usr/local/Cellar/pcl/")
# FIND_PACKAGE( PCL 1.9 REQUIRED COMPONENTS common io visualization filters )
# ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
# INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
# LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


# SET(OpenCV_DIR "/Users/lingqiujin/Third_Party_Packages/opencv-2.4.13.6/build")
# find_package( OpenCV 2.4 REQUIRED )
# SET(OpenCV_DIR "/Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build")
# SET(OpenCV_DIR "/usr/local/Cellar/opencv@3/3.4.5_1")

find_package( OpenCV 3.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIB_DIR})

# include_directories( /usr/local/Cellar/opencv@3/3.4.5_1/include )
# set(SIFT_LIBS "/usr/local/Cellar/opencv@3/3.4.5_1/lib/libopencv_xfeatures2d.dylib" )

# set(G2O_ROOT "/home/hzhang8/work/3rdlib")
# SET(G2O_DIR "${G2O_ROOT}/g2o")
# SET(G2O_LIB "${G2O_ROOT}/g2o/lib/")
# include_directories( ${G2O_DIR} )
# link_directories( ${G2O_LIB} )

set(G2O_ROOT /home/davidz/work/3rdlib/g2o/install/usr/local)
include_directories(${G2O_ROOT}/include /usr/include/suitesparse) 
link_directories(${G2O_ROOT}/lib /usr/lib/i386-linux-gnu)
set(G2O_LIBS g2o_types_slam3d g2o_types_slam2d g2o_solver_csparse g2o_stuff g2o_types_sba g2o_core g2o_cli g2o_csparse_extension  cxsparse )

# FIND_PACKAGE( G2O REQUIRED )

SET(CSparse_include_DIR "/usr/local/Cellar/suite-sparse/5.3.0/include/")
# SET(CSparse_include_LIB "/usr/local/Cellar/suite-sparse/5.3.0/lib/")

# find_package(suitesparse REQUIRED)
# include_directories( ${CSparse_include_DIR} )
# link_directories( ${CSparse_include_LIB} )


# ADD_LIBRARY( myG2Oedge myG2Oedge.cpp )
# TARGET_LINK_LIBRARIES( myG2Oedge
#    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_solver_csparse g2o_types_slam3d
#    ${CSparse_include_LIB} 
# )


ADD_EXECUTABLE( testCal testCal.cpp edge_expmap_t_norm.cpp)
TARGET_LINK_LIBRARIES( testCal
    ${OpenCV_LIBS}
    ${G2O_LIBS} ${G2O_LIB}
    # g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    # ${CSparse_include_LIB} 
)


ADD_EXECUTABLE( testCalMult testCalMult.cpp multi_edge_expmap_t_norm.cpp)
TARGET_LINK_LIBRARIES( testCalMult
    ${OpenCV_LIBS}
    ${G2O_LIBS} ${G2O_LIB}
    # g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    # ${CSparse_include_LIB} 
)
