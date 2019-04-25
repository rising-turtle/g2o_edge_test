// for std
#include <stdio.h>
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <boost/concept_check.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

// #include "edge_expmap_t_norm.h"
#include "multi_edge_expmap_t_norm.h"

#include <opencv2/core/eigen.hpp>

// #include "myG2Oedge.h"

using namespace std;
using namespace cv;

/*
// void check_error( g2o::SparseOptimizer& pg, int M, int N)
void check_error(vector<g2o::EdgeSE3ExpmapNorm*> & vedge)
{
    double sum_angle = 0; 
    double sum_norm_t = 0; 
    
    int N = vedge.size(); 
    for(int i = 1; i<N; i++)
    {
	g2o::EdgeSE3ExpmapNorm* pe = (vedge[i]); 
	pe->computeError(); 
	g2o::Vector4 err = pe->error(); 
	Eigen::Vector3d ae = err.block<3,1>(0,0); 
	double te = err[3]; 
	sum_angle += ae.norm(); 
	sum_norm_t += te; 
    }
    if(N > 0)
    {
	cout <<"average angle_error: "<<sum_angle/(double)(N)<<endl;
	cout <<"average t_norm_error: "<<sum_norm_t/(double)(N)<<endl; 
    }
}*/


bool g_use_encoder_constraint = true; 

const int DT_ID = 7777; 

int main(int argc, char const **argv)
{
  
  vector< vector< Point3f > > object_points;
  vector< vector< Point2f > > image_points;
  vector< Point2f > corners;

  Mat img, gray;
  std::vector<Mat> imgs;
  int board_width, board_height, num_imgs;
  float square_size;

  img = imread("60.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("110.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  img = imread("160.png", CV_LOAD_IMAGE_COLOR);
  imgs.push_back(img);
  //  img = imread("210.png", CV_LOAD_IMAGE_COLOR);
  // imgs.push_back(img);
  // img = imread("260.png", CV_LOAD_IMAGE_COLOR);

  double cx = 322.5009460449219;
  double cy = 242.32693481445312;
  double fx = 615.2476806640625;
  double fy = 615.4488525390625; 

  square_size = 50.0/1000; // mm to m
  bool found = false;
  board_width = 5;
  board_height = 4;
  Size board_size = Size(5, 4);
  vector< Point3f > obj;
  for (int i = 0; i < board_height; i++)
    for (int j = 0; j < board_width; j++)
      obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

  for (int i=0;i<imgs.size();i++){
    cv::cvtColor(imgs[i], gray, CV_BGR2GRAY);
    found = cv::findChessboardCorners(gray, board_size, corners,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if (found) {
      cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

       drawChessboardCorners(gray, board_size, corners, true);
       cv::imshow( "checkerboard", gray );
       cv::waitKey( 0 );  

      // cout << "corners"<<endl<< corners <<endl;

      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }

/*
  Mat rvec,tvec;
  double camera_matrix_data[3][3] = {
      {fx, 0, cx},
      {0, fy, cy},
      {0, 0, 1}
  };
  cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
  solvePnP(obj, image_points[0], cameraMatrix, Mat(), rvec, tvec); 
  cout << tvec<<endl;
  solvePnP(obj, image_points[1], cameraMatrix, Mat(), rvec, tvec); 
  cout << tvec<<endl;
  solvePnP(obj, image_points[2], cameraMatrix, Mat(), rvec, tvec); 
  cout << tvec<<endl;*/
//=========================================================================================================================
  g2o::SparseOptimizer optimizer;
    
  // typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  // typedef LinearSolverCSparse<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;
  // typedef g2o::LinearSolverCSparse::BlockSolverX SlamBlockSolver; 
  typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 
  std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
  linearSolver->setBlockOrdering( false );
  std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );
  optimizer.setAlgorithm( solver );
  optimizer.setVerbose( true );


  // all pseudo 3d points are fixed
  for ( size_t i=0; i<obj.size(); i++ )
  {
      g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
      v->setFixed( true ); 
      v->setId( i );
      
      double z = (obj[i]).z; // avoid zero depth
      double x = (obj[i]).x; 
      double y = (obj[i]).y; 
      v->setMarginalized(true);
      v->setEstimate( Eigen::Vector3d(x,y,z) );
      optimizer.addVertex( v );
  }
  
  int M = obj.size(); // number of feature points 
  int N = imgs.size();  // number of camera pose

  // camera pose || reference is points on wall which is also fixed 
  for ( int i=0; i<imgs.size(); i++ )
  {
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setId( M + i );
    // if ( i == 0)
    //      v->setFixed( true );
    // if(i==0)
    {
	g2o::SE3Quat mea(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.11, -0.066, 0.6 + i*0.2)); 
	// v->setEstimate( g2o::SE3Quat() );
	v->setEstimate(mea);
    }
    
    optimizer.addVertex( v );
  }


  g2o::CameraParameters* camera = new g2o::CameraParameters( (fx+fy)/2, Eigen::Vector2d(cx, cy), 0 );
  camera->setId(0);
  optimizer.addParameter( camera );

  for (int idx =0;idx< imgs.size();idx++){
    for ( size_t i=0; i<(image_points[idx]).size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->vertices() [0] = optimizer.vertex( i );
        edge->vertices() [1] = optimizer.vertex( obj.size()+idx);

	g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(edge->vertices()[1]);
	g2o::VertexSBAPointXYZ* vp = dynamic_cast<g2o::VertexSBAPointXYZ*>(edge->vertices()[0]); 
	// cout<<"node point i = "<<i<<" estimate() "<< vp->estimate()<<endl; 
	// cout<<"node pose j = "<< (obj.size()+idx+1) << "estimate() " <<v->estimate().toVector()<<endl;
    
        edge->setMeasurement( Eigen::Vector2d(((image_points[idx])[i]).x, ((image_points[idx])[i]).y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
	
        // edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
	edge->computeError(); 
	// cout<<"edge-> "<<i<<" "<<obj.size()+idx +1<<" error: "<<edge->error()<<endl;
	
    }
  }

  // add dt node 
  g2o::VertexPointXY* p_dt_v = new g2o::VertexPointXY; 
  p_dt_v->setId(DT_ID); 
  optimizer.addVertex(p_dt_v); 

  const double t_norm = 0.5; 
  // vector<g2o::EdgeSE3ExpmapNorm*> vedge; 
  g2o::MultiEdgeSE3ExpmapNorm* pmul = new g2o::MultiEdgeSE3ExpmapNorm(t_norm); 
  pmul->initialEstimate(&optimizer, M, N, DT_ID); 
  if(g_use_encoder_constraint)
    optimizer.addEdge(pmul); 
  

  /*
  for (int idx =1;idx< imgs.size();idx++){
    // g2o::EdgeSE3* edge_fix = new g2o::EdgeSE3();
    g2o::EdgeSE3ExpmapNorm* edge_fix = new g2o::EdgeSE3ExpmapNorm(t_norm); 
    edge_fix->vertices() [0] = optimizer.vertex( idx+ M -1);
    edge_fix->vertices() [1] = optimizer.vertex( idx+ M );
    Eigen::Matrix<double, 4, 4> information_fix = 1e7*Eigen::Matrix< double, 4,4 >::Identity();
    edge_fix->setInformation( information_fix );
    // edge_fix->setMeasurement( T_prior );
    if(g_use_encoder_constraint)
        optimizer.addEdge(edge_fix);
    vedge.push_back(edge_fix); 
  }*/
  
  if(g_use_encoder_constraint){
    cout<<"encoder constraints are added into graph structure!"<<endl; 
  }else{
    cout<<"no encoder constraint, only bundle adjustment "<<endl; 
  }
  cout<<"before optimization, check error: "<<endl; 

  optimizer.save("./result_before.g2o");
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  optimizer.save("./result_after.g2o");

  if(g_use_encoder_constraint){
    cout<<"encoder constraints are added into graph structure!"<<endl; 
  }else{
    cout<<"no encoder constraint, only bundle adjustment "<<endl; 
  }

  cout<<"after optimization, check error: "<<endl;; 
  // check_error(vedge); 

  // g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>( optimizer.vertex(1) );
  // Eigen::Isometry3d pose = v->estimate();
  // cout<<"Pose="<<endl<<pose.matrix()<<endl;


  // Mat testTTT;
  // eigen2cv(pose.matrix(),testTTT);
  // cout << testTTT<<endl;


  return 0;
}
