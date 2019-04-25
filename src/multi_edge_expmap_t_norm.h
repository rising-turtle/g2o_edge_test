
/*
    Multiple Edges connect k expmap vertex 
    measurement [Rotation, tx, ty, tz] 
    
    vertex 0 is the start point 
    vertex 1 is the incremental dt[tx, ty, tz] // ty almost zero, merely [tx, tz]
*/

#pragma once 

#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <iostream>

#include <cmath>


namespace g2o{

/*
class VertexUnit:public BaseVertex<2, Vector2>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexUnit(){}
    virtual bool read(std::istream& is); 
    virtual bool write(std::ostream& os); 
    
    virtual void setToOriginImpl(){_estimate.fill(0.); }
    
    virtual void oplusImpl(const double* update_){
	Eigen::Map<const Vector2> update(update_); 
	_estimate += update;
    }
    virtual int estimateDimension() const{ return 2;}
    virtual bool getEstimateData(double* est) const{
	Eigen::Map<Vector2> _est(est); 
	_est = _estimate; return true; 
    }
    virtual bool setEstimateData(const double* est){
	Eugin::Map<Vector2> _est(est); 
	_estimate = _est; return true;  
    }
    virtual bool setMinimalEstimateDataImpl(const double* est){
	_estimate = Eigen::Map<const Vector2>(est); 
	return true; 
    }
    virtual bool getMinimalEstimateData(double* est) const{
	Eigen::Map<Vector2> v(est); 
	v = _estimate; return true; 
    }
};*/

class MultiEdgeSE3ExpmapNorm : public BaseMultiEdge<-1, VectorX>
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
    MultiEdgeSE3ExpmapNorm(double t_norm); 

    void setDimension(int dimension)
    {
	std::cout<<"dimension = "<<dimension<<std::endl; 
	_dimension = dimension; 
	_information.resize(dimension, dimension); 
	_error.resize(dimension, 1); 
	_measurement.resize(dimension, 1); 
    }

    void setSize(int vertices){
	_k = vertices; 
	resize(vertices+1); // id 1 is vertex incremental
	std::cout<<"set size k = "<<vertices<<std::endl;
	setDimension((vertices-1)*6); // (k-1) relative pose estimation 
    }
    
    virtual void initialEstimate(g2o::SparseOptimizer* pg, int M, int N,int );

    virtual void computeError(); 
    virtual bool read(std::istream& is); 
    virtual bool write(std::ostream& os) const; 

    int _k; // number of frames 
    double _t_norm; 
};

}






