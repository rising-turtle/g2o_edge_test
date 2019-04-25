
/*
    Edge connect two expmap vertex 
    measurement [Rotation, |t|] 

*/

#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam3d/types_slam3d.h>

#include <cmath>

namespace g2o{

class EdgeSE3ExpmapNorm : public BaseBinaryEdge<4, Vector4, VertexSE3Expmap, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3ExpmapNorm(double norm); 
    virtual ~EdgeSE3ExpmapNorm(); 
    
    virtual bool read(std::istream& is); 
    virtual bool write(std::ostream& os) const; 
    void computeError(){
	const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]); // Ti2w
	const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); // Tj2w
	
	// TODO: take measurement R12 into consideration 
	SE3Quat Tij = vi->estimate() * vj->estimate().inverse(); 
	
	// rotation should be Identity matrix 
	Vector6 err = Tij.log(); 
	_error[0] = err[0]; _error[1] = err[1]; _error[2] = err[2]; 
	
	// translation equal to norm 
	Vector3 tij = Tij.translation(); 
	double t_norm  = tij.norm(); 
	_error[3] = fabs(t_norm - _t_norm); 
	
    }
    double _t_norm;
};


}
