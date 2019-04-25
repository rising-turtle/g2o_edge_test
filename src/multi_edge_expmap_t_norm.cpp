/*
    Multiple Edges connect k expmap vertex 
    measurement [Rotation, tx, ty, tz] 
    
    vertex 0 is the start point 
    vertex 1 is the incremental dt[tx, ty, tz] // ty almost zero, merely [tx, tz]
*/


#include "multi_edge_expmap_t_norm.h"
#include <g2o/types/slam2d/types_slam2d.h>

namespace g2o{


MultiEdgeSE3ExpmapNorm::MultiEdgeSE3ExpmapNorm(double t_norm): 
BaseMultiEdge<-1, VectorX>(), 
_t_norm(t_norm)
{
    resize(0); 
}

void MultiEdgeSE3ExpmapNorm::computeError()
{
    VertexSE3Expmap* v0 = static_cast<VertexSE3Expmap*>(_vertices[0]); // root pose 
    
    SE3Quat T0 = v0->estimate().inverse(); 
    Vector3 t0 = T0.translation(); 

    VertexPointXY* dt_node = static_cast<VertexPointXY*>(_vertices[1]); 
    Vector2 angle = dt_node->estimate(); 
    double tx, ty, tz; 
    tx = _t_norm * sin(angle(1)) * cos(angle(0)); 
    ty = _t_norm * sin(angle(1)) * sin(angle(0)); 
    tz = _t_norm * cos(angle(1)); 

    for(unsigned int i = 0 ; i< _k-1; i++){
	
	VertexSE3Expmap* vi = static_cast<VertexSE3Expmap*>(_vertices[i+2]); 

	SE3Quat Ti = vi->estimate().inverse(); 

	// TODO: take measurement R12 into consideration 
	SE3Quat T0i = v0->estimate() * Ti; // vi->estimate().inverse(); 
	
	// rotation should be Identity matrix 
	Vector6 err = T0i.log(); 
	_error[0+i*6] = err[0]; _error[1+i*6] = err[1]; _error[2+i*6] = err[2]; 
	
	// translation equal to norm 
	Vector3 ti = Ti.translation(); 
	_error[3+i*6] = ti[0] - ((i+1)*tx + t0[0]); 
	_error[4+i*6] = ti[1] - ((i+1)*ty + t0[1]); 
	_error[5+i*6] = ti[2] - ((i+1)*tz + t0[2]); 
    }
    return ; 
}

void MultiEdgeSE3ExpmapNorm::initialEstimate(g2o::SparseOptimizer* pg, int M, int N, int dt_ID)
{
    setSize(N); 
    for(int i=0; i<N; i++){
	VertexSE3Expmap* v = dynamic_cast<VertexSE3Expmap*>(pg->vertex(M+i)); 
	if(i > 0){
	    vertices()[i+1] = v;     
	}else{
	    vertices()[i] = v; 
	}
    }
    
    // for the delta_dt 
    VertexPointXY* pv = dynamic_cast<VertexPointXY*>(pg->vertex(dt_ID)); 
    vertices()[1] = pv; 

    // set information matrix 
    for(int i=0; i<_dimension; i++)
    for(int j=i; j<_dimension; j++){
	if(i==j)
	    _information(i,j) = 1e7; 
	else {
	    _information(i,j) = 0; 
	    _information(j,i) =0;
	}
    }
}

bool MultiEdgeSE3ExpmapNorm::read(std::istream& is){
    // TODO: 
    return true; 
}

bool MultiEdgeSE3ExpmapNorm::write(std::ostream& os) const {
    // TODO: 
    return true; 
}


}


