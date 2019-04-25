
/*
    Edge connect two expmap vertex 
    measurement [Rotation, |t|] 

*/


#include "edge_expmap_t_norm.h"

namespace g2o{


EdgeSE3ExpmapNorm::EdgeSE3ExpmapNorm(double norm): 
BaseBinaryEdge<4, Vector4, VertexSE3Expmap, VertexSE3Expmap>(), 
_t_norm(norm){}

EdgeSE3ExpmapNorm::~EdgeSE3ExpmapNorm(){}

bool EdgeSE3ExpmapNorm::read(std::istream& is)
{
    double fake; 
    for(int i=0; i<4; i++)
	is>>fake; 
    is >> _t_norm; 
    for(int i=0; i<4; i++)
    for(int j=i; j<4; j++)
    {
	is >> information()(i,j);
	if(i!=j)
	    information()(j,i) = information()(i,j); 
    }
    return true; 
}

bool EdgeSE3ExpmapNorm::write(std::ostream& os) const
{
    os<<0<<" "<<0<<" "<<0<<" "<<1<<" "<<_t_norm<<" "; 
    for(int i=0; i<4; i++)
    for(int j=i; j<4; j++){
	os <<" "<< information()(i,j); 
    }
    return true; 
}



}
