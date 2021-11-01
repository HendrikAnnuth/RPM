#include "vec_ops.h"
#include <cmath>
#include <float.h>
using namespace cgx;

bool cgx::equal(const Vec* p1,const Vec* p2)
{
	return p1->x==p2->x && p1->y==p2->y && p1->z==p2->z;
}

Vec* cgx::crossProduct(const Vec* p1,const Vec* p2,Vec* res)
{
	res->x=p1->y*p2->z - p2->y*p1->z;
	res->y=p1->z*p2->x - p2->z*p1->x;
	res->z=p1->x*p2->y - p2->x*p1->y;
	return res;
}

Vec* cgx::scale(const Vec* p,const float scalar,Vec* res)
{
	res->x=p->x*scalar;
	res->y=p->y*scalar;
	res->z=p->z*scalar;
	return res;
}

Vec* cgx::scaleTo(const Vec* p,const float newVectorLength,Vec* res)
{
	return scale(normalize(p,res),newVectorLength,res);
}

Vec* cgx::vec_AB(const Vec* a,const Vec* b,Vec* res)
{
	return subt(b,a,res);
}

Vec* cgx::div(const Vec* p,const float divider,Vec* res)
{
	res->x=p->x/divider;
	res->y=p->y/divider;
	res->z=p->z/divider;
	return res;
}

Vec* cgx::subt(const Vec* p1,const Vec* p2,Vec* res)
{
	res->x=p1->x-p2->x;
	res->y=p1->y-p2->y;
	res->z=p1->z-p2->z;
	return res;
}

float cgx::qlength(const Vec* p)
{
	return p->x*p->x+p->y*p->y+p->z*p->z;
}

float cgx::length(const Vec* p)
{
	return (float) sqrt(qlength(p));
}

float cgx::qdist(const Vec* a,const Vec* b,Vec* res)
{
	return qlength(subt(b,a,res));
}

float cgx::dist(const Vec* a,const Vec* b,Vec* res)
{
	return (float) sqrt(qdist(a,b,res));
}

Vec* cgx::add(const Vec* p1,const Vec* p2,Vec* res)
{
	res->x=p1->x+p2->x;
	res->y=p1->y+p2->y;
	res->z=p1->z+p2->z;
	return res;
}

Vec* cgx::invert(const Vec* p1,Vec* res)
{
	res->x=-p1->x;
	res->y=-p1->y;
	res->z=-p1->z;
	return res;
}

float cgx::scalarProduct(const Vec* p1,const Vec* p2)
{
	return p1->x*p2->x+p1->y*p2->y+p1->z*p2->z;
}

Vec* cgx::normalize(const Vec* p,Vec* res)
{
	float l=length(p);
	if(l==0)
	{
		res->x=0;
		res->y=0;
		res->z=0;
	}
	else
	{
		res->x=p->x/l;
		res->y=p->y/l;
		res->z=p->z/l;
	}
	return res;
}

Vec* cgx::normalOnTriangleCCW(const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* res)
{
	return crossProduct(subt(p3,p2,temp1),subt(p1,p2,temp2),res);
}

Vec* cgx::setTo(Vec* aim,const Vec* source)
{
	aim->x=source->x;
	aim->y=source->y;
	aim->z=source->z;
	return aim;
}

Vec* cgx::setTo(Vec* aim,const float x,const float y,const float z)
{
	aim->x=x;
	aim->y=y;
	aim->z=z;
	return aim;
}

bool cgx::zeroVector(const Vec* p)
{
	return p->x==0 && p->y==0 && p->z==0;
}

Vec* cgx::centroidTriangle(const Vec* p1,const Vec* p2,const Vec* p3,Vec* res)
{
	return div(add(p1,add(p2,p3,res),res),3,res);
}

Vec* cgx::rotateX(const float angleRad,const Vec* p,Vec* res)
{
		res->x=  p->x;
		res->y=  p->y*cos(angleRad) + p->z*sin(angleRad);
		res->z= -p->y*sin(angleRad) + p->z*cos(angleRad);
		return res;
}

Vec* cgx::rotateY(const float angleRad,const Vec* p,Vec* res)
{
		res->x=  p->x*cos(angleRad) - p->z*sin(angleRad);
		res->y=  p->y;
		res->z=  p->x*sin(angleRad) + p->z*cos(angleRad);
		return res;
}

Vec* cgx::rotateZ(const float angleRad,const Vec* p,Vec* res)
{
		res->x=  p->x*cos(angleRad) + p->y*sin(angleRad);
		res->y= -p->x*sin(angleRad) + p->y*cos(angleRad);
		res->z= p->z;
		return res;
}

float cgx::calcAngleRad(const Vec* n1,const Vec* n2,Vec* temp1,Vec* temp2)
{
	return acos(scalarProduct(normalize(n1,temp1),normalize(n2,temp2)));
}

float cgx::qdistPtToPlane(const Vec* pt,const Vec* originPt,const Vec* n,Vec* temp)
{
	return cgx::qdist(projectPtOnPlane(pt,originPt,n,temp),pt,temp);
}

float cgx::distPtToPlane(const Vec* pt,const Vec* originPt,const Vec* n,Vec* temp)
{
	return (float) sqrt(cgx::qdistPtToPlane(pt,originPt,n,temp));
}

bool cgx::ptAbovePlane(const Vec* pt,const Vec* originPt,const Vec* n,Vec* temp)
{
	return  0<scalarProduct(subt(pt,originPt,temp),n);
}

Vec* cgx::inLineSegment(const Vec* p1,const Vec* p2,Vec* temp1,Vec* temp2,Vec* res)
{
	if(scalarProduct(subt(res,p1,temp1),subt(res,p2,temp2))>=0)
		return 0;
	return res;
}


void cgx::barycentricCoordinates(const Vec* p1,const Vec* p2,const Vec* p3,const Vec* p,Vec* u,Vec* v,Vec* w,float& b1,float& b2)
{
	subt(p3,p1,u);
	subt(p2,p1,v);
	subt(p,p1,w);
	float uu = scalarProduct(u, u);
	float uv = scalarProduct(u, v);
	float vv = scalarProduct(v, v);
	float wu = scalarProduct(w, u);
	float wv = scalarProduct(w, v);
	float D = uv * uv - uu * vv;
	b1 = (uv * wu - uu * wv) / D;
    b2 = (uv * wv - vv * wu) / D;
}

void cgx::barycentricCoordinates(const Vec* p1,const Vec* p2,const Vec* p3,const Vec* p,Vec* u,Vec* v,Vec* w,float& b1,float& b2,float& b3)
{
	barycentricCoordinates(p1,p2,p3,p,u,v,w,b2,b3);
	b1=1.0f-(b3+b2);
}
	

Vec* cgx::inTriangle(const Vec* u,const Vec* v,const Vec* w,Vec* res)
{
	float uu = scalarProduct(u, u);
	float uv = scalarProduct(u, v);
	float vv = scalarProduct(v, v);
	float wu = scalarProduct(w, u);
	float wv = scalarProduct(w, v);
	float D = uv * uv - uu * vv;
	vv = (uv * wv - vv * wu) / D;
    if (vv < 0.0 || vv > 1.0)
        return 0;
    uu = (uv * wu - uu * wv) / D;
    if (uu < 0.0 || (vv + uu) > 1.0)  
		return 0;
	return res;
}

Vec* cgx::projectPtOnPlane(const Vec* pt,const Vec* oriPt,const Vec* n,Vec* res)
{
	return add(pt,scale(n,-scalarProduct(n,subt(pt,oriPt,res))/scalarProduct(n,n),res),res);
}

Vec* cgx::projectPtOnRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* res)
{
	crossProduct(subt(pt,oriPt,temp1),dir,res);
	return projectPtOnPlane(pt,oriPt,crossProduct(res,dir,temp1),res);
}

Vec* cgx::projectPtOnTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* u,Vec* v,Vec* w,Vec* res)
{
	subt(p3,p1,u);
	subt(p2,p1,v);
	projectPtOnPlane(pt,p1,crossProduct(u,v,w),res);
	subt(res,p1,w);
	return inTriangle(u,v,w,res);
}

Vec* cgx::intersectionRayPlane(const Vec* oriPtRay,const Vec* dir,const Vec* oriPtPlane,const Vec* n,Vec* res)
{
    float b = scalarProduct(n,dir);
	if (b == 0)
		return 0;
    return add(oriPtRay,scale(dir,-scalarProduct(n,subt(oriPtRay,oriPtPlane,res))/b,res),res);
}

Vec* cgx::intersectionRayTriangle(const Vec* oriPt,const Vec* dir,const Vec* p1,const Vec* p2,const Vec* p3,Vec* u,Vec* v,Vec* w,Vec* res)
{
	subt(p3,p1,u);
	subt(p2,p1,v);
	crossProduct(u,v,w);
    float b = scalarProduct(w,dir);
	if (b == 0)
		return 0;
    subt(add(oriPt,scale(dir,-scalarProduct(w,subt(oriPt,p1,res))/b,res),res),p1,w);
	return inTriangle(u,v,w,res);
}

int cgx::pointOfTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3)
{
  if(equal(pt,p1))
	  return 1;
  if( equal(pt,p2))
	  return 2;
  if(equal(pt,p3))
	  return 3;
  return -1;
}

bool cgx::intersectionTriangleTriangle(const Vec* t1_1,const Vec* t1_2,const Vec* t1_3,const Vec* t2_1,const Vec* t2_2,const Vec* t2_3,Vec* dir,Vec* u,Vec* v,Vec* w,Vec* inter)
{
	float b;int check;
	bool doCheck112=true,doCheck113=true,doCheck123=true,doCheck212=true,doCheck213=true,doCheck223=true;
	if((check=pointOfTriangle(t1_1,t2_1,t2_2,t2_3))!=-1)
	{
		if(pointOfTriangle(t1_2,t2_1,t2_2,t2_3)!=-1 || pointOfTriangle(t1_3,t2_1,t2_2,t2_3)!=-1)
			return false;
		doCheck112=false;
		doCheck113=false;
	}
	else
		if((check=pointOfTriangle(t1_2,t2_1,t2_2,t2_3))!=-1)
		{
			if(pointOfTriangle(t1_3,t2_1,t2_2,t2_3)!=-1)
				return false;
			doCheck112=false;
			doCheck123=false;
		}
		else
			if((check=pointOfTriangle(t1_3,t2_1,t2_2,t2_3))!=-1)
			{
				doCheck113=false;
				doCheck123=false;
			}
	if(check!=-1)
		if(check==1)
		{
			doCheck212=false;
			doCheck213=false;		
		}
		else
			if(check==2)
			{
				doCheck212=false;
				doCheck223=false;		
			}
			else
			{
				doCheck213=false;
				doCheck223=false;
			}

	vec_AB(t2_1,t2_3,u);
	vec_AB(t2_1,t2_2,v);	
	crossProduct(u,v,w);

	if(doCheck112)
	{
		vec_AB(t1_1,t1_2,dir);
		b = scalarProduct(w,dir);
		if (b == 0) 
			return false;
		subt(add(t1_1,scale(dir,-scalarProduct(w,subt(t1_1,t2_1,inter))/b,inter),inter),t2_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}
	if(doCheck113)
	{
		vec_AB(t1_1,t1_3,dir);
		b = scalarProduct(w,dir);
		if (b == 0) 
			return false;
		subt(add(t1_1,scale(dir,-scalarProduct(w,subt(t1_1,t2_1,inter))/b,inter),inter),t2_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}
	if(doCheck123)
	{
		vec_AB(t1_2,t1_3,dir);
		b = scalarProduct(w,dir);
		if (b == 0) 
			return false;
		subt(add(t1_2,scale(dir,-scalarProduct(w,subt(t1_2,t2_1,inter))/b,inter),inter),t2_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}

	vec_AB(t1_1,t1_3,v);
	vec_AB(t1_1,t1_2,v);
	crossProduct(u,v,w);

	if(doCheck212)
	{
		vec_AB(t2_1,t2_2,dir);
		float b;
		b = scalarProduct(w,dir);
		if (b == 0)
			return false;
		subt(add(t2_1,scale(dir,-scalarProduct(w,subt(t2_1,t1_1,inter))/b,inter),inter),t1_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}
	if(doCheck213)
	{
		vec_AB(t2_1,t2_3,dir);
		b = scalarProduct(w,dir);
		if (b == 0) 
			return false;
		subt(add(t2_1,scale(dir,-scalarProduct(w,subt(t2_1,t1_1,inter))/b,inter),inter),t1_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}
	if(doCheck223)
	{
		vec_AB(t2_2,t2_3,dir);
		b = scalarProduct(w,dir);
		if (b == 0) 
			return false;
		subt(add(t2_2,scale(dir,-scalarProduct(w,subt(t2_2,t1_1,inter))/b,inter),inter),t1_1,dir);
		if(inTriangle(u,v,dir,inter))
			return true;
	}
	return false;
}

float cgx::distPtRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* temp2)
{
	return sqrt(qdistPtRay(pt,oriPt,dir,temp1,temp2));
}

float cgx::qdistPtRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* temp2)
{
	projectPtOnRay(pt,oriPt,dir,temp1,temp2);
	subt(pt,temp2,temp1);
	return scalarProduct(temp1,temp1);
}

float cgx::distPtTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4)
{
	return sqrt(qdistPtTriangle(pt,p1,p2,p3,temp1,temp2,temp3,temp4));
}

float cgx::qdistPtTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4)
{
	if(projectPtOnTriangle(pt,p1,p2,p3,temp1,temp2,temp3,temp4))//Point above triangle?
		return qlength(subt(pt,temp4,temp4));
	float res,current;
	res=FLT_MAX;

	current=qdistPtRay(pt,p1,temp1,temp3,temp4);//temp1 is u (see projectPtOnTriangle)
	if(inLineSegment(p1,p3,temp1,temp3,temp4))//temp4 is point projected on ray
		res=current;
	current=qdistPtRay(pt,p1,temp2,temp3,temp4);//temp2 is v (see projectPtOnTriangle)
	if(inLineSegment(p1,p2,temp1,temp3,temp4) && current<res)//temp4 is point projected on ray
		res=current;
	current=qdistPtRay(pt,p2,subt(p2,p3,temp1),temp3,temp4);
	if(inLineSegment(p2,p3,temp1,temp3,temp4) && current<res)//temp4 is point projected on ray
		res=current;

	current=qdist(pt,p1,temp1);
	if(current<res)
		res=current;
	current=qdist(pt,p2,temp1);
	if(current<res)
		res=current;
	current=qdist(pt,p3,temp1);
	if(current<res)
		res=current;
	return res;
}

float cgx::lengthOfLongestTriangleEdge(const Vec* pt1,const Vec* pt2,const Vec* pt3,Vec* temp)
{
	float l1=qlength(subt(pt1,pt2,temp));
	float l2=qlength(subt(pt2,pt3,temp));
	float l3=qlength(subt(pt3,pt1,temp));
	if(l1>=l2 && l1>=l3)
		return sqrt(l1);
	if(l2>=l3)
		return sqrt(l2);
	return sqrt(l3);
}

float cgx::triangleQuality(const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3)
{
	float le=lengthOfLongestTriangleEdge(p1,p2,p3,temp1);
	return length(normalOnTriangleCCW(p1,p2,p3,temp1,temp2,temp3)) / (le*le*SIN60);
}