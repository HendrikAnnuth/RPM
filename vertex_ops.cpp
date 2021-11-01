#include "vertex_ops.h"
#include "vec_ops.h"
#include "Mesh.h"
#include "VertexSelection.h"
#include "VertexNeigborhood.h"
using namespace cgx;
bool cgx::atBorder(const Edge* e)
{
	return e->face==NULL;
}

Edge* cgx::atBorder(const Vertex* v)
{
	return outGoingBorder(v);
}

bool cgx::twoBorderEdge(const Edge* e)
{
	return e->face==NULL && e->pair->face==NULL;
}

Edge* cgx::twoBorderEdge(const Vertex* v)
{
	if(valenceZeroVertex(v))
		return NULL;
	Edge* e=v->edge;
	do{   
		if(twoBorderEdge(e))
			return e;
		e=e->pair->next;
	}while(e!=v->edge);
	return NULL;
}

float cgx::averageEdgeLength(const Vertex* v,Vec* temp)
{	
	float res=0;
	unsigned short vl=0;
	Edge* longest=NULL;
	Edge* edge = v->edge;
     do {
		  res+=edgeQLength(edge,temp);
		  edge = edge->pair->next;
		  vl++;
     } while (edge != v->edge);
	 return sqrt(res/vl);
}

float cgx::lengthOfLongestEdge(const Face* f,Vec* temp)
{
	return lengthOfLongestTriangleEdge(f->edge->vertex,
									   f->edge->next->vertex,
									   f->edge->next->next->vertex,temp);
}

Edge* cgx::longestEdge(const Vertex* v,Vec* temp)
{
	float best=-1;
	float check;
	Edge* longest=NULL;
	Edge* edge = v->edge;
     do {
		  check=edgeQLength(edge,temp);
		  if(best<check)
		  {
			best=check;
			longest=edge;
		  }
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 return longest;
}

float cgx::lengthOfLongestEdge(const Vertex* v,Vec* temp)
{
	return length(constructVector(longestEdge(v,temp),temp));
}

float cgx::lengthOfShortestEdge(const Vertex* v,Vec* temp)
{
	return length(constructVector(shortestEdge(v,temp),temp));
}

Edge* cgx::shortestEdge(const Vertex* v,Vec* temp)
{
	float best=FLT_MAX;
	float check;
	Edge* shortest=NULL;
	Edge* edge = v->edge;
     do {
		  check=edgeQLength(edge,temp);
		  if(best>check)
		  {
			best=check;
			shortest=edge;
		  }
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 return shortest;
}

Edge* cgx::longestEdge(const Face* f,Vec* temp)
{
	Edge* e1=f->edge;
	Edge* e2=e1->next;
	Edge* e3=e2->next;
	float l1=edgeQLength(e1,temp);
	float l2=edgeQLength(e2,temp);
	float l3=edgeQLength(e3,temp);
	if(l1>=l2 && l1>=l3)
		return e1;
	if(l2>=l3)
		return e2;
	return e3;
}

float cgx::triangleHeight(const Edge* e,Vec* temp1,Vec* temp2,Vec* temp3)
{
	faceNormal(e->face,temp1,temp2,temp3);
	subt(e->vertex,e->pair->vertex,temp2);
	crossProduct(temp3,temp2,temp1);
	return scalarProduct(subt(e->next->vertex,e->vertex,temp3),
						 normalize(temp1,temp2));
}

Vec* cgx::constructVector(const Edge* e,Vec* res)
{
	return subt(e->vertex,e->pair->vertex,res);
}

float cgx::edgeLength(const Edge* e,Vec* temp)
{
	return sqrt(edgeQLength(e,temp));
}

float cgx::edgeQLength(const Edge* e,Vec* temp)
{
	return qlength(constructVector(e,temp));
}

unsigned short cgx::getValence(const Vertex* v)
{
	 unsigned short numOfEdges=0;
	 Edge* edge = v->edge;
	 if(edge==NULL)
		 return 0;
     do {
		  numOfEdges++;
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 return numOfEdges;
}

bool cgx::valenceZeroVertex(const Vertex* v)
{
	return v->edge==NULL;
}

bool cgx::connected(const Vertex* v,const Edge* e)
{
	 Edge* edge = v->edge;
     do {
		  if(e==edge || e==edge->pair)
			  return true;
     } while (edge != v->edge);
	 return false;
}

float cgx::distancePtToFace(const Vec* p,const Face* f,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4)
{
	return distPtTriangle(p,f->edge->vertex,f->edge->next->vertex,f->edge->next->next->vertex,temp1,temp2,temp3,temp4);
}

void cgx::sphereDefinedByFace(Vec& center,float& radius,const Face* f,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4)
{
	//To understand this calculation, read how to calculate the cycle defined by a triangle
	Vec* n2=subt(f->edge->next->next->vertex,f->edge->next->vertex,temp2);
	Vec* bD=crossProduct(faceNormal(f,temp1,temp2,temp3),n2,temp3);
	Vec* b=add(f->edge->next->vertex,div(n2,2,temp4),temp4);

	Vec* n=subt(f->edge->vertex,f->edge->next->vertex,temp1);
	Vec* x=add(f->edge->next->vertex,div(n,2,temp2),temp2);

	float s=(scalarProduct(n,x)-scalarProduct(b,n))/scalarProduct(n,bD);
	center.x=s*bD->x;
	center.y=s*bD->y;
	center.z=s*bD->z;
	add(b,&center,&center);
	radius=length(subt(f->edge->vertex,&center,temp1));
}

bool cgx::noConnection(const Vertex* v1,const Vertex* v2)
{
	return NULL==connection(v1,v2);
}

Vec* cgx::laplaceOperand(const Vertex* v,Vec* temp,Vec* res)
{
	 setTo(res,0,0,0);
	 unsigned short numOfEdges=0;
	 Edge* edge = v->edge;
     do {
		  numOfEdges++;
		  add(res,constructVector(edge,temp),res);
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 return div(res,numOfEdges,res);
}

Vec* cgx::borderLaplaceOperand(const Vertex* v,Vec* temp,Vec* res)
{
	 Edge* e=inGoingBorder(v);
	 return div(add(constructVector(e->pair,res),constructVector(e->next,temp),res),2.0f,res);
}

Vec* cgx::faceNormal(const Face* f,Vec* temp1,Vec* temp2,Vec* res)
{
	return normalOnTriangleCCW(f->edge->vertex,f->edge->next->vertex,f->edge->next->next->vertex,temp1,temp2,res);
}

float cgx::faceQuality(const Face* f,Vec* temp1,Vec* temp2,Vec* temp3)
{
	return triangleQuality(f->edge->vertex,f->edge->next->vertex,f->edge->next->next->vertex,temp1,temp2,temp3);
}

Vec* cgx::vertexNormal(const Vertex* v,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res)
{
	setTo(res,0,0,0);
	if(valenceZeroVertex(v) || valenceOneVertex(v))
		return res;
	 unsigned short normals=0;
	 Edge* edge = v->edge;
	 if(v->edge==NULL)
		return res;
     do {
		  if(edge->face!=NULL)//NULL -> Case of Border-Vertex
		  {
			  normals++;
			  add(res,faceNormal(edge->face,temp1,temp2,temp3),res);
		  }
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 if(normals==0)
		 return res;
	 return normalize(div(res,normals,res),res);
}

int cgx::edgeDist(Vertex* v1,Vertex* v2,VertexNeigborhood* vn)
{
	vn->clean();
	vn->addVertex(v1);
	while(vn->numOfVertices!=0 && !vn->insightNeighborhood(v2))
		vn->expandNeigborhood();
	if(vn->numOfVertices==0)
		return -1;
	return vn->neighborhoodDegree;
}

void connect(EdgePath* path,Vertex* v1,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int dist)
{
	assert(edgeDist(v1,v2,&VertexNeigborhood())==dist);
	assert(dist>0);
	if(dist==1)
	{
		assert(cgx::connection(v1,v2)!=NULL);
		path->add(path->last,connection(v1,v2));
		assert(path->invariant());
	}
	else
	{
		ef1->init(v1);
		ef2->init(v2);
		while( dist - (ef1->expansionLevel + ef2->expansionLevel) != 1 )
		{
			if(ef1->frontVerticesTotal > ef2->frontVerticesTotal)
				ef2->expandFront();
			else
				ef1->expandFront();
		}
		unsigned int i,firstDist;
		Vertex* v;
		if( ef1->front.size() < ef2->front.size() )
		{
			firstDist=ef1->expansionLevel+1;
			do
			{
				i=0;
				ef1->singleExpansion();
				while(i<ef1->numAddToFront && ef2->front.search(ef1->addToFront[i])==NULL)
					i++;
			}while(i==ef1->numAddToFront);
			v=ef1->addToFront[i];
		}
		else
		{
			firstDist=ef1->expansionLevel;
			do
			{
				i=0;
				ef2->singleExpansion();
				while(i<ef2->numAddToFront && ef1->front.search(ef2->addToFront[i])==NULL)
					i++;
			}while(i==ef2->numAddToFront);
			v=ef2->addToFront[i];
			
		}
		connect(path,v1,v,ef1,ef2,firstDist);
		connect(path,v,v2,ef1,ef2,dist-firstDist);
	}
}

bool init_connect(EdgePath* path,Vertex* v1,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize)
{
	if(v1==v2)
		return false;
	EdgeFront* first=ef1;
	ef1->init(v1);

static int a=0;
a++;
EdgeFront* withOne=ef1;

	ef2->init(v2);
	unsigned int i;
	do
	{
		i=0;
		if(ef1->front.size() > ef2->front.size())
			std::swap(ef1,ef2);
		while(i<ef1->front.size() && ef2->front.search(ef1->front.getVal(i))==NULL)
			i++;
		if(i<ef1->front.size())
		{
			Vertex* v=ef1->front.getVal(i);	
			if(first!=ef1)
				std::swap(ef1,ef2);
			unsigned int dist=ef2->expansionLevel;
			connect(path,v1,v,ef1,ef2,ef1->expansionLevel);
			connect(path,v,v2,ef1,ef2,dist);
			return true;
		}
		if(ef1->frontVerticesTotal + ef2->frontVerticesTotal > searchSpaceSize)
			return false;
		if(ef1->frontVerticesTotal > ef2->frontVerticesTotal)
			std::swap(ef1,ef2);
		ef1->expandFront();

//if(a>=3284)
//	if(ef1==withOne)
//		ef1->invariant(v1);
//	else
//		ef1->invariant(v2);

		while(i<ef1->front.size() && ef2->front.search(ef1->front.getVal(i))==NULL)
			i++;
	}while(ef1->front.size()!=0);
	return false;
}


bool cgx::connectionPath(EdgePath* path,Vertex* v1,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize)
{
	path->clean();
	if(init_connect(path,v1,v2,ef1,ef2,searchSpaceSize))
		return true;
	return false;
}

bool cgx::connectionPath(Vertex* v1,EdgePath* path,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize)
{
	if(path->empty())
		return false;
	path->reverse();
	if(init_connect(path,path->last->val->vertex,v1,ef1,ef2,searchSpaceSize))
	{
		path->reverse();
		return true;
	}
	path->clean();
	return false;
}

bool cgx::connectionPath(EdgePath* path,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize)
{
	if(path->empty())
		return false;
	if(init_connect(path,path->last->val->vertex,v2,ef1,ef2,searchSpaceSize))
		return true;
	path->clean();
	return false;
}

bool cgx::connection(Vertex* v1,Vertex* v2,VertexNeigborhood* vn)
{
	return edgeDist(v1,v2,vn)!=-1;
}

bool cgx::trianglesIntersection(Face* f1,Face* f2,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4,Vec* temp5)
{
	return intersectionTriangleTriangle(f1->edge->vertex,f1->edge->next->vertex,f1->edge->next->next->vertex,
										f2->edge->vertex,f2->edge->next->vertex,f2->edge->next->next->vertex,
										temp1,temp2,temp3,temp4,temp5);
}

Vec* cgx::borderVertexNormal(const Vertex* v,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res)
{
	setTo(res,0,0,0);
	Edge* e=inGoingBorder(v);
	if(!e || !e->pair->face || !e->next->pair->face || 
	   valenceZeroVertex(v) || valenceOneVertex(v))
		return res;
	faceNormal(e->pair->face,temp1,temp2,res);
	subt(e->vertex,e->pair->vertex,temp1);
	crossProduct(res,temp1,temp3);
	e=e->next;
	faceNormal(e->pair->face,temp1,temp2,res);
	subt(e->vertex,e->pair->vertex,temp1);
	crossProduct(res,temp1,temp2);
	return normalize(div(add(temp2,temp3,res),2.0f,res),res);
	
	//setTo(res,0,0,0);
	//if(valenceZeroVertex(v) || valenceOneVertex(v))
	//	return res;
	//Edge* e=inGoingBorder(v);
	//Vec* m=div(subt(e->next->vertex,e->pair->vertex,temp1),2,temp1);
	//add(m,e->pair->vertex,m);
	//e=e->next->pair->next;
	//unsigned short c=1;
	//while(e->pair->face!=NULL)
	//{
	//	c++;
	//	add(res,subt(m,e->vertex,temp2),res);
	//	e=e->pair->next;
	//}
	//return 	normalize(div(res,(float)c,res),res);
}

float cgx::curvature(const Vertex* v,const Vec* v_normal,Vec* temp1,Vec* temp2)
{
	if(zeroVector(v_normal))
		return 0.0f;
	 setTo(temp2,0,0,0);
	 unsigned short numOfEdges=0;
	 float edgeQLengthSum=0.0f;
	 Edge* edge = v->edge;
     do {
		  numOfEdges++;
		  constructVector(edge,temp1);
		  edgeQLengthSum+=qlength(temp1);
		  add(temp2,temp1,temp2);
		  edge = edge->pair->next;
     } while (edge != v->edge);
	 return distPtToPlane(add(v,div(temp2,numOfEdges,temp2),temp2),v,v_normal,temp1) /
			sqrt(edgeQLengthSum/numOfEdges);
}

float cgx::borderCurvature(const Vertex* v,const Vec* v_normal,Vec* temp1,Vec* temp2)
{
	Edge* e=inGoingBorder(v);
	if(e==NULL)
		return 0.0f;
	constructVector(e->pair,temp1);
	constructVector(e->next,temp2);
	float aveEdgeLength=sqrt((qlength(temp1)+qlength(temp2))/2.0f);
	return distPtToPlane(add(v,div(add(temp1,temp2,temp2),2.0f,temp2),temp2),v,v_normal,temp2) / aveEdgeLength;
}

float cgx::surfaceArea(const Face* f,Vec* temp1,Vec* temp2,Vec* temp3)
{
	 return length(faceNormal(f,temp1,temp2,temp3))/2.0f;
}

bool cgx::miniHole(const Edge* e)
{
	return e->face==NULL && e==e->next->next->next &&
		   (e->next->pair->next->pair != e ||
			e->next->next->pair->next->pair != e->next ||
			e->pair->next->pair != e->next->next);
}

bool cgx::multipleBorderVertex(const Vertex* v)
{	
	Edge* e=inGoingBorder(v);
	return e && inGoingBorderNext(v,e);
}

unsigned int cgx::multipleBorderVertexSeparations(const Vertex* v)
{	
	Edge* e=inGoingBorder(v);
	unsigned int res=0;
	while(e!=NULL)
	{
		res++;
		e=inGoingBorderNext(v,e);
	}
	return res;
}

Edge* cgx::connection(const Vertex* neighborhood,const Vertex* potentcialNeighbor)
{
	if(valenceZeroVertex(neighborhood))
		return NULL;
	Edge* e=neighborhood->edge;
     do {
		 if(e->vertex==potentcialNeighbor)
			 return e;
		  e = e->pair->next;
     } while (e != neighborhood->edge);
   return NULL;
}

bool  cgx::bridgeRailing(const Edge* e)
{
	return atBorder(e) &&
		   connection(e->vertex,e->next->next->vertex) &&
		   !miniHole(e) &&
		   getValence(e->vertex)>4 &&
		   getValence(e->next->vertex)>2 &&
		   getValence(e->next->next->vertex)>4;
}

bool cgx::valenceOneVertex(const Vertex* v)
{
	return v->edge!=NULL && 
		   v->edge==v->edge->pair->next;
}

bool cgx::valenceTwoVertex(const Vertex* v)
{
	return v->edge!=NULL &&
		   v->edge!=v->edge->pair->next && 
		   v->edge==v->edge->pair->next->pair->next;
}

Edge* cgx::before(const Edge* e)
{
	Edge* edgeBefore=(Edge*)e;
	do{
		edgeBefore=edgeBefore->pair->next;
	}while(edgeBefore->pair->next!=e);
	return edgeBefore->pair;
}

Edge* cgx::outGoingBorder(const Vertex* v)
{
	if(valenceZeroVertex(v))
		return NULL;
	Edge* e=v->edge;
	do e=e->pair->next;
	while(e->face!=NULL && e!=v->edge);
	return (e->face==NULL)?(e):(NULL);
}

Edge* cgx::inGoingBorder(const Vertex* v)
{
	if(valenceZeroVertex(v))
		return NULL;
	Edge* e=v->edge;
	//The following if statment assures that 
	//inGoingBorder and outGoingBorder allways return 
	//corrisponding edges in case of a multiple border vertex
	if(e->pair->face!=NULL)
	{
		do e=e->pair->next;
		while(e->pair->face!=NULL && e!=v->edge);
	}
	return (e->pair->face==NULL)?(e->pair):(NULL);
}

Edge* cgx::outGoingBorderNext(const Vertex* v,Edge* last)
{
	if(last->face!=NULL)
		return NULL;
	Edge* e=last;
	//in outGoingBorder the first possible result for an out going border 
	//is v->edge->pair->next therefor end needs to be v->edge->pair->next
	Edge* end=v->edge->pair->next;
	do e=e->pair->next;
	while(e->face!=NULL && e!=end);
	return e==end ? NULL : e;
}

Edge* cgx::inGoingBorderNext(const Vertex* v,Edge* last)
{
	if(last->face!=NULL)
		return NULL;
	Edge* e=last->pair;
	Edge* end=v->edge;
	do e=e->pair->next;
	while(e->pair->face!=NULL && e!=end);
	return e==end ? NULL : e->pair;
}

Vertex* cgx::vertexSplit(Vertex* v,Edge* vsE,Vec* posNewV,Mesh* mesh,VertexSelection* effectedVertices)
{	
	//See vertex split operation in computer graphics standard literatur
	Vertex* newV=mesh->createVertex(posNewV->x,posNewV->y,posNewV->z,vsE);

	if(effectedVertices)
	{
		effectedVertices->addUnique(newV);
		effectedVertices->addUnique(vsE->pair->vertex);
	}

	vsE->pair->vertex=newV;
	Edge* left=before(vsE)->pair;
	Edge* right=vsE->pair->next;
	bool takeRight=true;
	unsigned short takeOverEdges=getValence(v);
	if(takeOverEdges<6)
		takeOverEdges=0;
	else
		takeOverEdges=(takeOverEdges-4)/2;
	while(takeOverEdges>0)
	{
		if(takeRight)
		{
			right->pair->vertex=newV;
			right=right->pair->next;
		}
		else
		{	
			left->pair->vertex=newV;
			if(left->face==NULL)
				left=before(left)->pair;
			else
				left=left->next->next->pair;
		}
		takeRight=!takeRight;
		takeOverEdges--;
	}
	Edge* beforeRight;
	if(right->face==NULL)
		beforeRight=before(right);
	Edge* newLeftEdge3;
	if(left->pair->face==NULL)
	{
		newLeftEdge3=mesh->createEdge(NULL,newV,NULL,left->pair->next);
		left->pair->next=newLeftEdge3;
	}
	else
	{
		//Left 1
		left->pair->face->edge=left->pair->next;
		Edge* newLeftEdge1=mesh->createEdge(left->pair->face,newV,NULL,left->pair->next);
		left->pair->next->next->next=newLeftEdge1;

		//Left 2
		Face* newLeftFace=mesh->createFace(left->pair);
		left->pair->face=newLeftFace;
		Edge* newLeftEdge2=mesh->createEdge(newLeftFace,left->vertex,newLeftEdge1,left->pair);
		newLeftEdge1->pair=newLeftEdge2;
		newLeftEdge3=mesh->createEdge(newLeftFace,newV,NULL,newLeftEdge2);
		left->pair->next=newLeftEdge3;
	}
	v->edge=newLeftEdge3;
	if(right->face==NULL)
	{
		Edge* newRightEdge3=mesh->createEdge(NULL,v,newLeftEdge3,right);
		newLeftEdge3->pair=newRightEdge3;
		beforeRight->next=newRightEdge3;
	}
	else
	{	
		//Right 1
		right->face->edge=right->next;
		Edge* newRightEdge1=mesh->createEdge(right->face,right->vertex,NULL,right->next);
		right->next->next->next=newRightEdge1;

		//Right 2
		Face* newRightFace=mesh->createFace(right);
		right->face=newRightFace;
		Edge* newRightEdge2=mesh->createEdge(newRightFace,newV,newRightEdge1,NULL);
		newRightEdge1->pair=newRightEdge2;
		right->next=newRightEdge2;
		Edge* newRightEdge3=mesh->createEdge(newRightFace,v,newLeftEdge3,right);
		newLeftEdge3->pair=newRightEdge3;
		newRightEdge2->next=newRightEdge3;
	}
	return newV;
}

unsigned int cgx::calcECError(const Edge* e)
{	
	//This is a simple error formula, which calculates the edge to delete
	//to keep the best valence relations (see Seidel 03)
	short a=static_cast<short>(getValence(e->pair->vertex));
	short b=static_cast<short>(getValence(e->vertex));
	short c=static_cast<short>(getValence(e->next->vertex));
	short d=static_cast<short>(getValence(e->pair->next->vertex));
	unsigned int res=0;
	if(!atBorder(e->vertex) || !atBorder(e->pair->vertex))
	{
		int temp=(a+b-10);
		res+=static_cast<unsigned int>(temp*temp);
		temp=(c-7);
		res+=static_cast<unsigned int>(temp*temp);
		temp=(d-7);
		res+=static_cast<unsigned int>(temp*temp);
	}
	else
	{
		if(e->face==NULL)
			c=d;
		int temp=(a+b-7);
		res+=static_cast<unsigned int>(temp*temp);
		temp=(c-7);
		res+=static_cast<unsigned int>(temp*temp);
	}
	return res;
}

bool cgx::twoEdgeConnection(Edge* edge,Edge*& toConnecter,Edge*& fromConnecter)
{
	edge=edge->pair;
	Vertex* v=edge->vertex;
	Edge* end;
	if(edge->face==NULL)
		end=edge;
	else
		end=edge->next->next->pair;
	if(edge->pair->face==NULL)
		toConnecter=edge->pair->next;
	else
		toConnecter=edge->pair->next->pair->next;
	while (toConnecter!=end)
	{
		fromConnecter=connection(toConnecter->vertex,v);
		if(fromConnecter)
		   return true;
		toConnecter=toConnecter->pair->next;
	}
	return false;
}

unsigned int numOfVerticesNextToBottleneck(const Edge* e1,const Edge* e2,const Edge* e3,VertexSelection* temp)
{
	temp->clean();
	Edge* e=e1->pair->next;
	while(e->vertex!=e2->vertex)
	{
		temp->addUnique(e->vertex);
		e=e->pair->next;
	}
	e=e2->pair->next;
	while(e->vertex!=e3->vertex)
	{
		temp->addUnique(e->vertex);
		e=e->pair->next;
	}
	e=e3->pair->next;
	while(e->vertex!=e1->vertex)
	{
		temp->addUnique(e->vertex);
		e=e->pair->next;
	}
	return temp->size();
}

void cutOutBottleneck(Edge* e1,Edge* e2,Edge* e3,Mesh* mesh,VertexSelection* temp,VertexSelection* effectedVertices)
{
	if(numOfVerticesNextToBottleneck(e1,e2,e3,temp) > numOfVerticesNextToBottleneck(e2->pair,e1->pair,e3->pair,temp))
	{
		Edge* temp=e1->pair;
		e1=e2->pair;
		e2=temp;
		e3=e3->pair;
	}
	if(e1->pair->next->vertex!=e2->vertex)
		cutOutPieSlice(e1->pair->next,before(e3->pair),mesh,effectedVertices);
	if(e2->pair->next->vertex!=e3->vertex)
		cutOutPieSlice(e2->pair->next,before(e1->pair),mesh,effectedVertices);
	if(e3->pair->next->vertex!=e1->vertex)
		cutOutPieSlice(e3->pair->next,before(e2->pair),mesh,effectedVertices);
	closeMiniHole(e1->pair,mesh,NULL);
}

bool cgx::bottleneck(const Vertex* v,Edge*& e1,Edge*& e2,Edge*& e3)
{
	if(valenceZeroVertex(v))
		return false;
	e1=v->edge;
	do{
		if(twoEdgeConnection(e1,e2,e3))
			return true;
		e1=e1->pair->next;
	}while(e1!=v->edge);
	return false;
}

Edge* cgx::collapsibleEdge(const Vertex* v)
{
	if(valenceZeroVertex(v))
		return NULL;
	unsigned short best=USHRT_MAX;
	unsigned short check;
	Edge* toCollapse=NULL;
	Edge *temp1,*temp2;
	unsigned short valence=0;
	Edge* edge = v->edge;
    do {
		  check=calcECError(edge);
		  if(best>check && !twoEdgeConnection(edge,temp1,temp2))
		  {
			best=check;
			toCollapse=edge;
		  }
		  valence++;
		  edge = edge->pair->next;
    } while (edge != v->edge);
	if(valence == 3 &&  toCollapse && //A thetrahedron can not be collapesed, to a sealed triangle
	   getValence(toCollapse->vertex) == 3 && 
	   !atBorder(v) && !atBorder(toCollapse->vertex))
		return NULL;
	return toCollapse;
}


Edge* cgx::connectVertices(Vertex* v1,Vertex* v2,Mesh* mesh,VertexSelection* effectedVertices,Vec* tp0,Vec* tp1,Vec* tp2)
{
	Edge* v1Edge=inGoingBorder(v1);
	Edge* v2Edge=inGoingBorder(v2);
	if(v1Edge==NULL || v2Edge==NULL)
		return NULL;
	Vertex* v1Left=v1Edge->pair->vertex;
	Vertex* v1Right=v1Edge->next->vertex;
	Vertex* v2Left=v2Edge->next->vertex;
	Vertex* v2Right=v2Edge->pair->vertex;

	float check,best=triangleQuality(v1,v2,v2Left,tp0,tp1,tp2);
	Vertex* p1=v1;
	Vertex* p2=v2;
	Vertex* p3=v2Left;
	Edge* towards=v1Edge;
	Edge* between=v2Edge->next;
	if(v1Left!=v2Left)
	{
		check=triangleQuality(v2,v1Left,v1,tp0,tp1,tp2);
		if(check>best)
		{
			best=check;
			p1=v2;
			p2=v1Left;
			p3=v1;
			towards=v2Edge;
			between=v1Edge;
		}
	}
	check=triangleQuality(v1,v2Right,v2,tp0,tp1,tp2);
	if(check>best)
	{
		best=check;
		p1=v1;
		p2=v2Right;
		p3=v2;
		towards=v1Edge;
		between=v2Edge;
	}
	if(v1Right!=v2Right)
	{
		check=triangleQuality(v2,v1,v1Right,tp0,tp1,tp2);
		if(check>best)
		{
			best=check;
			p1=v2;
			p2=v1;
			p3=v1Right;
			towards=v2Edge;
			between=v1Edge->next;
		}
	}
	createTriangle(p1,p2,p3,between,towards,mesh,effectedVertices);
	return between;
}

void cgx::createTriangle(Vertex* v3,Vertex* v1,Vertex* v2,Edge* e2,Edge* beforeV3,Mesh* mesh,VertexSelection* effectedVertices)
{
	Face* f=mesh->createFace(e2);
	e2->face=f;
	Edge* e1=NULL;
	Edge* e3=NULL;
	bool e1NewlyCreated=false;
	Edge* vertex_connection=connection(v3,v1);
	if(beforeV3->next->vertex==v1 || vertex_connection)//Triangle is in the corner of a border
	{
		if(vertex_connection!=beforeV3->next)//multipleBorderVertex edges in between
			cutOutPieSlice(beforeV3->next,before(vertex_connection),mesh,effectedVertices);
		e1=beforeV3->next;
		e1->face=f;
		if(e1->next!=e2)//multipleBorderVertex edges in between
			cutOutPieSlice(e1->next,before(e2),mesh,effectedVertices);
	}
	else
	{
		e1=mesh->createEdge(f,v1,NULL,e2);
		e1->pair=mesh->createEdge(NULL,v3,e1,beforeV3->next);
		before(e2)->next=e1->pair;
		e1NewlyCreated=true;
	}
	vertex_connection=connection(v2,v3);
	if(e2->next->vertex==v3 || vertex_connection)//Triangle is in the corner of a border
	{
		if(vertex_connection!=e2->next)//multipleBorderVertex edges in between
			cutOutPieSlice(e2->next,before(vertex_connection),mesh,effectedVertices);
		e3=e2->next;
		e3->face=f;
		if(e1NewlyCreated)
			e3->next=e1;
		else
		    if(e3->next!=e1)//multipleBorderVertex edges in between
				cutOutPieSlice(e3->next,before(e1),mesh,effectedVertices);
	}
	else
	{
		e3=mesh->createEdge(f,v3,NULL,e1);
		e3->pair=mesh->createEdge(NULL,v2,e3,e2->next);
		e2->next=e3;
		beforeV3->next=e3->pair;
	}
}

Edge* cgx::fixMultipleBorderVertex(Vertex* ne_v,Edge* border1,Edge* border2,Mesh* mesh,VertexSelection* effectedVertices,Vec* tp0,Vec* tp1,Vec* tp2)
{
		Edge* res;
		if( triangleQuality(border1->pair->vertex,ne_v,border1->next->vertex,tp0,tp1,tp2) >
			triangleQuality(border2->pair->vertex,ne_v,border2->next->vertex,tp0,tp1,tp2)
		  )
			res=border1;
		else
			res=border2;
		createTriangle(res->pair->vertex,ne_v,res->next->vertex,res->next,before(res),mesh,effectedVertices);
		if(effectedVertices)effectedVertices->addUnique(res->next->vertex);
		if(effectedVertices)effectedVertices->addUnique(res->next->next->vertex);
		return res;
}

void cgx::closeMiniHole(Edge* e, Mesh* mesh,VertexSelection* effectedVertices)
{
	assert(e->vertex==e->next->next->next->vertex);
	if(effectedVertices)
	{
		effectedVertices->addUnique(e->vertex);
		effectedVertices->addUnique(e->next->vertex);
		effectedVertices->addUnique(e->next->next->vertex);
	}
	Face* f=mesh->createFace(e);
	e->face=f;
	e->next->face=f;
	e->next->next->face=f;
	
}

bool cgx::closeHole(Edge* e,Mesh* mesh,VertexSelection* effectedVertices)
{
	unsigned short num=0;
	Edge* start=e;
	do{
		e=e->next;
		num++;
	}while(e!=start);
	num-=2;
	while(num>0)
	{	
		if(num % 2 == 0)
			e=before(e);
		start=e;
		while(e!=NULL && connection(e->vertex,e->next->next->vertex) && e->next->next->next->vertex!=e->vertex)
		{
			e=before(e);
			if(e==start)
				e=NULL;
		}
		if(e==NULL)
			break;
		createTriangle(e->vertex,e->next->vertex,e->next->next->vertex,e->next->next,e,mesh,effectedVertices);
		num--;
	}
	return e!=NULL;
}

void cgx::edgeCollapse(Edge* edgeToCollapse,Mesh* mesh,VertexSelection* effectedVertices)
{
	mesh->deleteVertex(edgeToCollapse->pair->vertex);
	if(effectedVertices)
	{
		effectedVertices->del(edgeToCollapse->pair->vertex);
		effectedVertices->addUnique(edgeToCollapse->vertex);
		if(edgeToCollapse->face!=NULL)
			effectedVertices->addUnique(edgeToCollapse->next->vertex);
		if(edgeToCollapse->pair->face!=NULL)
			effectedVertices->addUnique(edgeToCollapse->pair->next->vertex);
	}

	Edge* edge=edgeToCollapse->pair->next;
	while (edge != edgeToCollapse)//Connect edges to new vertex
	{
			edge->pair->vertex=edgeToCollapse->vertex;
			edge = edge->pair->next;
	}
	if(twoBorderEdge(edgeToCollapse) && (valenceOneVertex(edgeToCollapse->pair->vertex) || valenceOneVertex(edgeToCollapse->vertex)))
	{
		if(valenceOneVertex(edgeToCollapse->pair->vertex) && valenceOneVertex(edgeToCollapse->vertex))
			edgeToCollapse->vertex->edge=NULL;
		else
		{
			if(valenceOneVertex(edgeToCollapse->pair->vertex))
			{
				edgeToCollapse->vertex->edge=edgeToCollapse->next;
				before(edgeToCollapse->pair)->next=edgeToCollapse->next;
			}
			else
			{			
				edgeToCollapse->vertex->edge=edgeToCollapse->next->next;
				before(edgeToCollapse)->next=edgeToCollapse->next->next;
			}
		}
	}
	else
	{
		if(edgeToCollapse->face!=NULL)
		{
			edgeToCollapse->next->pair->pair=edgeToCollapse->next->next->pair;
			edgeToCollapse->next->next->pair->pair=edgeToCollapse->next->pair;
			edgeToCollapse->next->vertex->edge=edgeToCollapse->next->pair;
		}
		else
			before(edgeToCollapse)->next=edgeToCollapse->next;
		if(edgeToCollapse->pair->face!=NULL)
		{
			edgeToCollapse->pair->next->pair->pair=edgeToCollapse->pair->next->next->pair;
			edgeToCollapse->pair->next->next->pair->pair=edgeToCollapse->pair->next->pair;
			edgeToCollapse->pair->next->vertex->edge=edgeToCollapse->pair->next->pair;
		}
		else
			before(edgeToCollapse->pair)->next=edgeToCollapse->pair->next;
		edgeToCollapse->vertex->edge=edgeToCollapse->next->pair->next;
		if(edgeToCollapse->pair->face!=NULL)
		{
			mesh->deleteFace(edgeToCollapse->pair->face);
			mesh->deleteEdge(edgeToCollapse->pair->next->next);
			mesh->deleteEdge(edgeToCollapse->pair->next);
		}
		if(edgeToCollapse->face!=NULL)
		{
			mesh->deleteFace(edgeToCollapse->face);
			mesh->deleteEdge(edgeToCollapse->next->next);
			mesh->deleteEdge(edgeToCollapse->next);
		}
		mesh->deleteEdge(edgeToCollapse->pair);
		mesh->deleteEdge(edgeToCollapse);
	}
}

void cgx::edgeSwap(Edge* swapEdge,VertexSelection* effectedVertices)
{	
	if(swapEdge->face!=NULL && swapEdge->pair->face!=NULL && cgx::noConnection(swapEdge->next->vertex,swapEdge->pair->next->vertex))
	{
		if(effectedVertices)
		{
			effectedVertices->addUnique(swapEdge->vertex);
			effectedVertices->addUnique(swapEdge->pair->vertex);
			effectedVertices->addUnique(swapEdge->next->vertex);
			effectedVertices->addUnique(swapEdge->pair->next->vertex);	
		}

		//New vertex setting
		swapEdge->vertex->edge=swapEdge->next;
		swapEdge->pair->vertex->edge=swapEdge->pair->next;
		swapEdge->vertex=swapEdge->next->vertex;
		swapEdge->pair->vertex=swapEdge->pair->next->vertex;

		//New face setting
		swapEdge->face->edge=swapEdge->next->next;
		swapEdge->pair->face->edge=swapEdge->pair->next->next;
		swapEdge->next->face=swapEdge->pair->face;
		swapEdge->pair->next->face=swapEdge->face;

		//New edge setting
		swapEdge->pair->next->next->next=swapEdge->next;
		swapEdge->next->next->next=swapEdge->pair->next;
		Edge* tempPair=swapEdge->pair->next->next;
		Edge* temp=swapEdge->next->next;
		swapEdge->pair->next->next=swapEdge;
		swapEdge->next->next=swapEdge->pair;
		swapEdge->pair->next=tempPair;
		swapEdge->next=temp;
	}
}

void cgx::cutOutPieSlice(Edge* start,Edge* end,Mesh* mesh,VertexSelection* effectedVertices)
{
	if(effectedVertices)effectedVertices->addUnique(start->pair->vertex);
	if(start==end->next)//Everything arround the vertex need to be deleted
		start->pair->vertex->edge=NULL;
	else
	{
		start->pair->vertex->edge=end->next;
		before(start)->next=end->next;
		if(start->face!=NULL)
		{
			start->next->next->face=NULL;
			if(effectedVertices)effectedVertices->addUnique(start->next->vertex);
		}
		if(end->face!=NULL)
		{
			mesh->deleteFace(end->face);
			end->next->face=NULL;
			end->next->next->face=NULL;
			if(effectedVertices)effectedVertices->addUnique(end->next->vertex);
		}
	}
	Edge* e=start;
	end=end->next;
	do
	{	
		if(e->face!=NULL)
		{
			mesh->deleteFace(e->face);
			e->next->face=NULL;
		}
		if(e->next==e->pair)//vertex with valence 1
			e->vertex->edge=NULL;
		else
		{
			before(e->pair)->next=e->next;
			e->vertex->edge=e->next;
		}
		if(effectedVertices)effectedVertices->addUnique(e->vertex);
		mesh->deleteEdge(e->pair);
		mesh->deleteEdge(e);
		e=e->pair->next;
	}while(e!=end);//end can be deleted at this point, but no data is accessed here.
}

void cgx::cutOutVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices)
{
	if(!valenceZeroVertex(v))
		cutOutPieSlice(v->edge->pair->next,v->edge->pair,mesh,effectedVertices);
	mesh->deleteVertex(v);
	if(effectedVertices)effectedVertices->del(v);
}

void cgx::cutMultipleBorderVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices)
{
	Edge *start,*end;
	Edge* currentStart;
	Edge* e=v->edge;
	unsigned short best=0;
	unsigned short check=1;
	while(e->face!=NULL)
		e=e->pair->next;
	v->edge=e;
	currentStart=e;
	//This will figure out the biggist pie slice of the multiple border vertex.
	//Which will remain
	do{
		e=e->pair->next;
		if(e->face==NULL)
		{
			if(best<check)
			{
				start=e;
				end=before(currentStart);
				best=check;
			}
			currentStart=e;
			check=0;
		}
		check++;
	}while(e!=v->edge);
	cutOutPieSlice(start,end,mesh,effectedVertices);
}

void cgx::cutOutBridge(Edge* e,Mesh* mesh,VertexSelection* effectedVertices)
{
	//The understand was this function does,
	//study the bridge structure
	Vertex* vertexOtherside=e->next->next->vertex;
	Edge* eOtherside=e->next->next->next;
	Edge* middle=eOtherside;
	while(middle->vertex!=e->vertex)
		middle=middle->pair->next;
	if(middle!=eOtherside)
		cutOutPieSlice(eOtherside,before(middle),mesh,effectedVertices);
	if(middle!=e)
		cutOutPieSlice(middle->next,e,mesh,effectedVertices);
	closeMiniHole(middle,mesh,effectedVertices);
}

void cgx::deleteSelection(VertexSelection* selection,Mesh* mesh,VertexSelection* effectedVertices)
{
	while(selection->size()!=0)
	{
		cutOutVertex(selection->getRootVal(),mesh,effectedVertices);
		selection->del(selection->getRootVal());
	}
}

bool cgx::meshSegmentExceedsSizeX(Vertex* v,VertexSelection* vs,const unsigned int x)
{	
	bool res=false;
	if(vs->addUnique(v))
	{	
		if(vs->size() > x)
			return true;
		Edge* e=v->edge;
		do{
			res = meshSegmentExceedsSizeX(e->vertex,vs,x);
			e=e->pair->next;
		}while(!res && e!=v->edge);
	}
	return res;
}

unsigned int cgx::meshSegmentSize(Vertex* v,VertexNeigborhood* vn)
{	
	unsigned int res=0;
	vn->clean();
	vn->addVertex(v);
	do{
		res+=vn->numOfVertices;
		vn->expandNeigborhood();
	}while(vn->numOfVertices>0);
	return res;
}

void cgx::laplaceSmoothing(Vertex* v,const Vec* normal,const Vec* laplace,const float smoothingDegree,Vec* tp0)
{
	add(v,scale(subt(laplace,scale(normal,scalarProduct(laplace,normal),tp0),tp0),smoothingDegree,tp0),v);
}

void cgx::cut(EdgePath* cutPath,Mesh* mesh,VertexSelection* effectedVertices)
{
	assert(cutPath->cycle() || 
		   (atBorder(cutPath->first->val->pair->vertex) && atBorder(cutPath->last->val->vertex)));
	//Determine smaller cut
	unsigned int above=0;
	unsigned int beneath=0;
	unsigned int* currentCounter;
	if(cutPath->cycle())
	{
		fast::DList<Edge*> * start=cutPath->first;
		fast::DList<Edge*> * current=start;
		do{
			currentCounter=&beneath;
			Edge* e=current->val->pair->next;
			do
			{
				if(e->pair==current->previous->val)
					currentCounter=&above;
				else
					(*currentCounter)++;
				e=e->pair->next;
			}while(e!=current->val);
			current=current->next;
		}while(current!=start);
		cut(cutPath,above<=beneath,mesh,effectedVertices);
	}
	else
	{
		Edge* start=outGoingBorder(cutPath->first->val->pair->vertex);
		Edge* e;
		fast::DList<Edge*> * current=cutPath->first;
		while(current!=NULL)
		{
			currentCounter=&above;
			e=start;
			do{
				if(e==current->val)
					currentCounter=&beneath;
				else
					(*currentCounter)++;
				e=e->pair->next;
			}while(e!=start);	
			start=current->val->next;
			current=current->next;
		}
		start=outGoingBorder(cutPath->last->val->vertex);
		e=start;
		do{
			if(e==cutPath->last->val)
				currentCounter=&above;
			else
				(*currentCounter)++;
			e=e->pair->next;
		}while(e!=start);
		cut(cutPath,above<=beneath,mesh,effectedVertices);
	}
}

void cgx::cut(EdgePath* cutPath,bool above,Mesh* mesh,VertexSelection* effectedVertices)
{
	if(cutPath->cycle())
	{
		fast::DList<Edge*> * start=cutPath->first;
		fast::DList<Edge*> * current=start;
		Edge* cutStart;
		Edge* cutEnd;
		if(above)
		{
			do{
				if(current->next->val!=current->val->next)
				{
					cutStart=current->val->next;
					cutEnd=cutStart;
					do cutEnd=cutEnd->pair->next;
					while(cutEnd->pair->next!=current->next->val);
					cutEnd=cutEnd->pair;
					cgx::cutOutPieSlice(cutStart,cutEnd,mesh,effectedVertices);
				}
				current=current->next;
			}while(current!=start);
		}
		else
		{
			do{
				if(current->previous->val->pair!=current->val->pair->next)
				{
					cutStart=current->val->pair->next;
					cutEnd=cutStart;
					do cutEnd=cutEnd->pair->next;
					while(cutEnd->pair->next!=current->previous->val->pair);
					cutEnd=cutEnd->pair;
					cgx::cutOutPieSlice(cutStart,cutEnd,mesh,effectedVertices);
				}
				current=current->next;
			}while(current!=start);
		}
	}
	else
	{
		Edge* end;
		Edge* start;
		Edge* e;
		fast::DList<Edge*> * current=cutPath->first;
		if(above)
		{
			end=inGoingBorder(cutPath->last->val->vertex);
			start=outGoingBorder(cutPath->first->val->pair->vertex);
			fast::DList<Edge*> * current=cutPath->first;
			while(current!=NULL)
			{
				e=start;
				if(start!=current->val)
				{
					do	e=e->pair->next;
					while(e->pair->next!=current->val);	
					e=e->pair;
					cgx::cutOutPieSlice(start,e,mesh,effectedVertices);
				}
				start=current->val->next;
				current=current->next;
			}
			if(cutPath->last->val!=end)
				cgx::cutOutPieSlice(cutPath->last->val->next,end,mesh,effectedVertices);
		}
		else
		{
			end=inGoingBorder(cutPath->first->val->pair->vertex);
			start=outGoingBorder(cutPath->last->val->vertex);
			fast::DList<Edge*> * current=cutPath->last;
			while(current!=NULL)
			{
				e=start;
				if(start!=current->val->pair)
				{
					do	e=e->pair->next;
					while(e->pair->next!=current->val->pair);	
					e=e->pair;
					cgx::cutOutPieSlice(start,e,mesh,effectedVertices);
				}
				start=current->val->pair->next;
				current=current->previous;
			}
			if(cutPath->first->val->pair!=end)
				cgx::cutOutPieSlice(cutPath->first->val->pair->next,end,mesh,effectedVertices);			
		}
	}
}

class FlipEdge{
public:
	FlipEdge(){}
	~FlipEdge(){}	
	static int compF(const FlipEdge a,const FlipEdge b)
	{
			return a.e<b.e ? -1 : (a.e==b.e ? 0 : -1);
	};
	Edge* e;
	Edge* edgeBefore;
	Vertex* vertexBefore;
};

void flipEdge(FlipEdge* fE,fast::RedBlackTree<FlipEdge>* flipEdgeTree)
{
	Edge* e=fE->e;
	e->next=fE->edgeBefore;
	e->vertex=fE->vertexBefore;
	flipEdgeTree->del(*fE);
}

void cgx::flip(Vertex* initialVertex,VertexNeigborhood* vn)
{
	if(!valenceZeroVertex(initialVertex))
	{
		unsigned int i;
		FlipEdge fE;
		vn->clean();
		vn->addVertex(initialVertex);

		//could be optimised, when often used
		fast::RedBlackTree<FlipEdge>* new_flipEdge=new fast::RedBlackTree<FlipEdge>();
		fast::RedBlackTree<FlipEdge>* current_flipEdge=new fast::RedBlackTree<FlipEdge>();
		fast::RedBlackTree<FlipEdge>* previous_flipEdge=new fast::RedBlackTree<FlipEdge>();
		new_flipEdge->setComparator(FlipEdge::compF);
		current_flipEdge->setComparator(FlipEdge::compF);
		previous_flipEdge->setComparator(FlipEdge::compF);
		Edge* e;
		do
		{
			i=0;
			while(i < vn->numOfVertices)
			{
				e=vn->vertices[i]->edge;
				do{
					fE.e=e;
					if(new_flipEdge->search(fE)==NULL &&
					   current_flipEdge->search(fE)==NULL &&
					   previous_flipEdge->search(fE)==NULL)
					{
						fE.edgeBefore=before(e);
						fE.vertexBefore=e->pair->vertex;
						new_flipEdge->addUnique(fE);

						fE.e=e->pair;
						fE.edgeBefore=before(fE.e);
						fE.vertexBefore=e->vertex;
						new_flipEdge->addUnique(fE);
					}
					e=e->pair->next;
				}while(e!=vn->vertices[i]->edge);
				i++;
			}
			while(previous_flipEdge->size()!=0)
				flipEdge(&previous_flipEdge->getRootVal(),previous_flipEdge);
			std::swap(new_flipEdge,current_flipEdge);
			std::swap(new_flipEdge,previous_flipEdge);
			vn->expandNeigborhood();
			i=0;
			while(i < vn->numOfPreviousVertices)
			{
				vn->previousVertices[i]->edge=vn->previousVertices[i]->edge->pair;
				i++;
			}
		}while(vn->numOfVertices>0);
		while(previous_flipEdge->size()!=0)
				flipEdge(&previous_flipEdge->getRootVal(),previous_flipEdge);
		while(current_flipEdge->size()!=0)
				flipEdge(&current_flipEdge->getRootVal(),current_flipEdge);
		delete new_flipEdge;
		delete current_flipEdge;
		delete previous_flipEdge;
	}

}

void cgx::clean(VertexSelection* selection,Mesh* mesh,VertexFilters* vf)
{
	Vertex* v;
	bool filtered;
	while(selection->size()!=0)
	{
		filtered=false;
		v=selection->getRootVal();
		vf->initIterator();
		while(!filtered && vf->current())
		{	
			filtered=vf->current()->filter(v,mesh,selection);
			vf->next();
		}
		if(!filtered)
			selection->del(v);
	}
}

class Locals
{
public:
	static VertexSelection locals;
	static bool isLocal(Vertex* v)
	{
		return locals.search(v)!=NULL;
	}
};

VertexSelection Locals::locals;

void cgx::clean_local(VertexSelection* selection,Mesh* mesh,VertexFilters* vf)
{
	Locals::locals.copy(selection);
	bool filtered;
	//This avoids a filter cascade, only initial values go thru the filter. A new value will immediately added to passedFilters
	fast::RedBlackNode<Vertex*>* n=selection->search(Locals::isLocal);
	Vertex* v=n?n->val:NULL;
	while(v)
	{
		filtered=false;
		vf->initIterator();
		while(!filtered && vf->current())
		{
			filtered=vf->current()->filter(v,mesh,selection);
			vf->next();
		}
		if(!filtered)
			selection->del(v);
		n=selection->search(Locals::isLocal);
		v=n?n->val:NULL;
	}
}

void cgx::cleanMesh(Mesh* mesh,VertexFilters* vf)
{
	VertexSelection vs;
	vs.nodeSource.resizePool(mesh->getNumOfVertices());
	vs.nodeSource.resizeDeletedStack(mesh->getNumOfVertices());
	unsigned int i=0;
	while(i<mesh->getNumOfVertices())
		vs.add(mesh->getVertex(i++));
	clean(&vs,mesh,vf);
}

unsigned int cgx::seperateMeshSegments(Mesh* mesh,VertexNeigborhood* vn)
{
	unsigned int res=0,i=0;
	VertexSelection vs;
	vs.nodeSource.resizePool(mesh->getNumOfVertices());
	while(i<mesh->getNumOfVertices())
		vs.add(mesh->getVertex(i++));
	while(vs.size()!=0)
	{
		res++;
		vn->clean();
		vn->addVertex(vs.getRootVal());
		do{
			for(unsigned int j=0;j<vn->numOfVertices;j++)
				vs.del(vn->vertices[j]);
			vn->expandNeigborhood();
		}while(vn->numOfVertices!=0);
	}
	return res;
}

void cgx::cutOutTVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices)
{
	if((getValence(v)==3 &&	
		   v->edge->face!=NULL &&
		   v->edge->pair->next->face!=NULL && 
		   v->edge->pair->next->pair->next->face!=NULL))
		edgeCollapse(v->edge,mesh,effectedVertices);
}

void cgx::cutOutValenceFourVertex(Edge* e,Mesh* mesh,VertexSelection* effectedVertices)
{
	if(getValence(e->pair->vertex)==4 &&	
			e->face!=NULL &&
			e->pair->next->face!=NULL && 
			e->pair->next->pair->next->face!=NULL &&
			e->pair->next->pair->next->pair->next->face!=NULL)
		edgeCollapse(e,mesh,effectedVertices);
}

bool VertexFiltertwoBorderEdge::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	Edge* e=twoBorderEdge(v);
	if(e)
	{
		do
		{
			cutOutPieSlice(e,e->pair,mesh,selection);
			e=twoBorderEdge(v);
		}while(e);
		return true;
	}
	return false;
};

bool VertexFilterValenceZeroVertex::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	if(valenceZeroVertex(v))
	{
		cutOutVertex(v,mesh,selection);
		return true;
	}
	return false;
};

bool VertexFilterValenceTwoVertex::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
		if(valenceTwoVertex(v))
		{
			cutOutVertex(v,mesh,selection);
			return true;
		}
		return false;
};

bool VertexFilterMultipleBorderVertex::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	if(multipleBorderVertex(v))
	{
		if(cutNotFix)
			cutMultipleBorderVertex(v,mesh,selection);
		else
		{
			Edge* inGoing=inGoingBorder(v);
			fixMultipleBorderVertex(v,inGoing,inGoingBorderNext(v,inGoing),mesh,selection,&t1,&t2,&t3);
		}
		return true;
	}
	return false;
};

bool VertexFilterMiniHole::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	bool closedOne=false;
	do
	{
		e=inGoingBorder(v);
		while(e!=NULL && !miniHole(e))
			e=inGoingBorderNext(v,e);
		if(e!=NULL)
		{
			closeMiniHole(e,mesh,selection);
			closedOne=true;
		}
	}
	while(e!=NULL);
	if(closedOne)
		return true;
	return false;
};

bool VertexFilterBridge::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	bool bridgeCut=false;
	do
	{
		e=inGoingBorder(v);
		while(e!=NULL && !bridgeRailing(e) && !bridgeRailing(before(e)))
			e=inGoingBorderNext(v,e);
		if(e!=NULL)
		{
			if(bridgeRailing(e))
			   cutOutBridge(e,mesh,selection);
			else
			   cutOutBridge(before(e),mesh,selection);
			bridgeCut=true;
		}
	}
	while(e!=NULL);
	if(bridgeCut)
		return true;
	return false;
};

bool VertexFilterBottleneck::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	if(bottleneck(v,e1,e2,e3))
	{
		cutOutBottleneck(e1,e2,e3,mesh,&vs,selection);
		return true;
	}
	return false;
};

bool VertexFilterOversizedTriangles::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	bool deletedEdge=false;
	if(!valenceZeroVertex(v))
	{
		//averageEdgeLength=(averageEdgeLength*999+cgx::averageEdgeLength(v,&temp))/1000;
		Edge* e=v->edge;
		do
		{
			if(edgeLength(e,&temp)>averageEdgeLength*numOfAveBeforeDel)
			{	
				cutOutPieSlice(e,e->pair,mesh,selection);
				if(valenceZeroVertex(v))
					break;
				e=v->edge;
				deletedEdge=true;
			}
			e=e->pair->next;
		}while(e!=v->edge);
	}
	return deletedEdge;
};

bool VertexFilterOptimizeValences::filter(Vertex* v,Mesh* mesh,VertexSelection* selection)
{
	Edge* res=NULL;
	a=getValence(v);
	if(a>2)
	{   
		if(!useEdgeSwapOnly &&
		   (a==3 || a==4 && v->edge->pair->next->pair->next->pair->next->face!=NULL) &&
		   v->edge->face!=NULL &&
		   v->edge->pair->next->face!=NULL && 
		   v->edge->pair->next->pair->next->face!=NULL)
	   {
			if(a==3 && getValence(v->edge->vertex)!=3)//Destroying T-Vertex if it is not a tetrahedron
			{
				edgeCollapse(v->edge,mesh,selection);
				return true;
			}
			else /*a==4 &&*/
			{
				a=getValence(v->edge->vertex);
				b=getValence(v->edge->pair->next->vertex);
				c=getValence(v->edge->pair->next->pair->next->vertex);
				d=getValence(v->edge->pair->next->pair->next->pair->next->vertex);

				oldNum=4;//= (4-6)!^2
				q=a-6;q*=q;oldNum+=q;
				q=b-6;q*=q;oldNum+=q;
				q=c-6;q*=q;oldNum+=q;
				q=d-6;q*=q;oldNum+=q;

				newNum=0;
				q=a-6;q*=q;newNum+=q;
				q=b-7;q*=q;newNum+=q;
				q=c-6;q*=q;newNum+=q;
				q=d-7;q*=q;newNum+=q;
				best=0;

				if(oldNum>newNum &&
				   noConnection(v->edge->vertex,v->edge->pair->next->pair->next->vertex))
				{
					best=oldNum-newNum;
					res=v->edge;
				}

				newNum=0;
				q=a-7;q*=q;newNum+=q;
				q=b-6;q*=q;newNum+=q;
				q=c-7;q*=q;newNum+=q;
				q=d-6;q*=q;newNum+=q;

				if(oldNum>newNum && 
				   best<oldNum-newNum &&
				   noConnection(v->edge->pair->next->pair->next->vertex,v->edge->pair->next->pair->next->pair->next->vertex))
						res=v->edge->pair->next;

				if(res)
				{
					edgeCollapse(res,mesh,selection);
					return true;
				}

			}
	   }
	   else
	   {
			Edge* e=v->edge;
			best=0;
			a_oldVal=a-6;a_oldVal*=a_oldVal;
			a_newVal=a-7;a_newVal*=a_newVal;
			c=getValence(e->vertex);
			e=e->pair->next;
			d=getValence(e->vertex);
			do
			{
				b=c;
				c=d;
				d=getValence(e->pair->next->vertex);
				if(e->face!=NULL && e->pair->face!=NULL)
				{		
					oldNum=a_oldVal;
					q=b-6;q*=q;oldNum+=q;
					q=c-6;q*=q;oldNum+=q;
					q=d-6;q*=q;oldNum+=q;
					newNum=a_newVal;
					q=b-5;q*=q;newNum+=q;
					q=c-7;q*=q;newNum+=q;
					q=d-5;q*=q;newNum+=q;
					if(oldNum>newNum && best<oldNum-newNum && cgx::noConnection(e->next->vertex,e->pair->next->vertex))
					{
						best=oldNum-newNum;
						res=e;
					}
				}
				e=e->pair->next;
			}while(e!=v->edge);
			if(res)
				edgeSwap(res,selection);
			//The filter never return true because, their could be a cycle of valence improvements, that would cause an infinite loop in the filtering
	   }
	}
	return false;
}
