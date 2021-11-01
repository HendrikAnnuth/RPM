#ifndef HEADER_DEFINED_vertex_ops
#define HEADER_DEFINED_vertex_ops

#include "Vertex.h"
#include "VertexSelection.h"
#include "VertexNeigborhood.h"
#include "vertexFilters.h"
#include "EdgePath.h"
#include "EdgeFront.h"
#include <assert.h>
namespace cgx{


class Mesh;

/**
* @return shortest distance between face and point
*/
float distancePtToFace(const Vec* p,const Face* f,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4);
/**
* This funktion is used to check delaunayCriteria. The sphere center will lay on the face.
*
* @param center will be the center of the defined sphere
* @param radius will be the radius of the defined sphere
* @param the face that defined the sphere
*/
void sphereDefinedByFace(Vec& center,float& radius,const Face* f,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4);

/**
* @return the normal that direkts towards the border
*/
Vec* borderVertexNormal(const Vertex* v,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);
float averageEdgeLength(const Vertex* v,Vec* temp);
float lengthOfLongestEdge(const Face* f,Vec* temp);
float lengthOfLongestEdge(const Vertex* v,Vec* temp);
Edge* longestEdge(const Face* f,Vec* temp);
Edge* longestEdge(const Vertex* v,Vec* temp);
float lengthOfShortestEdge(const Vertex* v,Vec* temp);
Edge* shortestEdge(const Vertex* v,Vec* temp);
float faceQuality(const Face* f,Vec* temp1,Vec* temp2,Vec* temp3);
float triangleHeight(const Edge* e,Vec* temp1,Vec* temp2,Vec* temp3);
Vec* faceNormal(const Face* f,Vec* temp1,Vec* temp2,Vec* res);
/**
* @return normal that is calculatted as average of all surrounding faces
*/
Vec* vertexNormal(const Vertex* v,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);

float curvature(const Vertex* v,const Vec* v_normal,Vec* temp1,Vec* temp2);
float borderCurvature(const Vertex* v,const Vec* v_normal,Vec* temp1,Vec* temp2);
/**
* Value needed for the growing cell smoothing process
*/
Vec* laplaceOperand(const Vertex* v,Vec* temp,Vec* res);
Vec* borderLaplaceOperand(const Vertex* v,Vec* temp,Vec* res);
unsigned short getValence(const Vertex* v);
/**
* @return if there is no connection between v1 and v2 by an edge
*/
bool noConnection(const Vertex* v1,const Vertex* v2);
/**
* @param e a border edge
* @return the border edge pointing towards e
*/
Edge* before(const Edge* e);
/**
* @return the vector defined by e
*/
Vec* constructVector(const Edge* e,Vec* res);
float edgeLength(const Edge* e,Vec* temp);
float edgeQLength(const Edge* e,Vec* temp);
float surfaceArea(const Face* f,Vec* temp1,Vec* temp2,Vec* temp3);

/**
* @param v vertex that might has no edges
* @return v has no edges connected to it
*/
bool valenceZeroVertex(const Vertex* v);
bool valenceOneVertex(const Vertex* v);

/**
* This function is used to avoid, double connections 
* when performing edge collapes. It tells if to vertices
* connected by edge, have an aditional connection over two edges
* above the normal two <|> or if at border one |>
* @param edge the one connector edge
* @param toConnecter to the third vertex
* @param fromConnecter from the third vertex
* @return e there an aditionl two connection
*/
bool twoEdgeConnection(Edge* edge,Edge*& toConnecter,Edge*& fromConnecter);

/**
* @param v vertex that might be connectet to e
* @param e edge that might be connectet to v
* @return e is conncted with v
*/
bool connected(const Vertex* v,const Edge* e);

/**
* @param v the vertex we want to know of if it is connected to a border
* @return is v connected to a border
*/
Edge* atBorder(const Vertex* v);
/**
* @param e the edge we want to know of if it is connected to a border
* @return is e at a border
*/
bool atBorder(const Edge* e);

/**
* @param v vertex that is check for a faceless edge (edge that has no face on either side)
* @return one of v's faceless edges 
*/
Edge* twoBorderEdge(const Vertex* v);
/**
* @param v vertex that is check for a faceless edge (edge that has no face on either side)
* @return one of v's faceless edges 
*/
bool twoBorderEdge(const Edge* e);

/**
* This function identifies a structure that will be deleted by the ACO
* @return is a deletion worthy structur
*/
bool multipleBorderVertex(const Vertex* v);
unsigned int multipleBorderVertexSeparations(const Vertex* v);
/**
* This function identifies a structure that will be deleted by the ACO
* @return is a deletion worthy structur
*/
bool bridgeRailing(const Edge* e);
/**
* This function identifies a structure that will be deleted by the ACO
* @return is a deletion worthy structur
*/
bool valenceTwoVertex(const Vertex* v);
/**
* @return is a border that consists of three edges
*/
bool miniHole(const Edge* e);
Edge* connection(const Vertex* neighborhood,const Vertex* potentcialNeighbor);
int edgeDist(Vertex* v1,Vertex* v2,VertexNeigborhood* vn);
bool connection(Vertex* v1,Vertex* v2,VertexNeigborhood* vn);

bool connectionPath(EdgePath* path,Vertex* v1,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize);
bool connectionPath(Vertex* v1,EdgePath* path,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize);
bool connectionPath(EdgePath* path,Vertex* v2,EdgeFront* ef1,EdgeFront* ef2,unsigned int searchSpaceSize);

/**
* @param v the vertex that in case of a boarder vertex, we get the out going edge of
* @return the out going edge or if not border vertex NULL
*/
Edge* outGoingBorder(const Vertex* v);
/**
* @param v the vertex that in case of a boarder vertex, we get the in going edge of
* @return the in going edge or if not border vertex NULL
*/
Edge* inGoingBorder(const Vertex* v);
Edge* outGoingBorderNext(const Vertex* v,Edge* last);
Edge* inGoingBorderNext(const Vertex* v,Edge* last);

/**
* @param v the vertex we want to split
* @param vsE edge where the vertex split should take place
* @param posNewV position of the new vertex
* @param mesh the mesh in which the operation should take place
* @return the new created vertex
*/
Vertex* vertexSplit(Vertex* v,Edge* vsE,Vec* posNewV,Mesh* mesh,VertexSelection* effectedVertices);
/**
* @param e the edge that the resulting Error should be calculated for
* @return resulting Error when collapsing that Edge (high valances away from the optimum value 6 accure)
*/
unsigned int calcECError(const Edge* e);

/**
* @param v vertex that shoud be collapsed
* @return edge that can be collapsed without double connections, needle eyes, valenceTwoVertexs and creates the best valence, it can be NULL if there is no legal edge
*/
Edge* collapsibleEdge(const Vertex* v);
/**
* This function closes a hole concisting of three edges
* @param e one edge of that border
*/
void closeMiniHole(Edge* e,Mesh* mesh,VertexSelection* effectedVertices);
/**
* This function closes a hole of arbitray size
* @param e one edge of that border
* @param mesh the mesh in which the operation should take place
*/
bool closeHole(Edge* e,Mesh* mesh,VertexSelection* effectedVertices);
/**
* @param e edge that shoud be collapsed
* @param mesh the mesh in which the operation should take place
*/
void edgeCollapse(Edge* edgeToCollapse,Mesh* mesh,VertexSelection* effectedVertices);
/**
* This function swaps an edge
* @param swapEdge the edge that will be swaped
*/
void edgeSwap(Edge* swapEdge,VertexSelection* effectedVertices);
/**
* @param v the vertex that might is part of bottleneck
*         (bottleneck = The thinnest structure allowed in a manifold, perimeter=3 edges)
* @return is v a part of a bottleneck
*/
bool bottleneck(const Vertex* v,Edge*& e1,Edge*& e2,Edge*& e3);
/**
* This implements the concrete coalecing process
* @param v1 first of the vertices that should be connected
* @param v2 second of the vertices that should be connected
* @param mesh the mesh in which the operation should take place
* @return edge between the second and third vertex of the triangle (first is the one that might be a needle eye, after the operation)
*/
Edge* connectVertices(Vertex* v1,Vertex* v2,Mesh* mesh,VertexSelection* effectedVertices,Vec* tp0,Vec* tp1,Vec* tp2);
/**
* This implements the concrete coalecing process
* @param v1 the single vertex one the one side
* @param v2 first vertex on the other side
* @param v3 second vertex on the other side
* @param e2 edge between v1 and v2
* @param beforeV3 border edge pointig to v3
* @param mesh the mesh in which the operation should take place
*/
void createTriangle(Vertex* v1,Vertex* v2,Vertex* v3,Edge* e2,Edge* beforeV1,Mesh* mesh,VertexSelection* effectedVertices);
/**
* @param ne_v needle eye that should be fixed
* @param border1 edge pointig to ne_v from one boarder
* @param border2 edge pointig to ne_v from another boarder
* @param mesh the mesh in which the operation should take place
* @return edge between the second and third vertex of the triangle
*/
Edge* fixMultipleBorderVertex(Vertex* ne_v,Edge* border1,Edge* border2,Mesh* mesh,VertexSelection* effectedVertices,Vec* tp0,Vec* tp1,Vec* tp2);

void deleteSelection(VertexSelection* selection,Mesh* mesh,VertexSelection* effectedVertices);

/**
* This function cuts out a vertex an makes shure the mesh it valid after doing so.
* @param v vertex that should be deleted
*/
void cutOut(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices);

/**
* This function cuts out a vertex an makes shure the mesh it valid after doing so.
* @param v vertex that should be deleted
*/
void cutOutVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices);
/**
* This function cuts out a needle eye.
* @param v vertex that has the a needle eye, which should be deleted
*/
void cutMultipleBorderVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices);
/**
* This function cuts out a some faces in form of a pie slice.
* @param v vertex that has that slice, which should be deleted
*/
void cutOutPieSlice(Edge* start,Edge* end,Mesh* mesh,VertexSelection* effectedVertices);
/**
* This function cuts out a some faces in form of a pie slice.
* @param v vertex that has that slice, which should be deleted
*/
void cutOutBridge(Edge* e,Mesh* mesh,VertexSelection* effectedVertices);

/**
* This function cuts out T-Vertex (Valence == 3 and not a boundary Vertex).
* @param v T-Vertex
*/
void cutOutTVertex(Vertex* v,Mesh* mesh,VertexSelection* effectedVertices);

/**
* This function cuts out a vertex of valence 4.
* @param e edge that will be collapsed
*/
void cutOutValenceFourVertex(Edge* e,Mesh* mesh,VertexSelection* effectedVertices);


/**
* This function returns if the two triangles intersect. If
* one triangle is parallel and insight the other triangle will not be checked.
* @param f1 first triangle
* @param f2 secound triangle
* @return intersection has tested positiv
*/
bool trianglesIntersection(Face* f1,Face* f2,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4,Vec* temp5);

void laplaceSmoothing(Vertex* v,const Vec* normal,const Vec* laplace,const float smoothingDegree,Vec* tp0);

bool meshSegmentExceedsSizeX(Vertex* v,VertexSelection* vs,const unsigned int x);

unsigned int meshSegmentSize(Vertex* v,VertexNeigborhood* vn);

void cut(EdgePath* cutPath,bool above,Mesh* mesh,VertexSelection* effectedVertices);
void cut(EdgePath* cutPath,Mesh* mesh,VertexSelection* effectedVertices);

void flip(Vertex* initialVertex,VertexNeigborhood* vn);

void clean(VertexSelection* selection,Mesh* mesh,VertexFilters* vf);
void clean_local(VertexSelection* selection,Mesh* mesh,VertexFilters* vf);
void cleanMesh(Mesh* mesh,VertexFilters* vf);

unsigned int seperateMeshSegments(Mesh* mesh,VertexNeigborhood* vn);

//Unfinished
class VertexAttribute
{
  public:
	VertexAttribute(){
	}
	~VertexAttribute(){
	}
	virtual bool hasAttribute(Vertex* v)=0;
};

class VertexFiltertwoBorderEdge:
	public VertexFilter
{
public:
	VertexFiltertwoBorderEdge(){
	}
	~VertexFiltertwoBorderEdge(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterValenceZeroVertex:
	public VertexFilter
{
public:
	VertexFilterValenceZeroVertex(){
	}
	~VertexFilterValenceZeroVertex(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterValenceTwoVertex:
	public VertexFilter
{
public:
	VertexFilterValenceTwoVertex(){
	}
	~VertexFilterValenceTwoVertex(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterMultipleBorderVertex:
	public VertexFilter
{
protected:
	Vec t1,t2,t3;	
public:
	bool cutNotFix;
	VertexFilterMultipleBorderVertex(){
		this->cutNotFix=true;
	}
	VertexFilterMultipleBorderVertex(bool cutNotFix){
		this->cutNotFix=cutNotFix;
	}
	~VertexFilterMultipleBorderVertex(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterMiniHole:
	public VertexFilter
{
protected:
	Edge* e;
public:
	VertexFilterMiniHole(){
	}
	~VertexFilterMiniHole(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterBridge:
	public VertexFilter
{
protected:
	Edge* e;
public:
	VertexFilterBridge(){
	}
	~VertexFilterBridge(){
	}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterBottleneck:
	public VertexFilter
{
protected:
	VertexSelection vs;
	Edge *e1,*e2,*e3;
public:
	VertexFilterBottleneck(){}
	~VertexFilterBottleneck(){}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterOversizedTriangles:
	public VertexFilter
{
protected:
	Vec temp;
	float averageEdgeLength;
	unsigned int numOfAveBeforeDel;
public:
	VertexFilterOversizedTriangles(Mesh* mesh,unsigned int numOfAveBeforeDel)
	{
		this->numOfAveBeforeDel=numOfAveBeforeDel;
		unsigned int edgesToTest=mesh->getNumOfFaces();
		if(edgesToTest>1000000)
			edgesToTest/=100;
		else
			if(edgesToTest>1000)
				edgesToTest/=10;
		averageEdgeLength=0;
		unsigned int step=mesh->getNumOfFaces()/edgesToTest;
		for(unsigned int i=0;i<edgesToTest;i++)
			averageEdgeLength+=edgeLength(mesh->getFace(i*step)->edge,&temp);
		averageEdgeLength/=edgesToTest;
	}
	~VertexFilterOversizedTriangles(){}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

class VertexFilterOptimizeValences:
	public VertexFilter
{
protected:
	int q,a,b,c,d,a_oldVal,a_newVal,oldNum,newNum,best;
	bool useEdgeSwapOnly;
public:
	VertexFilterOptimizeValences(bool useEdgeSwapOnly)
	{
		this->useEdgeSwapOnly=useEdgeSwapOnly;
	}
	~VertexFilterOptimizeValences(){}
	virtual bool filter(Vertex* v,Mesh* mesh,VertexSelection* selection);
};

}
#endif