/**
 * @author  Hendrik Annuth
 * @version 1.0
 *
 * @section DESCRIPTION
 * 
 * This class represents an vertex.
 *
 * @member edge one of the edges comming out of this vertex
 * @member nature spezific quality of this vertex
 */

#ifndef HEADER_DEFINED_Vertex
#define HEADER_DEFINED_Vertex
#include "Vec.h"

namespace cgx{

typedef unsigned int indexType;

class Edge;
//A vertex can be labeled in some ways, to chenge its behavior and representation in the 
//mesh structure

class Vertex:public Vec {
public:
	Vertex(){};
	Vertex(float x,float  y,float  z,Edge* edge)
		:Vec(x,y,z)
	{
		this->edge=edge;
	}
	virtual ~Vertex()
	{}
	#if _DEBUG
		indexType id;
	#endif
	Edge* edge;
};

class Face{
public:
	Face(){};
	Face(Edge* edge)
	{
		this->edge=edge;
	}
	~Face()
	{}
	Edge* edge;
};

class Edge{
public:
	Edge(){};
	Edge(Face* face,Vertex* vertex,Edge* pair,Edge* next)
	{
		this->face=face;
		this->vertex=vertex;
		this->pair=pair;
		this->next=next;
	}
	~Edge()
	{
	}
	Face* face;
	Vertex* vertex;
	Edge* pair;
	Edge* next;
};
}
#endif