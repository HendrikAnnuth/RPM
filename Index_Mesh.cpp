#include "Index_Mesh.h"
#include <cstdint>
#include <cstring>

using namespace cgx;
#define SET_VEC(ptr,index,input) ptr[index*3]=(input).x;ptr[index*3+1]=(input).y;ptr[index*3+2]=(input).z

#define SET_UV_COORDINATE(ptr,index,input) ptr[index*2]=(input).u;ptr[index*2+1]=(input).v

#define SET_COLOR(ptr,index,input) ptr[index*4]=(input).r;ptr[index*4+1]=(input).g;ptr[index*4+2]=(input).b;ptr[index*4+3]=(input).a

#define SET_FACE_INDICES(ptr,index,a,b,c) ptr[index*3]=a;ptr[index*3+1]=b;ptr[index*3+2]=c


bool Index_Mesh::hasUV_Coordinates()
{
	return uv_coords != nullptr;
}

bool Index_Mesh::hasNormals()
{
	return normals != nullptr;
}

bool Index_Mesh::hasColor()
{
	return colors != nullptr;
}

Index_Mesh::Index_Mesh(void)
{
	numOfVertices = 0;
	numOfFaces = 0;
	numOfCopies = 0;

	colors = nullptr;
	vertices = nullptr;
	normals = nullptr;
	uv_coords = nullptr;
	uv_vertex_copies = nullptr;
	indices = nullptr;
}

Index_Mesh::~Index_Mesh(void)
{
	clean();
}

Index_Mesh* Index_Mesh::copy()
{
	Index_Mesh* res = new Index_Mesh();

	res->vertices = new float[numOfVertices * 3];
	memcpy(res->vertices, vertices, numOfVertices * 3 * sizeof(float));
	res->numOfVertices = numOfVertices;

	res->indices = new unsigned int[numOfFaces * 3];
	memcpy(res->indices, indices, numOfFaces * 3 * sizeof(unsigned int));
	res->numOfFaces = numOfFaces;

	if (hasNormals())
	{
		res->normals = new float[numOfVertices * 3];
		memcpy(res->normals, normals, numOfVertices * 3 * sizeof(float));
	}

	if (hasColor())
	{
		res->colors = new unsigned char[numOfVertices * 4];
		memcpy(res->colors, colors, numOfVertices * 4 * sizeof(unsigned char));
	}

	if (hasUV_Coordinates())
	{
		res->uv_coords = new float[numOfVertices * 2];
		memcpy(res->uv_coords, uv_coords, numOfVertices * 2 * sizeof(float));
		res->uv_vertex_copies = new unsigned int[numOfCopies * 2];
		memcpy(res->uv_vertex_copies, uv_vertex_copies, numOfCopies * 2 * sizeof(unsigned int));
		res->numOfCopies = numOfCopies;
	}
	return res;
}

void Index_Mesh::clean()
{
	delete[] vertices;
	delete[] colors;
	delete[] normals;
	delete[] indices;
	delete[] uv_coords;
	delete[] uv_vertex_copies;

	numOfVertices = 0;
	numOfFaces = 0;
	numOfCopies = 0;

	colors = nullptr;
	vertices = nullptr;
	normals = nullptr;
	uv_coords = nullptr;
	uv_vertex_copies = nullptr;
	indices = nullptr;
}

void Index_Mesh::clean_normals()
{
	delete[] normals;
	normals = nullptr;
}

void Index_Mesh::clean_color()
{
	delete[] colors;
	colors = nullptr;
}

class FacePosPlace {
public:
	FacePosPlace() {};
	unsigned int originalPlace;
	float pos;
};

int sortFacePosPlace(const void* a, const void* b)
{
	return ((FacePosPlace*)a)->pos < ((FacePosPlace*)b)->pos ? -1 : (((FacePosPlace*)a)->pos == ((FacePosPlace*)b)->pos ? 0 : 1);
}