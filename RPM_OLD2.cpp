// RPM.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//
#include "Index_Mesh.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include<algorithm>
#include<unordered_set>
#include <vector>       // std::vector
#include <bitset>  
#ifdef _DEBUG
#include <sstream> 
#endif // DEBUG

using namespace std;
using namespace cgx;

void endian_swap(void* a)
{
	unsigned int* x = (unsigned int*)a;
	*x = (*x >> 24) |
		((*x << 8) & 0x00FF0000) |
		((*x >> 8) & 0x0000FF00) |
		(*x << 24);
}


Index_Mesh* load_ply(string pathName)
{

	unsigned int index1, index2, index3;
	ifstream mesh_stream(pathName, ios::in | ios::binary);
	Index_Mesh* mesh = new Index_Mesh();
	bool color = false, uv_Coordinat = false, normal = false;
	unsigned char dummy;
	char c[1000];
	enum { LE, BE, ASC } plyType;
	std::string content;

	if (mesh_stream)
	{
		do
		{
			mesh_stream.getline(c, 1000, '\n');
			content = std::string(c);
			if (content.find(std::string("element vertex ")) != -1)
			{
				content = content.substr(std::string("element vertex ").length() - 1);
				mesh->numOfVertices = (unsigned int)atoi(content.c_str());
			}
			if (content.find(std::string("property float nx")) != -1)
				normal = true;
			if (content.find(std::string("property uchar red")) != -1 ||
				content.find(std::string("property uchar r")) != -1)
				color = true;
			if (content.find(std::string("property float texture_u")) != -1 ||
				content.find(std::string("property float u")) != -1 ||
				content.find(std::string("property float s")) != -1)
				uv_Coordinat = true;
			if (content.find(std::string("element face ")) != -1)
			{
				content = content.substr(std::string("element faces ").length() - 1);
				mesh->numOfFaces = (unsigned int)atoi(content.c_str());
			}
			if (content.find(std::string("format ")) != -1)
			{
				if (content.find(std::string("binary_big_endian")) != -1)
					plyType = BE;
				if (content.find(std::string("ascii")) != -1)
					plyType = ASC;
				if (content.find(std::string("binary_little_endian")) != -1)
					plyType = LE;
			}
		} while (content.find(std::string("end_header")) == -1);
		mesh->vertices = new float[mesh->numOfVertices * 3];
		if (normal)
			mesh->normals = new float[mesh->numOfVertices * 3];
		if (uv_Coordinat)
			mesh->uv_coords = new float[mesh->numOfVertices * 2];
		if (color)
			mesh->colors = new unsigned char[mesh->numOfVertices * 4];
		float x, y, z, nx, ny, nz, u, v;
		unsigned char r, g, b, a;

		mesh->indices = new unsigned int[mesh->numOfFaces * 3];

		for (unsigned int i = 0; i < mesh->numOfVertices; i++)//Reading in the vertex information based on the ply data type
		{
			switch (plyType)
			{
			case BE:
				mesh_stream.read((char*)&x, sizeof(float)); endian_swap(&x);
				mesh_stream.read((char*)&y, sizeof(float)); endian_swap(&y);
				mesh_stream.read((char*)&z, sizeof(float)); endian_swap(&z);
				if (normal)
				{
					mesh_stream.read((char*)&nx, sizeof(float)); endian_swap(&nx);
					mesh_stream.read((char*)&ny, sizeof(float)); endian_swap(&ny);
					mesh_stream.read((char*)&nz, sizeof(float)); endian_swap(&nz);
				}
				if (color)
				{
					mesh_stream.read((char*)&r, sizeof(unsigned char)); endian_swap(&r);
					mesh_stream.read((char*)&g, sizeof(unsigned char)); endian_swap(&g);
					mesh_stream.read((char*)&b, sizeof(unsigned char)); endian_swap(&b);
					mesh_stream.read((char*)&a, sizeof(unsigned char)); endian_swap(&a);
				}
				if (uv_Coordinat)
				{
					mesh_stream.read((char*)&u, sizeof(float)); endian_swap(&u);
					mesh_stream.read((char*)&v, sizeof(float)); endian_swap(&v);
				}
				break;
			case LE:
				mesh_stream.read((char*)&x, sizeof(float));
				mesh_stream.read((char*)&y, sizeof(float));
				mesh_stream.read((char*)&z, sizeof(float));
				if (normal)
				{
					mesh_stream.read((char*)&nx, sizeof(float));
					mesh_stream.read((char*)&ny, sizeof(float));
					mesh_stream.read((char*)&nz, sizeof(float));
				}
				if (color)
				{
					mesh_stream.read((char*)&r, sizeof(unsigned char));
					mesh_stream.read((char*)&g, sizeof(unsigned char));
					mesh_stream.read((char*)&b, sizeof(unsigned char));
					mesh_stream.read((char*)&a, sizeof(unsigned char));
				}
				if (uv_Coordinat)
				{
					mesh_stream.read((char*)&u, sizeof(float));
					mesh_stream.read((char*)&v, sizeof(float));
				}
				break;
			case ASC:
				mesh_stream >> x >> y >> z;
				if (normal)
					mesh_stream >> nx >> ny >> nz;
				if (color)
					mesh_stream >> r >> g >> b >> a;
				if (uv_Coordinat)
					mesh_stream >> u >> v;
				mesh_stream.getline(c, 1000);
				break;
			}
			mesh->vertices[i * 3] = x;
			mesh->vertices[i * 3 + 1] = y;
			mesh->vertices[i * 3 + 2] = z;
			if (normal)
			{
				mesh->normals[i * 3] = nx;
				mesh->normals[i * 3 + 1] = ny;
				mesh->normals[i * 3 + 2] = nz;
			}
			if (color)
			{
				mesh->colors[i * 4] = r;
				mesh->colors[i * 4 + 1] = g;
				mesh->colors[i * 4 + 2] = b;
				mesh->colors[i * 4 + 3] = a;
			}
			if (uv_Coordinat)
			{
				mesh->uv_coords[i * 2] = u;
				mesh->uv_coords[i * 2 + 1] = v;
			}
		}
		for (unsigned int i = 0; i < mesh->numOfFaces; i++)//Reading in the face information based on the ply data type
		{
			switch (plyType)
			{
			case BE:
				mesh_stream.read((char*)&dummy, sizeof(unsigned char));
				mesh_stream.read((char*)&index1, sizeof(int));
				mesh_stream.read((char*)&index2, sizeof(int));
				mesh_stream.read((char*)&index3, sizeof(int));
				endian_swap(&index1);
				endian_swap(&index2);
				endian_swap(&index3);
				break;
			case LE:
				mesh_stream.read((char*)&dummy, sizeof(unsigned char));
				mesh_stream.read((char*)&index1, sizeof(int));
				mesh_stream.read((char*)&index2, sizeof(int));
				mesh_stream.read((char*)&index3, sizeof(int));
				break;
			case ASC:
				mesh_stream >> dummy >> index1 >> index2 >> index3;
				mesh_stream.getline(c, 1000);
				break;
			}
			mesh->indices[i * 3] = index1;
			mesh->indices[i * 3 + 1] = index2;
			mesh->indices[i * 3 + 2] = index3;
		}
		mesh_stream.close();
	}
	else
	{
		//LOG(e_error,"File " << pathName.c_str() << " could not be loaded ");
		return NULL;
	}
	return mesh;
}

void getAB(const unsigned int v, const unsigned int faceIndexPosition, const Index_Mesh* mesh, unsigned int& a, unsigned int& b)
{
	if (mesh->indices[faceIndexPosition] == v)
	{
		a = mesh->indices[faceIndexPosition + 1];
		b = mesh->indices[faceIndexPosition + 2];
	}
	else if (mesh->indices[faceIndexPosition + 1] == v)
	{
		a = mesh->indices[faceIndexPosition + 2];
		b = mesh->indices[faceIndexPosition];
	}
	else
	{
		a = mesh->indices[faceIndexPosition];
		b = mesh->indices[faceIndexPosition + 1];
	}
}

void sortVertices(unsigned int& a, unsigned int& b, unsigned int& c)
{
	if (a > b || a > c )
	{
		unsigned int temp;
		if (b > c) {
			temp = a;
			a = c;
			c = a;
			b = temp;
		} 
		else
		{
			temp = a;
			a = b;
			b = c;
			c = temp;
		}
	}
}

void getFacesPerVertex(const Index_Mesh* const mesh, unsigned int*& jumpToFacesPerVertexEnd, unsigned int*& faceIndicesPerVertex)
{
	unsigned short* numberOfFacesPerVertex = new unsigned short[mesh->numOfVertices];
	memset(numberOfFacesPerVertex, 0, mesh->numOfVertices * sizeof(*numberOfFacesPerVertex));
	unsigned short currentCount;
	unsigned int index, index2, v1, v2, v3, a, b, c;

	//Count faces per vertex and find hightest number
	for (unsigned int faceIndex = 0; faceIndex < mesh->numOfFaces; ++faceIndex)
	{
		index = faceIndex * 3;
		++numberOfFacesPerVertex[mesh->indices[index]];
		++numberOfFacesPerVertex[mesh->indices[index + 1]];
		++numberOfFacesPerVertex[mesh->indices[index + 2]];
	}

	jumpToFacesPerVertexEnd = new unsigned int[mesh->numOfVertices];
	//Prefix Sum - determine jump in FacePerVertexArray
	for (unsigned int i = 0; i <= mesh->numOfVertices; ++i)
	{
		jumpToFacesPerVertexEnd[i] = numberOfFacesPerVertex[i] + (i==0 ? 0 : jumpToFacesPerVertexEnd[i-1]);
	}

	faceIndicesPerVertex = new unsigned int[mesh->numOfFaces * 3];
	//Determine FaceIndicesPerVertex and sort them
	for (unsigned int faceIndex = 0; faceIndex < mesh->numOfFaces; ++faceIndex)
	{
		index = faceIndex * 3;
		v1 = mesh->indices[index];
		v2 = mesh->indices[index + 1];
		v3 = mesh->indices[index + 2];
		faceIndicesPerVertex[jumpToFacesPerVertexEnd[v1] - numberOfFacesPerVertex[v1]--] = faceIndex;
		faceIndicesPerVertex[jumpToFacesPerVertexEnd[v2] - numberOfFacesPerVertex[v2]--] = faceIndex;
		faceIndicesPerVertex[jumpToFacesPerVertexEnd[v3] - numberOfFacesPerVertex[v3]--] = faceIndex;
	}
}

const unsigned short MAX_PATCH_SIZE = 256;
const int NUMBER_OF_SETUP_BLOCKS = 1;
const int INITIAL_PATCH_SIZE_WITH_ONE_VERTEX = 7 + NUMBER_OF_SETUP_BLOCKS; //1 Counters + 3 Outer-Vertices + 0 Innter-Vertices + 3 Connetced Patches + 1 Face


#ifdef _DEBUG
bool writeHelpText = false;
#endif // DEBUG

class Patch
{
public:
#ifdef _DEBUG
	string help;
#endif // DEBUG

	const static unsigned int UNDEFINED_PATCH = INT_MAX;
	unsigned short numOuterVertices;
	unsigned short numInnerVertices;
	unsigned short numNeighborPatches;
	unsigned short numFaces;
	unsigned int patchIndex;
	const unsigned int* outerVertices;
	const unsigned int* innerVertices;
	const unsigned int* faces;
	unsigned int* neighborPatches;



	Patch()
	{
		neighborPatches = new unsigned int[MAX_PATCH_SIZE];
	}

	~Patch()
	{
		delete[] neighborPatches;
	}

	void setup(const unsigned int& patchIndex, const unsigned int* const& jumpToPatchesStart, const unsigned int* const& patches)
	{
		setup(patchIndex, jumpToPatchesStart, patches, nullptr);
	}

	void setup(const unsigned int& patchIndex, const unsigned int* const& jumpToPatchesStart, const unsigned int* const& patches, const unsigned int* const& patchesIndexTranslation)
	{
		this->patchIndex = patchIndex;
		numOuterVertices = ((unsigned char*)(patches + jumpToPatchesStart[patchIndex]))[0] + getBit(patches + jumpToPatchesStart[patchIndex], 1);
		numInnerVertices = ((unsigned char*)(patches + jumpToPatchesStart[patchIndex]))[1] + getBit(patches + jumpToPatchesStart[patchIndex], 2);
		numNeighborPatches = ((unsigned char*)(patches + jumpToPatchesStart[patchIndex]))[2];
		numFaces = jumpToPatchesStart[patchIndex + 1] - jumpToPatchesStart[patchIndex] - (numOuterVertices + numInnerVertices + numNeighborPatches + NUMBER_OF_SETUP_BLOCKS);
		outerVertices = patches + jumpToPatchesStart[patchIndex] + NUMBER_OF_SETUP_BLOCKS;
		innerVertices = outerVertices + numOuterVertices;
		faces = innerVertices + numNeighborPatches + numInnerVertices;

		const unsigned int* oldIndicesOfNeighborPatches = innerVertices + numInnerVertices;
		unsigned int oldNumberOfNeighborPatches = numNeighborPatches;
		numNeighborPatches = 0;
		unsigned int translatedPatchIndex;
		for (unsigned int i = 0; i < oldNumberOfNeighborPatches; ++i)
		{
			if (patchesIndexTranslation != nullptr)
			{
				translatedPatchIndex = patchesIndexTranslation[oldIndicesOfNeighborPatches[i]];
				if (!containsNeighborPatch(translatedPatchIndex))
				{
					neighborPatches[numNeighborPatches++] = translatedPatchIndex;
				}
			}
			else
			{
				neighborPatches[numNeighborPatches++] = oldIndicesOfNeighborPatches[i];
			}
		}


#ifdef _DEBUG
		if (writeHelpText)
		{
			std::stringstream ss;
			ss << "Patch Index: " << patchIndex << std::endl;
			ss << "Outer Vertices " << numOuterVertices << ": ";
			for (unsigned int i = 0; i < numOuterVertices; ++i)
				ss << " " << outerVertices[i];
			ss << std::endl;
			ss << "Inner Vertices " << numInnerVertices << ": ";
			for (unsigned int i = 0; i < numInnerVertices; ++i)
				ss << " " << innerVertices[i];
			ss << std::endl;
			ss << "Neigbor Patches " << numNeighborPatches << ": ";
			for (unsigned int i = 0; i < numNeighborPatches; ++i)
				ss << " " << neighborPatches[i];
			ss << std::endl;
			ss << "Old Neigbor Patches " << oldNumberOfNeighborPatches << ": ";
			for (unsigned int i = 0; i < oldNumberOfNeighborPatches; ++i)
				ss << " " << oldIndicesOfNeighborPatches[i];
			ss << std::endl;
			ss << "Faces " << numFaces << ": ";
			for (unsigned int i = 0; i < numFaces; ++i)
				ss << " " << faces[i];
			ss << std::endl;
			help = ss.str();
		}
#endif // DEBUG
	}

	unsigned int setupBlock() const
	{
		return Patch::setSetupBlock(numOuterVertices, numInnerVertices, numNeighborPatches);
	}

	unsigned int size() const
	{
		return NUMBER_OF_SETUP_BLOCKS + numOuterVertices + numInnerVertices + numNeighborPatches + numFaces;
	}

	bool containsNeighborPatch(const unsigned int& patchIndex) const
	{
		for (unsigned int i = 0; i < numNeighborPatches; ++i)
			if (neighborPatches[i] == patchIndex)
				return true;
		return false;
	}

	bool containsFace(const unsigned int& face) const
	{
		for (unsigned int i = 0; i < numFaces; ++i)
			if (faces[i] == face)
				return true;
		return false;
	}

	bool containsOuterVertex(const unsigned int& vertex) const
	{
		for (unsigned int i = 0; i < numOuterVertices; ++i)
			if (outerVertices[i] == vertex)
				return true;
		return false;
	}

	static void setToMerged(const unsigned int& patchIndex, const unsigned int* const& jumpToPatchesStart, unsigned int* const& patches)
	{
		setBit(patches + jumpToPatchesStart[patchIndex],0,true);
	}

	static bool isMerge(const unsigned int& patchIndex, const unsigned int* const& jumpToPatchesStart, const unsigned int* const& patches)
	{
		return getBit(patches + jumpToPatchesStart[patchIndex],0);
	}

	static void setBit(unsigned int* setupBlock, unsigned char pos, bool val)
	{
		((std::bitset<8>*)((unsigned char*)(setupBlock)+3))->set(pos, true);
	}

	static bool getBit(const unsigned int* const setupBlock, unsigned char pos)
	{
		return (*((std::bitset<8>*)((unsigned char*)(setupBlock) + 3)))[pos];
	}	

	static unsigned int setSetupBlock(const unsigned short& outer, const  unsigned short& inner, const  unsigned short& neigbor)
	{
		return setSetupBlock(outer, inner, neigbor, false);
	}

	static unsigned int setSetupBlock(const unsigned short& outer, const  unsigned short& inner, const  unsigned short& neigbor, const bool merged)
	{
		unsigned int res = 0;
		((unsigned char*)&res)[3] = 0;
		if (outer < MAX_PATCH_SIZE)
			((unsigned char*)&res)[0] = outer;
		else
		{
			((unsigned char*)&res)[0] = MAX_PATCH_SIZE - 1;
			Patch::setBit(&res, 1, true);
		}
		if (inner < MAX_PATCH_SIZE)
			((unsigned char*)&res)[1] = inner;
		else
		{
			((unsigned char*)&res)[1] = MAX_PATCH_SIZE - 1;
			Patch::setBit(&res, 2, true);
		}
		((unsigned char*)&res)[2] = neigbor;
		if (merged)
			Patch::setBit(&res, 3, true);
		return res;
	}

};


class MergedPatch : public Patch
{
public:

#ifdef _DEBUG
	string help;
#endif // DEBUG

	unsigned short numCommonOuterVertices;
	unsigned short numNewInnerVertices;

	unsigned int* outerVertices;
	unsigned int* newCommonOuterVertices;
	unsigned int* newInnerVertices;
	unsigned int* newNeighborPatches;

	MergedPatch()
	{
		newCommonOuterVertices = new unsigned int[MAX_PATCH_SIZE];
		outerVertices = newCommonOuterVertices;
		newInnerVertices = new unsigned int[MAX_PATCH_SIZE];
		newNeighborPatches = new unsigned int[MAX_PATCH_SIZE];
	}

	~MergedPatch()
	{
		delete [] newCommonOuterVertices;
		delete[] newInnerVertices;
		delete[] newNeighborPatches;
	}


public:
	void setup(Patch& a, Patch& b, const unsigned int* const& jumpToFacesPerVertexEnd, const unsigned int* const& faceIndicesPerVertex)
	{
		numOuterVertices = 0;
		numCommonOuterVertices = 0;
		numNewInnerVertices = 0;
		numFaces = a.numFaces + b.numFaces;
		numNeighborPatches = 0;
		for (unsigned int i = 0; i < a.numOuterVertices; ++i)
		{
			if (b.containsOuterVertex(a.outerVertices[i]))
				if (MergedPatch::isInsideTwoPatches(a.outerVertices[i], a, b, jumpToFacesPerVertexEnd, faceIndicesPerVertex))
				{
					newInnerVertices[numNewInnerVertices++] = a.outerVertices[i];
				}
				else
				{
					newCommonOuterVertices[numCommonOuterVertices++] = a.outerVertices[i];
				}
		}
		numOuterVertices = a.numOuterVertices + b.numOuterVertices - numNewInnerVertices * 2 - numCommonOuterVertices;
		numInnerVertices = a.numInnerVertices + b.numInnerVertices + numNewInnerVertices;

		unsigned int writeAdditionalOuterVertices = numCommonOuterVertices;
		if (!mergePatchIsBiggerMaxPatchSize())
		{
			for (unsigned int i = 0; i < a.numOuterVertices; ++i)
			{
				if (!containsNewInnerVertex(a.outerVertices[i]) && !containsNewCommonOuterVertex(a.outerVertices[i]))
				{
					outerVertices[writeAdditionalOuterVertices++] = a.outerVertices[i];
				}
			}
			for (unsigned int i = 0; i < b.numOuterVertices; ++i)
			{
				if (!containsNewInnerVertex(b.outerVertices[i]) && !containsNewCommonOuterVertex(b.outerVertices[i]))
				{
					outerVertices[writeAdditionalOuterVertices++] = b.outerVertices[i];
				}
			}
			unsigned int translatedPatchIndex;
			for (unsigned int i = 0; i < a.numNeighborPatches; ++i)
			{
				if (a.neighborPatches[i] != b.patchIndex)
				{
					if (!containsNeighborPatch(a.neighborPatches[i]))
					{
						newNeighborPatches[numNeighborPatches++] = a.neighborPatches[i];
					}
				}
			}
			for (unsigned int i = 0; i < b.numNeighborPatches; ++i)
			{
				if (b.neighborPatches[i] != a.patchIndex)
				{
					if (!containsNeighborPatch(b.neighborPatches[i]))
					{
						newNeighborPatches[numNeighborPatches++] = b.neighborPatches[i];
					}
				}
			}
		}


#ifdef _DEBUG
		if (writeHelpText)
		{
			std::stringstream ss;
			ss << "Patches Merged: " << a.patchIndex << " + " << b.patchIndex << std::endl;
			ss << "Outer Vertices " << numOuterVertices << ": ";
			for (unsigned int i = 0; i < numOuterVertices; ++i)
				ss << " " << outerVertices[i];
			ss << std::endl;
			ss << "  Common Outer Vertices " << numCommonOuterVertices << ": ";
			for (unsigned int i = 0; i < numCommonOuterVertices; ++i)
				ss << " " << newCommonOuterVertices[i];
			ss << std::endl;
			ss << "Inner Vertices " << numInnerVertices << ": ";
			for (unsigned int i = 0; i < a.numInnerVertices; ++i)
				ss << " " << a.innerVertices[i];
			for (unsigned int i = 0; i < b.numInnerVertices; ++i)
				ss << " " << b.innerVertices[i];
			ss << std::endl;
			ss << "  New Inner Vertices " << numNewInnerVertices << ": ";
			for (unsigned int i = 0; i < numNewInnerVertices; ++i)
				ss << " " << newInnerVertices[i];
			ss << std::endl;
			ss << "Neigbor Patches " << numNeighborPatches << ": ";
			for (unsigned int i = 0; i < numNeighborPatches; ++i)
				ss << " " << newNeighborPatches[i];
			ss << std::endl;
			ss << "Merged Neigbor Patches" << a.numNeighborPatches + b.numNeighborPatches << ": ";
			ss << "  a Neigbor Patches " << a.numNeighborPatches << ": ";
			for (unsigned int i = 0; i < a.numNeighborPatches; ++i)
				ss << " " << a.neighborPatches[i];
			ss << std::endl;
			ss << "  b Neigbor Patches " << b.numNeighborPatches << ": ";
			for (unsigned int i = 0; i < b.numNeighborPatches; ++i)
				ss << " " << b.neighborPatches[i];
			ss << std::endl;
			ss << "Faces " << numFaces << ": ";
			for (unsigned int i = 0; i < a.numFaces; ++i)
				ss << " " << a.faces[i];
			for (unsigned int i = 0; i < b.numFaces; ++i)
				ss << " " << b.faces[i];
			ss << std::endl;
			help = ss.str();
		}
#endif // DEBUG
	}

	unsigned int size() const
	{
		return NUMBER_OF_SETUP_BLOCKS + numOuterVertices + numInnerVertices + numNeighborPatches + numFaces;
	}

	unsigned int setupBlock() const
	{
		return Patch::setSetupBlock(numOuterVertices,numInnerVertices,numNeighborPatches);
	}

	bool containsNewCommonOuterVertex(const unsigned int& vertex) const
	{
		for (unsigned int i = 0; i < numCommonOuterVertices; ++i)
			if (newCommonOuterVertices[i] == vertex)
				return true;
		return false;
	}

	bool containsNewInnerVertex(const unsigned int& vertex) const
	{
		for (unsigned int i = 0; i < numNewInnerVertices; ++i)
			if (newInnerVertices[i] == vertex)
				return true;
		return false;
	}

	bool containsNeighborPatch(const unsigned int& patchIndex) const
	{
		for (unsigned int i = 0; i < numNeighborPatches; ++i)
			if (newNeighborPatches[i] == patchIndex)
				return true;
		return false;
	}

	static bool isInsideTwoPatches(const unsigned int& vertex, const Patch& a, const Patch& b, const unsigned int* const& jumpToFacesPerVertexEnd, const unsigned int* const& faceIndicesPerVertex)
	{
		for (unsigned int i = vertex == 0 ? 0 : jumpToFacesPerVertexEnd[vertex - 1]; i < jumpToFacesPerVertexEnd[vertex]; ++i)
		{
			if (!a.containsFace(faceIndicesPerVertex[i]) && !b.containsFace(faceIndicesPerVertex[i]))
				return false;
		}
		return true;
	}

public:
	bool mergePatchIsBiggerMaxPatchSize()
	{
		return numOuterVertices + numInnerVertices > MAX_PATCH_SIZE;
	}
};


struct PatchStats {
	float roundness;
	float numOfNeigbors;
	float numOfVertices;
	float ratioOuterToInnerVertices;
};

struct CommonStats {
	PatchStats* a;
	PatchStats* b;
	PatchStats average;
	PatchStats merge;
	float quality;
	float iteration;
	float finalQuality;
};

void getOneFacePatches(const Index_Mesh* const mesh, const unsigned int* const& jumpToFacesPerVertexEnd, const unsigned int* const& faceIndicesPerVertex, unsigned int*& jumpToPatchesStart, unsigned int*& patches)
{
	unsigned int currentPosition, index, index2, v1, v2, v3, a, b, c;
	unsigned int jumpIndexInFacesPerVertex;

	bool one;
	jumpToPatchesStart = new unsigned int[mesh->numOfFaces + 1];
	jumpToPatchesStart[0] = 0;
	for (unsigned int i = 1; i <= mesh->numOfFaces; ++i)
	{
		jumpToPatchesStart[i] = i * INITIAL_PATCH_SIZE_WITH_ONE_VERTEX;
	}

	patches = new unsigned int[mesh->numOfFaces * INITIAL_PATCH_SIZE_WITH_ONE_VERTEX];
	currentPosition = -1;
	unsigned int setupBlock = Patch::setSetupBlock(3, 0, 3);
	for (unsigned int faceIndex = 0; faceIndex < mesh->numOfFaces; ++faceIndex)
	{
		index2 = faceIndex * 3;
		v1 = mesh->indices[index2];
		v2 = mesh->indices[index2 + 1];
		v3 = mesh->indices[index2 + 2];

		patches[++currentPosition] = setupBlock;
		patches[++currentPosition] = v1;
		patches[++currentPosition] = v2;
		patches[++currentPosition] = v3;


		one = false;
		for (jumpIndexInFacesPerVertex = (v1 == 0 ? 0 : jumpToFacesPerVertexEnd[v1-1]); jumpIndexInFacesPerVertex < jumpToFacesPerVertexEnd[v1]; ++jumpIndexInFacesPerVertex)
		{
			if (faceIndex != faceIndicesPerVertex[jumpIndexInFacesPerVertex])
			{
				index = faceIndicesPerVertex[jumpIndexInFacesPerVertex] * 3;
				a = mesh->indices[index];
				b = mesh->indices[index + 1];
				c = mesh->indices[index + 2];
				if (a == v2 || b == v2 || c == v2 || a == v3 || b == v3 || c == v3)
				{
					if (one)
					{
						patches[++currentPosition] = faceIndicesPerVertex[jumpIndexInFacesPerVertex];
						break;
					}
					else
					{
						one = true;
						patches[++currentPosition] = faceIndicesPerVertex[jumpIndexInFacesPerVertex];
					}
				}
			}
		}
		for (jumpIndexInFacesPerVertex = v2 == 0 ? 0 : jumpToFacesPerVertexEnd[v2 - 1]; jumpIndexInFacesPerVertex < jumpToFacesPerVertexEnd[v2]; ++jumpIndexInFacesPerVertex)
		{
			if (faceIndex != faceIndicesPerVertex[jumpIndexInFacesPerVertex])
			{
				index = faceIndicesPerVertex[jumpIndexInFacesPerVertex] * 3;
				a = mesh->indices[index];
				b = mesh->indices[index + 1];
				c = mesh->indices[index + 2];

				if ((mesh->indices[index] == v3 || mesh->indices[index + 1] == v3 || mesh->indices[index + 2] == v3))
				{
					patches[++currentPosition] = faceIndicesPerVertex[jumpIndexInFacesPerVertex];
					break;
				}
			}
		}
		patches[++currentPosition] = faceIndex;
	}
}

void getEdge(const Index_Mesh *const mesh, const unsigned int& const index, const unsigned int& a, const unsigned int& b, const unsigned int& c, unsigned int& d, unsigned int& e, unsigned int& f)
{
	d = mesh->indices[index];
	e = mesh->indices[index + 1];
	f = mesh->indices[index + 2];
	if (f == a || f == b || f == c) {//if so, f is part of the edge
		if (e == a || e == b || e == c) {//if so, e is part of the edge
			d = f;//thus, d is not part of the edge
		}
		else
		{
			e = f;//thus, e is not part of the edge
		}
	}
}

float getRoundness(const Index_Mesh *const mesh, const MergedPatch& patch)
{
	unsigned int i,v;
	float mx, my, mz, x, y, z, mean, vari;
	mx = 0;
	my = 0;
	mz = 0;
	mean = 0;
	vari = 0;
	for ( i = 0; i < patch.numOuterVertices; ++i)
	{
		v = patch.outerVertices[i];
		mx += mesh->vertices[v * 3];
		my += mesh->vertices[v * 3 + 1];
		mz += mesh->vertices[v * 3 + 2];
	}
	mx /= patch.numOuterVertices;
	my /= patch.numOuterVertices;
	mz /= patch.numOuterVertices;
	for (i = 0; i < patch.numOuterVertices; ++i)
	{
		v = patch.outerVertices[i];
		x = mesh->vertices[v * 3] - mx;
		y = mesh->vertices[v * 3 + 1] - my;
		z = mesh->vertices[v * 3 + 2] - mz;
		mean += sqrt(x * x + y * y + z * z);
	}
	mean /= patch.numOuterVertices;
	for (i = 0; i < patch.numOuterVertices; ++i)
	{
		v = patch.outerVertices[i];
		x = mesh->vertices[v * 3] - mx;
		y = mesh->vertices[v * 3 + 1] - my;
		z = mesh->vertices[v * 3 + 2] - mz;
		vari += pow((sqrt(x * x + y * y + z * z) - mean),2.0);
	}
	vari /= patch.numOuterVertices;
	return 1.0 - sqrt(vari)/mean;
}


PatchStats calcStats(const Index_Mesh* const mesh, MergedPatch& patch)
{
	PatchStats res;
	res.roundness = getRoundness(mesh, patch);
	res.numOfNeigbors = patch.numNeighborPatches;
	res.numOfVertices = patch.numInnerVertices + patch.numOuterVertices;
	res.ratioOuterToInnerVertices = static_cast<float>(patch.numInnerVertices) / res.numOfVertices;
	return res;
}

const float  PI_F = 3.14159265358979f;

float calcQuality(PatchStats currentStats, int iteration, std::vector<float> weights) //, PatchStats mean, PatchStats stdDis)
{
	float roundness = currentStats.roundness;
	float numOfNeigbors = 4.0 / currentStats.numOfNeigbors;
	//float numOfVertices = (1.0 - (currentStats.numOfVertices - mean.numOfVertices * 2.0) / (stdDis.numOfVertices * 2.0));
	float numOfVertices = currentStats.numOfVertices / pow(2.0, iteration);
	float ratioOuterToInnerVertices = (currentStats.ratioOuterToInnerVertices / (2.0 / (sqrt(currentStats.numOfVertices / PI_F))));
	return (roundness * weights[0] + numOfNeigbors * weights[1] + numOfVertices * weights[2] + ratioOuterToInnerVertices * weights[3]);
}

void getPatches(const Index_Mesh* const mesh,
	const unsigned int* const& jumpToFacesPerVertexEnd,
	const unsigned int* const& faceIndicesPerVertex,
	unsigned int& numOfPatches,
	unsigned int*& jumpToPatchesStart,
	unsigned int*& patches,
	std::vector<float> weights)
{
	numOfPatches = mesh->numOfFaces;

	MergedPatch _merged;
	MergedPatch _best;
	MergedPatch* merged = &_merged;
	MergedPatch* best = &_best;
	Patch a, b;

	PatchStats currentStats;
	PatchStats mean;
	PatchStats standardDeviation;
	std::vector<std::vector<CommonStats>> stats;
	unsigned int* patchesIndexTranslationAllTheWay = new unsigned int[numOfPatches];

	unsigned int numOfPatches2;
	unsigned int* jumpToPatchesStart2 = new unsigned int[numOfPatches + 1];
	jumpToPatchesStart2[0] = 0;
	unsigned int* patches2 = new unsigned int[numOfPatches * INITIAL_PATCH_SIZE_WITH_ONE_VERTEX];
	unsigned int* patchesIndexTranslation = new unsigned int[numOfPatches];
	for (unsigned int i = 0; i < numOfPatches; ++i)
	{
		patchesIndexTranslationAllTheWay[i] = i;
		patchesIndexTranslation[i] = i;
	}
	unsigned int* patchesIndexTranslation2 = new unsigned int[numOfPatches];
	bool merge;
	float bestQuality, currentQuality;
	unsigned int bestPartner;
	bool forwardIteration = true;
	unsigned int currentPatch;
	unsigned int writeHead;
	unsigned int currentIteration = -1;
	do
	{
		++currentIteration;
		forwardIteration = !forwardIteration;
		merge = false;
		numOfPatches2 = 0;

		for (unsigned int i = 0; i < numOfPatches; ++i)
		{
			currentPatch = forwardIteration ? i : numOfPatches - i - 1;
			bestQuality = -1;
			bestPartner = Patch::UNDEFINED_PATCH;
			if (!Patch::isMerge(currentPatch, jumpToPatchesStart, patches))
			{
				a.setup(currentPatch, jumpToPatchesStart, patches, patchesIndexTranslation);
				for (unsigned int j = 0; j < a.numNeighborPatches; ++j)
				{
					if (!Patch::isMerge(a.neighborPatches[j], jumpToPatchesStart, patches))
					{
						b.setup(a.neighborPatches[j], jumpToPatchesStart, patches, patchesIndexTranslation);
						merged->setup(a, b, jumpToFacesPerVertexEnd, faceIndicesPerVertex);
						if (!merged->mergePatchIsBiggerMaxPatchSize())
						{
							currentStats = calcStats(mesh, *merged);
							currentQuality = calcQuality(currentStats, currentIteration,weights);
							if (currentQuality > bestQuality)
							{
								bestQuality = currentQuality;
								bestPartner = a.neighborPatches[j];
								std::swap(merged, best);
							}
						}
					}
				}
				if (bestPartner == Patch::UNDEFINED_PATCH)
				{
					Patch::setToMerged(currentPatch, jumpToPatchesStart, patches);
					patchesIndexTranslation2[currentPatch] = numOfPatches2;
					a.setup(currentPatch, jumpToPatchesStart, patches, patchesIndexTranslation);
					writeHead = jumpToPatchesStart2[numOfPatches2];
					patches2[writeHead] = a.setupBlock();
					writeHead += NUMBER_OF_SETUP_BLOCKS;
					std::memcpy(patches2 + writeHead, a.outerVertices, (a.numOuterVertices + a.numInnerVertices) * sizeof(unsigned int));
					writeHead += a.numOuterVertices + a.numInnerVertices;
					std::memcpy(patches2 + writeHead, a.neighborPatches, a.numNeighborPatches * sizeof(unsigned int));
					writeHead += a.numNeighborPatches;
					std::memcpy(patches2 + writeHead, a.faces, a.numFaces * sizeof(unsigned int));
					++numOfPatches2;
					jumpToPatchesStart2[numOfPatches2] = jumpToPatchesStart2[numOfPatches2 - 1] + a.size();
				}
				else
				{
					merge = true;
					Patch::setToMerged(currentPatch, jumpToPatchesStart, patches);
					Patch::setToMerged(bestPartner, jumpToPatchesStart, patches);
					patchesIndexTranslation2[currentPatch] = numOfPatches2;
					patchesIndexTranslation2[bestPartner] = numOfPatches2;
					a.setup(currentPatch, jumpToPatchesStart, patches, patchesIndexTranslation);
					b.setup(bestPartner, jumpToPatchesStart, patches, patchesIndexTranslation);
					writeHead = jumpToPatchesStart2[numOfPatches2];
					patches2[writeHead] = best->setupBlock();
					writeHead += NUMBER_OF_SETUP_BLOCKS;
					std::memcpy(patches2 + writeHead, best->outerVertices, best->numOuterVertices * sizeof(unsigned int));
					writeHead += best->numOuterVertices;
					std::memcpy(patches2 + writeHead, a.innerVertices, a.numInnerVertices * sizeof(unsigned int));
					writeHead += a.numInnerVertices;
					std::memcpy(patches2 + writeHead, b.innerVertices, b.numInnerVertices * sizeof(unsigned int));
					writeHead += b.numInnerVertices;
					std::memcpy(patches2 + writeHead, best->newInnerVertices, best->numNewInnerVertices * sizeof(unsigned int));
					writeHead += best->numNewInnerVertices;
					std::memcpy(patches2 + writeHead, best->newNeighborPatches, best->numNeighborPatches * sizeof(unsigned int));
					writeHead += best->numNeighborPatches;
					std::memcpy(patches2 + writeHead, a.faces, a.numFaces * sizeof(unsigned int));
					writeHead += a.numFaces;
					std::memcpy(patches2 + writeHead, b.faces, b.numFaces * sizeof(unsigned int));
					++numOfPatches2;
					jumpToPatchesStart2[numOfPatches2] = jumpToPatchesStart2[numOfPatches2 - 1] + best->size();
				}
			}
		}
		swap(patches, patches2);
		swap(jumpToPatchesStart, jumpToPatchesStart2);
		swap(numOfPatches, numOfPatches2);
		swap(patchesIndexTranslation, patchesIndexTranslation2);
		for (unsigned int i = 0; i < mesh->numOfFaces; ++i)
		{
			patchesIndexTranslationAllTheWay[i] = patchesIndexTranslation[i];
		}
	} while (merge);

	delete[] patches2;
	delete[] jumpToPatchesStart2;
	unsigned int* temp = new unsigned int[numOfPatches + 1];
	std::memcpy(temp, jumpToPatchesStart, (numOfPatches + 1) * sizeof(unsigned int));
	delete[] jumpToPatchesStart;
	jumpToPatchesStart = temp;
	temp = new unsigned int[jumpToPatchesStart[numOfPatches]];
	std::memcpy(temp, patches, jumpToPatchesStart[numOfPatches] * sizeof(unsigned int));
	delete[] patches;
	patches = temp;
}

void getReorderPosition(const unsigned int *const patches, unsigned int*& vertices, unsigned int*& faces)
{

}

Index_Mesh* rpm(Index_Mesh* mesh)
{
	unsigned int* jumpToFacesPerVertexEnd;
	unsigned int* faceIndicesPerVertex;
	getFacesPerVertex(mesh, jumpToFacesPerVertexEnd, faceIndicesPerVertex);
	unsigned int* patches;
	unsigned int* jumpToPatchesStart;
	getOneFacePatches(mesh, jumpToFacesPerVertexEnd, faceIndicesPerVertex, jumpToPatchesStart, patches);
	unsigned int numOfPatches;
	float best = 1000;
	for ( int i = 0; i < 10000; ++i)
	{
		for (int j = 0; j < 11 - i; ++j)
		{
			for (int h = 0; h < 11 - (i + j); ++h)
			{
				for (int k = 0; k < 11 - (i + j + h); ++k)
				{

					std::vector<float> weights = { 0.1f * i,0.1f *j ,0.1f * h,0.1f * k };

					numOfPatches = mesh->numOfFaces;
					unsigned int* temp = new unsigned int[numOfPatches + 1];
					std::memcpy(temp, jumpToPatchesStart, (numOfPatches + 1) * sizeof(unsigned int));
					unsigned int* temp2 = new unsigned int[jumpToPatchesStart[numOfPatches]];
					std::memcpy(temp2, patches, jumpToPatchesStart[numOfPatches] * sizeof(unsigned int));
					getPatches(mesh, jumpToFacesPerVertexEnd, faceIndicesPerVertex, numOfPatches, temp, temp2, weights);
					Patch a;
					double ave = 0;
					for (int g = 0; g < numOfPatches; ++g)
					{
						a.setup(g, temp, temp2);
						ave += 1.0 * a.numOuterVertices / (a.numOuterVertices + a.numInnerVertices);
					}
					ave /= numOfPatches;
					if (ave < best)
					{
						best = ave;
						std::cout << best << endl;
						std::cout << " " << 0.1f * weights[0] << " " << 0.1f * weights[1] << " " << 0.1f * weights[2] << " " << 0.1f * weights[3] << endl;
					}
				}
			}
		}
	}
	delete[] jumpToFacesPerVertexEnd;
	delete[] faceIndicesPerVertex;

	unsigned int* vertices;
	unsigned int* faces;
	//getReorderPosition(jumpInFacesPerPatch, faceIndicesPerPatch, vertices, faces);

	return nullptr;
}

int main()
{
	Index_Mesh* m = load_ply("C:/Users/Hendrik Annuth/Desktop/bunny.ply");
	rpm(m);

	std::cout << m->numOfVertices;
}


// Programm ausführen: STRG+F5 oder "Debuggen" > Menü "Ohne Debuggen starten"
// Programm debuggen: F5 oder "Debuggen" > Menü "Debuggen starten"

// Tipps für den Einstieg: 
//   1. Verwenden Sie das Projektmappen-Explorer-Fenster zum Hinzufügen/Verwalten von Dateien.
//   2. Verwenden Sie das Team Explorer-Fenster zum Herstellen einer Verbindung mit der Quellcodeverwaltung.
//   3. Verwenden Sie das Ausgabefenster, um die Buildausgabe und andere Nachrichten anzuzeigen.
//   4. Verwenden Sie das Fenster "Fehlerliste", um Fehler anzuzeigen.
//   5. Wechseln Sie zu "Projekt" > "Neues Element hinzufügen", um neue Codedateien zu erstellen, bzw. zu "Projekt" > "Vorhandenes Element hinzufügen", um dem Projekt vorhandene Codedateien hinzuzufügen.
//   6. Um dieses Projekt später erneut zu öffnen, wechseln Sie zu "Datei" > "Öffnen" > "Projekt", und wählen Sie die SLN-Datei aus.
