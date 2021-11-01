// RPM.cpp : Diese Datei enthält die Funktion "main". Hier beginnt und endet die Ausführung des Programms.
//
#include "Index_Mesh.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include<algorithm>
#include <vector>       // std::vector

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


void getVerticesAndFacesPerVertex(const Index_Mesh const * mesh, unsigned int*& jumpInVertices, unsigned int*& vertexIndicesPerVertex, unsigned int*& jumpInFacesPerVertex, unsigned int*& faceIndicesPerVertex)
{
	unsigned short* numberOfFacesOrEdgesPerVertex = new unsigned short[mesh->numOfVertices];
	memset(numberOfFacesOrEdgesPerVertex, 0, mesh->numOfVertices * sizeof(*numberOfFacesOrEdgesPerVertex));
	unsigned int numOfIndices = mesh->numOfFaces * 3;
	unsigned int highstNumOfFaces = 0;
	unsigned short currentCount;

	//Count faces per vertex and find hightest number
	for (unsigned int i = 0; i < numOfIndices; ++i)
	{
		currentCount = ++numberOfFacesOrEdgesPerVertex[mesh->indices[i]];
		if (currentCount > highstNumOfFaces)
			highstNumOfFaces = currentCount;
	}

	jumpInFacesPerVertex = new unsigned int[mesh->numOfVertices];

	//Prefix Sum - determine jump in FacePerVertexArray
	for (unsigned int i = 0; i < mesh->numOfVertices; ++i)
	{
		jumpInFacesPerVertex[i] = numberOfFacesOrEdgesPerVertex[i];
		if (i != 0)
			jumpInFacesPerVertex[i] += jumpInFacesPerVertex[i - 1];
	}

	faceIndicesPerVertex = new unsigned int[numOfIndices];

	unsigned int intIndex, v1, v2, v3;

	//Determine FaceIndicesPerVertex and sort them
	for (unsigned int faceIndex = 0; faceIndex < mesh->numOfFaces; ++faceIndex)
	{
		intIndex = faceIndex * 3;
		v1 = mesh->indices[intIndex];
		v2 = mesh->indices[intIndex + 1];
		v3 = mesh->indices[intIndex + 2];
		faceIndicesPerVertex[jumpInFacesPerVertex[v1] - numberOfFacesOrEdgesPerVertex[v1]--] = faceIndex;
		faceIndicesPerVertex[jumpInFacesPerVertex[v2] - numberOfFacesOrEdgesPerVertex[v2]--] = faceIndex;
		faceIndicesPerVertex[jumpInFacesPerVertex[v3] - numberOfFacesOrEdgesPerVertex[v3]--] = faceIndex;
	}

	unsigned int* sortingEdges = new unsigned int[highstNumOfFaces * 2];
	unsigned int sumOfEdges = 0;

	unsigned int
		i, a, b, c, d,
		firstIndex,
		elements;
	//Sort Faces and determine VerticesPerVertex
	for (unsigned int v = 0; v < mesh->numOfVertices; ++v)
	{
		elements = jumpInFacesPerVertex[v];
		if (v != 0)
		{
			firstIndex = jumpInFacesPerVertex[v - 1];
			elements -= firstIndex;
		}
		else
		{
			firstIndex = 0;
		}
		i = 0;
		while (i < elements) {
			getAB(v, faceIndicesPerVertex[(i + firstIndex)] * 3, mesh, a, b);
			sortingEdges[i * 2] = a;
			sortingEdges[i * 2 + 1] = b;
			++i;
		}
		i = 0;
		unsigned int j, temp, searchVertex;
		//Sort triangles and count edges (or vertices) per vertex
		numberOfFacesOrEdgesPerVertex[v] = elements;
		while (i < elements) {
			searchVertex = sortingEdges[i * 2 + 1];
			for (j = i + 1; j < elements; ++j)
			{
				if (searchVertex == sortingEdges[j * 2])
				{
					if (j - i > 1)//Triangle is not next to its connected triangle, thus, swap required
					{
						//swap positions
						temp = sortingEdges[j * 2];
						sortingEdges[j * 2] = sortingEdges[(i + 1) * 2];
						sortingEdges[(i + 1) * 2] = temp;
						temp = sortingEdges[j * 2 + 1];
						sortingEdges[j * 2 + 1] = sortingEdges[(i + 1) * 2 + 1];
						sortingEdges[(i + 1) * 2 + 1] = temp;
						temp = faceIndicesPerVertex[firstIndex + j];
						faceIndicesPerVertex[firstIndex + j] = faceIndicesPerVertex[firstIndex + (i + 1)];
						faceIndicesPerVertex[firstIndex + (i + 1)] = temp;
					}
					break;
				}
			}
			//Edge was not found, thus extra edge
			if (j == elements &&
			   (i + 1 != elements || searchVertex != sortingEdges[0]))//Case for last face) 
			{
				++numberOfFacesOrEdgesPerVertex[v];
			}
			++i;
		}
		sumOfEdges += numberOfFacesOrEdgesPerVertex[v];
	}

	jumpInVertices = new unsigned int[mesh->numOfVertices];
	jumpInVertices[0] = numberOfFacesOrEdgesPerVertex[0];
	//Prefix sum vertex edge jumps
	for (unsigned int i = 1; i < mesh->numOfVertices; ++i)
	{
		jumpInVertices[i] = jumpInVertices[i - 1] + numberOfFacesOrEdgesPerVertex[i];
	}

	vertexIndicesPerVertex = new unsigned int[sumOfEdges];
	//Write connected vertices
	for (unsigned int v = 0; v < mesh->numOfVertices; ++v)
	{
		elements = jumpInFacesPerVertex[v];
		if (v != 0)
		{
			firstIndex = jumpInFacesPerVertex[v - 1];
			elements -= firstIndex;
		}
		else
		{
			firstIndex = 0;
		}
		getAB(v, faceIndicesPerVertex[firstIndex + elements - 1] * 3, mesh, c, d);
		i = 0;
		while (i < elements) {
			getAB(v, faceIndicesPerVertex[(firstIndex + i)] * 3, mesh, a, b);
			//No face inbetween
			if (d != a)
				vertexIndicesPerVertex[jumpInVertices[v] - numberOfFacesOrEdgesPerVertex[v]--] = a;
			vertexIndicesPerVertex[jumpInVertices[v] - numberOfFacesOrEdgesPerVertex[v]--] = b;
			//c = a; will not be accessed
			d = b;
			++i;
		}
	}
}

const unsigned char MAX_PATCH_SIZE = 255;
const short VERTEX_COUNTER = MAX_PATCH_SIZE;
const short VERTEX_FIRST_WRITING_POSITION = MAX_PATCH_SIZE - 1;
const unsigned char PATCH_DIVIDER = 230;
const short FACE_COUNTER = VERTEX_COUNTER + 1;
const short COMPLETE_PATCH_SIZE = MAX_PATCH_SIZE * 3 + 2;//255 Vertices, 2 * 255 Faces, 2 Counter
const unsigned int FREE = 0;

float getQualityOfExpansion(const Index_Mesh const* mesh, const unsigned int& v, unsigned int* patchIndex, unsigned int* patches)
{

}

struct QualityAndIndex {
	float quality;
	unsigned int index;
	bool operator()(const QualityAndIndex const & a, const QualityAndIndex const& b) const {
		return a.quality > b.quality;
	}
	QualityAndIndex(float quality, unsigned int index)
	{
		this->index = index;
		this->quality = quality;
	}
	QualityAndIndex()
	{
		this->index = 0;
		this->quality = 0.0f;
	}
};

bool isListedInPatch(const unsigned int const& vertex_index, const unsigned int const& patch_start, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int const* patchIndex, const unsigned int const* patches)
{
	if (patchIndex[vertex_index] == FREE)
		return false;
	if (patchIndex[vertex_index] == patch_start / COMPLETE_PATCH_SIZE)
		return true;
	for (unsigned int i = patch_start + VERTEX_FIRST_WRITING_POSITION - patches[patch_start + VERTEX_COUNTER]; i < patch_start + VERTEX_COUNTER; ++i)
	{
		if (patches[patch_start + i] == vertexIndicesPerVertex[i])
			return true;
	}
	return false;
}

enum EVertexType {
	Edge,
	Inside,
	Free,
	Foreign
};

bool isForeignToPatch(const unsigned int const& vertex_index, const unsigned int const& patch_start, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int* patchIndices, const unsigned int const* patches)
{
	return patchIndices[vertex_index] != patch_start / COMPLETE_PATCH_SIZE && !isListedInPatch(vertex_index, patch_start, jumpInVertices, vertexIndicesPerVertex, patchIndices, patches);
}

EVertexType getVertexType(const unsigned int const& vertex_index, const unsigned int const& patch_start, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int* patchIndices, const unsigned int const * patches)
{
	if (patchIndices[vertex_index] == FREE)
		return Free;
	if (isForeignToPatch(vertex_index, patch_start, jumpInVertices, vertexIndicesPerVertex, patchIndices, patches))
		return Foreign;
	for (unsigned int i = vertex_index == 0 ? 0 : jumpInVertices[vertex_index - 1]; i < jumpInVertices[vertex_index]; ++i)
	{
		if ( patchIndices[i] == FREE || isForeignToPatch(i, patch_start, jumpInVertices, vertexIndicesPerVertex, patchIndices, patches) )
			return Edge;
	}
	return Inside;
}

unsigned int countNewVertices(const unsigned int const & vertex_index, const unsigned int const & patch_start, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int const* patchIndices, const unsigned int const* patches)
{
	unsigned int res = 0;
	for (unsigned int i = vertex_index == 0 ? 0 : jumpInVertices[vertex_index - 1]; i < jumpInVertices[vertex_index]; ++i)
	{
		if (patchIndices[i] == FREE || ! isListedInPatch(vertex_index, patch_start, jumpInVertices, vertexIndicesPerVertex, patchIndices, patches) )
		{
			++res;
		}
	}
	return res;
}

void expandBest(std::vector<QualityAndIndex>& bestCandidate, const Index_Mesh const* mesh, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int const* jumpInFacesPerVertex, const unsigned int const* faceIndicesPerVertex, bool* facesUnused, unsigned int* patchIndices, unsigned int* patches)
{
	const unsigned int vertex_index = bestCandidate.front().index;
	const unsigned int patch_start = patchIndices[vertex_index] * COMPLETE_PATCH_SIZE;

	if (patches[patch_start + VERTEX_COUNTER] + countNewVertices(vertex_index, patch_start, jumpInVertices, vertexIndicesPerVertex, patchIndices, patches) >= MAX_PATCH_SIZE)
	{
		
		for (unsigned int i = vertex_index == 0 ? 0 : jumpInFacesPerVertex[vertex_index - 1]; i < jumpInFacesPerVertex[vertex_index]; ++i)
		{
			if (facesUnused[faceIndicesPerVertex[i]]) {
				facesUnused[faceIndicesPerVertex[i]] = false;
				patches[patch_start + patches[patch_start + FACE_COUNTER]++] = faceIndicesPerVertex[i];
			}
		}
		for (unsigned int i = vertex_index == 0 ? 0 : jumpInVertices[vertex_index - 1]; i < jumpInVertices[vertex_index]; ++i)
		{
			bool foreignNotJetAdded = false;
			if (patchIndices[i] != FREE && patchIndices[i] != vertex_index) {
				foreignNotJetAdded = true;
				for (unsigned int j = patch_start + VERTEX_FIRST_WRITING_POSITION - patches[patch_start + VERTEX_COUNTER]; j < patch_start + VERTEX_COUNTER; ++j)
					if (patches[patch_start + j] == vertexIndicesPerVertex[i])
					{
						foreignNotJetAdded = false;
						break;
					}
			}
			if (patchIndices[i] == FREE || foreignNotJetAdded) {
				patches[patch_start + VERTEX_FIRST_WRITING_POSITION - patches[patch_start + VERTEX_COUNTER]++] = vertexIndicesPerVertex[i];
				update(vertexIndicesPerVertex[i], bestCandidate, mesh, jumpInVertices, vertexIndicesPerVertex, jumpInFacesPerVertex, faceIndicesPerVertex, facesUnused, patchIndices, patches);
			}
			if (patchIndices[i] == FREE) {
				patchIndices[i] = vertex_index;
			}
		}
	}
}

void getFirstPatches(const Index_Mesh const* mesh, const unsigned int const* jumpInVertices, const unsigned int const* vertexIndicesPerVertex, const unsigned int const* jumpInFacesPerVertex, const unsigned int const* faceIndicesPerVertex, unsigned int*& patches)
{
	unsigned int numberOfPatches = mesh->numOfVertices / PATCH_DIVIDER;
	unsigned int* patchIndices = new unsigned int[mesh->numOfVertices];
	memset(patchIndices, 0, mesh->numOfVertices * sizeof(*patchIndices));
	bool* facesUnused = new bool[mesh->numOfVertices];
	memset(facesUnused, true, mesh->numOfVertices * sizeof(*facesUnused));

	patches = new unsigned int[numberOfPatches * 1.5 * COMPLETE_PATCH_SIZE];

	std::vector<QualityAndIndex> bestCandidate(mesh->numOfVertices);

	for (unsigned int i = 0; i < numberOfPatches; ++i)
	{
		patches[COMPLETE_PATCH_SIZE * i + VERTEX_FIRST_WRITING_POSITION] = i;
		patches[COMPLETE_PATCH_SIZE * i + VERTEX_COUNTER] = 1;
		patches[COMPLETE_PATCH_SIZE * i + FACE_COUNTER] = 0;
		patchIndices[PATCH_DIVIDER * i] = i;
	}
}

void getReorderPosition(const unsigned int const* patches, unsigned int*& vertices, unsigned int*& faces)
{

}

Index_Mesh* rpm(Index_Mesh* mesh)
{
	unsigned int* jumpInVertices;
	unsigned int* vertexIndicesPerVertex;
	unsigned int* jumpInFacesPerVertex;
	unsigned int* faceIndicesPerVertex;
	getVerticesAndFacesPerVertex(mesh, jumpInVertices, vertexIndicesPerVertex, jumpInFacesPerVertex, faceIndicesPerVertex);
	unsigned int* patches;
	getFirstPatches(mesh, jumpInVertices, vertexIndicesPerVertex, jumpInFacesPerVertex, faceIndicesPerVertex, patches);
	delete[] jumpInVertices;
	delete[] vertexIndicesPerVertex;
	delete[] jumpInFacesPerVertex;
	delete[] faceIndicesPerVertex;

	unsigned int* vertices;
	unsigned int* faces;
	getReorderPosition(patches, vertices, faces);
	delete[] patches;

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
