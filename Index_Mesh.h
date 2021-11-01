#ifndef HEADER_DEFINED_Index_Mesh
#define HEADER_DEFINED_Index_Mesh

namespace cgx {

	class Index_Mesh {
	public:
		Index_Mesh(void);
		~Index_Mesh(void);

		bool hasUV_Coordinates();
		bool hasNormals();
		bool hasColor();

		Index_Mesh* copy();
		void clean();

		void clean_uv();
		void clean_normals();
		void clean_color();

		void calc_normals();

		void optimiseTriangleIndicesForFastMemoryAccess();
		unsigned int getNumOfEdges();

		unsigned int numOfVertices;
		unsigned char* colors;
		float* vertices;
		float* normals;

		float* uv_coords;
		unsigned int numOfCopies;
		//A vertex copy consits of two values first is the index of the copy and second the index of the copy source.
		//A copy is not allowed to be a copy source, so first an second values have to be disjunct.
		unsigned int* uv_vertex_copies;

		unsigned int numOfFaces;
		unsigned int* indices;

		/**
		  * This will scale the entire mesh
		  * @parem scaleFactor the factor the mesh is scaled by
		  */
		void scale(float scaleFactor);

#if _DEBUG
		bool invariant();
#endif
	};

}
#endif