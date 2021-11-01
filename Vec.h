/**
 * @author  Hendrik Annuth
 * @version 1.0
 *
 * @section DESCRIPTION
 * 
 * This is the representation of a point. This basic class is used for all basic vector operations.
 *
 * @member 	x x-Coordinate
 * @member 	y y-Coordinate
 * @member  z z-Coordinate
 */
#ifndef HEADER_DEFINED_Vec
#define HEADER_DEFINED_Vec

namespace cgx{


#if  !defined(NULL)
     #define NULL 0
#endif

#define Pt Vec

typedef float coordType;

class  Vec{
 public:

	typedef coordType coord_type;

	Vec(coordType x,coordType y,coordType z)
	{
		this->x=x;
		this->y=y;
		this->z=z;
	}
	Vec() {};
	~Vec() {};
	coordType x;
	coordType y;
	coordType z;
};

class  Vec64{
 public:	
	 Vec64(double x,double y,double z)
	{
		this->x=x;
		this->y=y;
		this->z=z;
	}
	Vec64()
	{};
	~Vec64(){};
	double x;
	double y;
	double z;
};

}
#endif