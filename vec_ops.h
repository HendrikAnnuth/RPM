/**
 * @author  Hendrik Annuth
 * @version 1.0
 *
 * @section DESCRIPTION
 * 
 * This unit contains all commen vector operations, Vec* is interpreted as a vector hear
 */

#include "Vec.h"
#include <cmath>

namespace cgx{

#define COS10 0.98480775301f
//cos 20 which is used to identify movements in up direction
#define COS20 0.93969262078f
//cos 45 which is used to identify twits when connecting to borders
#define COS45 0.7071067811f

#define COS80 0.1736481777f
//If the edge length of and equal sided triangle is a, its
//surface area =  a*a*SIN60/2
#define SIN60 0.8660254f

bool equal(const Vec* p1,const Vec* p2);

inline Vec* crossProduct(const Vec* p1,const Vec* p2,Vec* res);

inline Vec* scale(const Vec* p,const float scalar,Vec* res);

Vec* scaleTo(const Vec* p,const float newVectorLength,Vec* res);

inline Vec* vec_AB(const Vec* a,const Vec* b,Vec* res);

Vec* div(const Vec* p,const float divider,Vec* res);

inline Vec* subt(const Vec* p1,const Vec* p2,Vec* res);

inline float qlength(const Vec* p);

inline float length(const Vec* p);

inline float qdist(const Vec* a,const Vec* b,Vec* res);

float dist(const Vec* a,const Vec* b,Vec* res);

inline Vec* add(const Vec* p1,const Vec* p2,Vec* res);

Vec* invert(const Vec* p1,Vec* res);

inline float scalarProduct(const Vec* p1,const Vec* p2);

Vec* normalize(const Vec* p,Vec* res);

inline Vec* normalOnTriangleCCW(const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* res);

Vec* centroidTriangle(const Vec* p1,const Vec* p2,const Vec* p3,Vec* res);

Vec* setTo(Vec* aim,const Vec* source);

Vec* setTo(Vec* aim,const float x,const float y,const float z);

bool zeroVector(const Vec* p);

inline int pointOfTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3);

inline Vec* rotateX(const float angleRad,const Vec* p,Vec* res);

inline Vec* rotateY(const float angleRad,const Vec* p,Vec* res);

inline Vec* rotateZ(const float angleRad,const Vec* p,Vec* res);

inline float calcAngleRad(const Vec* n1,const Vec* n2,Vec* temp1,Vec* temp2);

inline float qdistPtRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* temp2);

float distPtRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* temp2);

inline float qdistPtToPlane(const Vec* pt,const Vec* originPt,const Vec* normal,Vec* temp);

float distPtToPlane(const Vec* pt,const Vec* originPt,const Vec* normal,Vec* temp);

inline float qdistPtTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);

float distPtTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);

inline float qdistPtTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);

bool ptAbovePlane(const Vec* pt,const Vec* originPt,const Vec* n,Vec* temp);

inline Vec* inLineSegment(const Vec* p1,const Vec* p2,Vec* temp1,Vec* temp2,Vec* res);

inline Vec* inTriangle(const Vec* u,const Vec* v,const Vec* w,Vec* res);

inline Vec* projectPtOnPlane(const Vec* pt,const Vec* oriPt,const Vec* n,Vec* res);

inline Vec* projectPtOnRay(const Vec* pt,const Vec* oriPt,const Vec* dir,Vec* temp1,Vec* res);

inline Vec* projectPtOnTriangle(const Vec* pt,const Vec* p1,const Vec* p2,const Vec* p3,Vec* u,Vec* v,Vec* w,Vec* res);

Vec* intersectionRayPlane(const Vec* oriPtRay,const Vec* dir,const Vec* oriPtPlane,const Vec* n,Vec* res);

Vec* intersectionRayTriangle(const Vec* oriPt,const Vec* dir,const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* res);

bool intersectionTriangleTriangle(const Vec* t1_1,const Vec* t1_2,const Vec* t1_3,const Vec* t2_1,const Vec* t2_2,const Vec* t2_3,Vec* temp1,Vec* temp2,Vec* temp3,Vec* temp4,Vec* temp5);

void barycentricCoordinates(const Vec* p1,const Vec* p2,const Vec* p3,const Vec* p,Vec* u,Vec* v,Vec* w,float& b1,float& b2);

void barycentricCoordinates(const Vec* p1,const Vec* p2,const Vec* p3,const Vec* p,Vec* u,Vec* v,Vec* w,float& b1,float& b2,float& b3);

float lengthOfLongestTriangleEdge(const Vec* pt1,const Vec* pt2,const Vec* pt3,Vec* temp);

float triangleQuality(const Vec* p1,const Vec* p2,const Vec* p3,Vec* temp1,Vec* temp2,Vec* temp3);

}