/**
 * @file: point.h
 * @brief: Integer point type used by DynamicVoronoi (third-party, integrated)
 * @author Third-party (original authors unknown); integrated by Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

#endif
