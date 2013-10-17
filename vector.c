/* Copyright (C) 2007, 2008, 2009 Sean D'Epagnier <sean@depagnier.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <math.h>

#include "rotate.h"

#define MAKE_SIMPLE_VECTOR_OPS(OP, NAME) \
   /* compute x = a+b where all vectors are m by n */ \
   void vector_##NAME(float x[], const float a[], const float b[], int n) \
   { \
      int i; \
      for(i = 0; i<n; i++) \
         x[i] = a[i] OP b[i]; \
   }

MAKE_SIMPLE_VECTOR_OPS(+, add)
MAKE_SIMPLE_VECTOR_OPS(-, sub)
MAKE_SIMPLE_VECTOR_OPS(*, mul)
MAKE_SIMPLE_VECTOR_OPS(/, div)

float vector_length2(const float vec[], int n)
{
   float len = 0;
   int i;
   for(i=0; i<n; i++)
      len += vec[i]*vec[i];
   return len;
}

float vector_length(const float vec[], int n)
{
   return sqrt(vector_length2(vec, n));
}

float magnitude2(const float vec[3])
{
   return vector_length2(vec, 3);
}

float magnitude(const float vec[3])
{
   return sqrt(magnitude2(vec));
}

void normalize(float vec[3])
{
   float mag = magnitude(vec);
   vec[0] /= mag, vec[1] /= mag, vec[2] /= mag;
}

void cross(float x[3], const float a[3], const float b[3])
{
   x[0] = a[1] * b[2] - a[2] * b[1];
   x[1] = a[2] * b[0] - a[0] * b[2];
   x[2] = a[0] * b[1] - a[1] * b[0];
}

float dot(const float a[3], const float b[3])
{
   return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float dist2(const float a[3], const float b[3])
{
   float x = a[0] - b[0];
   float y = a[1] - b[1];
   float z = a[2] - b[2];
   return x*x + y*y + z*z;
}

float dist(const float a[3], const float b[3])
{
   return sqrt(dist2(a, b));
}

void vector_scale(float mag, float x[], int n)
{
   int i;
   for(i = 0; i<n; i++)
      x[i] *= mag;
}

void vector_from_polar(float x[3], float radius, float incline, float azimuth)
{
   float ri = deg2rad(incline), ra = deg2rad(azimuth);
   x[0] = radius*cos(ri)*cos(ra);
   x[1] = radius*cos(ri)*sin(ra);
   x[2] = radius*sin(ri);
}
