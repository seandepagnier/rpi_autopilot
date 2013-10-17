/* Copyright (C) 2007, 2009, 2013 Sean D'Epagnier <sean@depagnier.com>
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

#include "vector.h"
#include "rotate.h"

#define MULT_QUAT(d) \
   float res[4]; \
   res[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]; \
   res[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; \
   res[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; \
   res[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]; \
   d[0] = res[0], d[1] = res[1], d[2] = res[2], d[3] = res[3]; \

/* multiply quaternions q1 and q2 and put the result in q1 */
void MultQuat1(float q1[4], float q2[4])
{
   MULT_QUAT(q1);
}

/* multiply quaternions q1 and q2 and put the result in q2 */
void MultQuat2(float q1[4], float q2[4])
{
   MULT_QUAT(q2)
}

/* create a quaternion from an angle and vector it rotates around */
void angvec2quat(float q[4], float angle, float v[3])
{
   float n = magnitude(v);
   float fac;
   if(n == 0)
      fac = 0;
   else
      fac = sin(angle/2) / n;

   q[0] = cos(angle/2);
   q[1] = v[0]*fac;
   q[2] = v[1]*fac;
   q[3] = v[2]*fac;
}

/* take a vector and quaternion, and rotate the vector
   by the quaternion, store the result back in the vector */
void rotvecquat(float v[3], float q[4])
{
   float w[4] = {0, v[0], v[1], v[2]};
   float r[4] = {q[0], -q[1], -q[2], -q[3]};
   MultQuat2(q, w);
   MultQuat1(w, r);
   v[0] = w[1], v[1] = w[2], v[2] = w[3];
}

/* take two vectors a and b, and store the quaternion rotation
   from a to b in x */
void vec2vec2quat(float x[4], float a[3], float b[3])
{
   float n[3];
   cross(n, a, b);
      
   float fac = dot(a, b) / sqrt(magnitude2(a) * magnitude2(b));

   /* because of numerical errors, it is possible for this to be in some
      cases very slightly larger than 1, set to 1 or acos will barf */
   if(fac > 1)
      fac = 1;

   float ang = acos(fac);
   angvec2quat(x, ang, n);
}

float quatmagnitude(float q[4])
{
    return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

void quatnormalize(float q[4])
{
    float m = quatmagnitude(q);
    q[0] /= m;
    q[1] /= m;
    q[2] /= m;
    q[3] /= m;
}

/* give the angle of rotation of a quaternion */
float quatangle(float q[4])
{
  return 2*acos(q[0]);
}

/* give the angle of rotation of a quaternion
   from the imaginary part */
float subquatangle(float v[3])
{
  return 2*asin(magnitude(v));
}

void quaternion_pitch_roll_yaw(float q[4], float *pitch, float *roll, float *yaw)
{
    float q0_2 = q[0]*q[0], q1_2 = q[1]*q[1], q2_2 = q[2]*q[2], q3_2 = q[3]*q[3];
    *pitch = rad2deg(asin(2 * (q[0]*q[2] - q[1]*q[3])));
    *roll = rad2deg(atan2(2 * (q[0]*q[1] + q[2]*q[3]), q0_2  - q1_2 - q2_2 + q3_2));
    *yaw = rad2deg(atan2(2 * (q[1]*q[2] + q[0]*q[3]), q0_2 + q1_2 - q2_2 - q3_2));
}
