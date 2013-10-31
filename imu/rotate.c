/* Copyright (C) 2007, 2008 Sean D'Epagnier <sean@depagnier.com>
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

#include "vector.h"
#include "quaternion.h"

/* take a vector and apply the same rotation from,
   the alignment vector to the given axis */
static void alignaxis(float val[3], float align[3], float axis[3])
{
   float tilt[4]; /* tilt is a quaternion rotation */
   vec2vec2quat(tilt, align, axis);
   rotvecquat(val, tilt);
}

/* rotate vec around the vector axis by ang */
void rotatevecaroundaxis(float vec[3], float axis[3], float ang)
{
   float q[4];
   angvec2quat(q, ang, axis);
   rotvecquat(vec, q);
}

/* routine to take magnetometer and accelerometer readings and produce a magnetometer
   reading as if the accelerometer reading were <0, 0, 1> */
void rotateoutmag(float mval[3], float aval[3])
{
   /* laser is on x axis, so first rotate out x which is pitch, then
      rotate out roll */
   float vec1[3] = {aval[0], aval[1], aval[2]};
   normalize(vec1);
 
   float vec2[3] = {0, vec1[1], vec1[2]};
   normalize(vec2);

   /* rotate out tilt */
   alignaxis(mval, vec1, vec2);

   /* now rotate to g */
   vec1[0] = 0, vec1[1] = 0, vec1[2] = 1;
   alignaxis(mval, vec2, vec1);
}

float heading_resolve(float h)
{
    while(h < -180) h += 360;
    while(h >= 180) h -= 360;
    return h;
}

float heading_positive(float h)
{
    while(h < 0) h += 360;
    while(h >= 360) h -= 360;
    return h;
}
