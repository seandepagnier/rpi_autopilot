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

void vec2vec2quat(float x[4], float a[3], float b[3]);
void rotvecquat(float v[3], float q[4]);
void angvec2quat(float q[4], float angle, float vec[3]);
void MultQuat2(float q1[4], float q2[4]);
void MultQuat1(float q1[4], float q2[4]);

float quatmagnitude(float q[4]);
void quatnormalize(float q[4]);
float quatangle(float q[4]);
float subquatangle(float v[3]);
void quaternion_pitch_roll_yaw(float q[4], float *pitch, float *roll, float *yaw);
