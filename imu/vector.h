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

#define MAKE_SIMPLE_VECTOR_DECLS(NAME) \
   void vector_##NAME(float x[], const float a[], const float b[], int n);

MAKE_SIMPLE_VECTOR_DECLS(add)
MAKE_SIMPLE_VECTOR_DECLS(sub)
MAKE_SIMPLE_VECTOR_DECLS(mul)
MAKE_SIMPLE_VECTOR_DECLS(div)

float vector_length2(const float vec[], int n);
float vector_length(const float vec[], int n);

float magnitude2(const float vec[3]);
float magnitude(const float vec[3]);
void normalize(float vec[3]);
void cross(float x[3], const float a[3], const float b[3]);
float dot(const float a[3], const float b[3]);
float dist2(const float a[3], const float b[3]);
float dist(const float a[3], const float b[3]);

void vector_scale(float mag, float x[], int n);

void vector_from_polar(float x[3], float radius, float incline, float azimuth);

