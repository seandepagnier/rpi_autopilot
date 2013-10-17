/* Copyright (C) 2007 Sean D'Epagnier <sean@depagnier.com>
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

#define rad2deg(X) ((X)*180/M_PI)
#define deg2rad(X) ((X)/180*M_PI)

void rotatevecaroundaxis(float vec[3], float axis[3], float ang);
void rotateoutmag(float mval[3], float aval[3]);

float heading_resolve(float h);
float heading_positive(float h);

void print_value(float v);

static inline float lp_filter(float a, float b, float d)
{
    return d*a + (1-d)*b;
}
