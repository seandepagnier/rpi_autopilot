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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "quaternion.h"
#include "matrix.h"
#include "vector.h"
#include "rotate.h"

// calculates X += (JtJ)^-1*JtRt
#define ApplyLeastSquares(X, J, R, m, n) \
  _ApplyLeastSquares(X, (const float *)(J), R, m, n)
static int _ApplyLeastSquares(float X[], const float *pJ, const float R[], int m, int n)
{
   float (*J)[m] = (float(*)[m])pJ;
   int i, j, k;

   // Compute N=(JtRt)t, mxn*nx1 get mx1
   float N[m];
   for(i = 0; i<m; i++) {
      N[i] = 0;
      for(j = 0; j<n; j++)
         N[i] += J[j][i]*R[j];
   }

   // Compute S, S = JtJ, multiply mxn matrix with nxm and get mxm
   float S[m][m];
   for(i = 0; i<m; i++)
      for(j = 0; j<m; j++) {
         S[i][j] = 0;
         for(k = 0; k<n; k++)
            S[i][j] += J[k][i]*J[k][j];
      }

   // Invert S
   if(matrix_invert(S, m))
      return 1;

   // Compute X+=(S^-1*Nt)t, multiply mxm matrix with mx1 and get mx1
   for(i = 0; i<m; i++)
      for(j = 0; j<m; j++)
         X[i] += S[i][j]*N[j];

   return 0;
}

static void InitialEstimateSphere(float (*points)[3], int count, float state[4])
{
   int i, j;
   /* set state to the average */
   for(i = 0; i<3; i++) {
      state[i] = 0;
      for(j = 0; j<count; j++) {
         float val = points[j][i];
         state[i] += val;
      }
      state[i] /= count;
   }   

   state[3] = 1;
}

/* calibrate out bias in magnetic sensors using a sphere fit */
/* calculate best fit for a sphere
   (x-a)^2 + (y-b)^2 + (z-c)^2 = r^2
*/
static float CalcBestFitSphere(float (*points)[3], int count, float X[4])
{
    float J[count][4];
    float R[count];

    int i;
    for(i = 0; i<count; i++) {
        float d[3];
        vector_sub(d, points[i], X, 3);

        J[i][0] = -2*d[0];
        J[i][1] = -2*d[1];
        J[i][2] = -2*d[2];
        J[i][3] = -2*X[3];

        R[i] = X[3]*X[3] - magnitude2(d);
   }

    if(ApplyLeastSquares(X, J, R, 4, count))
        return 0;
    
    if(X[3] < 0 || isnan(X[0]))
        return 0;
    
    return vector_length(R, count) / count;
}

/* initialize measurement map to invalid */
static float mag_log[72][3];
void init_mag_calibration()
{
    int i;
    for(i=0; i<72; i++)
        mag_log[i][0] = NAN;
}

int calibrate_mag(float mag[3], float state[4])
{
    float theta = atan2(mag[0], mag[1]);
    int theta_i = (rad2deg(theta) + 180) / 30;
    if(theta_i < 0 || theta_i >= 12)
        theta_i = 0;

    float phi = atan2(mag[2], hypot(mag[0], mag[1]));
    int phi_i = (rad2deg(phi) + 90) / 30;
    if(phi_i < 0) phi_i = 0;
    if(phi_i >= 6) phi_i = 5;

    int ind = phi_i * 12 + theta_i, i;
    for(i=0; i<3; i++)
        mag_log[ind][i] = mag[i];

    float v_mag_log[72][3];
    int c = 0, j;
    for(i = 0; i<72; i++)
        if(!isnan(mag_log[i][0])) {
//            printf("log: %d %f %f %f\n", i, mag_log[i][0], mag_log[i][1], mag_log[i][2]);
            for(j=0; j<3; j++)
                v_mag_log[c][j] = mag_log[i][j];
            c++;
        }
    if(c < 4) /* less than 4 and can give very poor results */
        return 0;

//    printf("thetai: %d, phii: %d, ind: %d, c: %d\n", theta_i, phi_i, ind, c);

    InitialEstimateSphere(v_mag_log, c, state);
//    printf("initial estimate: %.3f %.3f %.3f %.3f  %d\n",
//           state[0], state[1], state[2], state[3], c);
    float newstate[4], d[4], residual;
    memcpy(newstate, state, sizeof newstate);
    int iterations;
    for(iterations = 0; iterations < 10; iterations++) {
        if(!(residual = CalcBestFitSphere(v_mag_log, c, newstate)))
            return 0;

        vector_sub(d, state, newstate, 4);
        memcpy(state, newstate, sizeof newstate);

        if(vector_length(state, 4) > 10) /* diverged */
            return 0;

        if(vector_length(d, 4) < 1e-5) {
//            printf("update estimate: %.3f %.3f %.3f %.3f\n",
//                   state[0], state[1], state[2], state[3]);

            if(residual < .02) {
                for(j=0; j<3; j++) {
                    for(i = 0; i<72; i++)
                        mag_log[i][j] -= state[j];
                }
                return 1;
            }
            return 0;
        }
    }
    return 0;
}

float CalcBestLine(float *points, int count, float X[2])
{
    float J[count][2];
    float R[count];

    X[0] = X[1] = 0; /* initial conditions */
    int i;
    for(i = 0; i<count; i++) {
        /*
        a*x + b - y = 0
        dy/da = -x
        dy/db = -1
        */
        float x = i, y = points[i];

        J[i][0] = -x;
        J[i][1] = -1;

        R[i] = X[0]*x + X[1] - y;
   }

    if(ApplyLeastSquares(X, J, R, 2, count))
        return 0;

    return vector_length(R, count) / count;
}

float CalcBestSine(float *points, int count, float X[4])
{
    float J[count][3];
    float R[count];

    int i;
    for(i = 0; i<count; i++) {
        
        /*
          y = a*sin(b*x+c) + d
        */
        float x = i, y = points[i];

        float inner = x/X[1] + X[2];
        J[i][0] = -sin(inner);
        J[i][1] = 2*x*X[0]*cos(inner)/(X[1]*X[1]);
        J[i][2] = -X[0]*cos(inner);

        R[i] = X[0]*sin(inner) - y;
   }

    if(ApplyLeastSquares(X, J, R, 3, count))
        return 0;
    
    return vector_length(R, count) / count;
}
