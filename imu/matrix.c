/* Copyright (C) 2008 Sean D'Epagnier <sean@depagnier.com>
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

#include "matrix.h"

#define MAKE_SIMPLE_MATRIX_OPS(OP, NAME) \
   /* compute x = a OP b where all matrixes are m by n */ \
   void _matrix_##NAME(float *X, const float *A, const float *B, \
                       int m, int n) \
   { \
      float (*x)[m] = (float (*)[m])X; \
      const float (*a)[m] = (const float (*)[m])A; \
      const float (*b)[m] = (const float (*)[m])B; \
      int i, j; \
      for(i = 0; i<m; i++) \
         for(j = 0; j<n; j++) \
            x[i][j] = a[i][j] OP b[i][j]; \
   }

MAKE_SIMPLE_MATRIX_OPS(+, add)
MAKE_SIMPLE_MATRIX_OPS(-, sub)

/* compute x = a*b where x is m by o, a is m by n, b is n by o */
void _matrix_mult(float *X, const float *A, const float *B,
                  int m, int n, int o)
{
   /* if dest is also a source, do a copy to ensure correct operation */
   if(X == A || X == B) {
      float res[m*o];
      matrix_mult(res, A, B, m, n, o);
      memcpy(X, res, sizeof res);
      return;
   }

   float (*x)[o] = (float (*)[o])X;
   const float (*a)[n] = (const float (*)[n])A;
   const float (*b)[o] = (const float (*)[o])B;

   int i, j, k;
   for(i = 0; i<m; i++)
      for(j = 0; j<o; j++) {
         x[i][j] = 0;
         for(k = 0; k<n; k++)
            x[i][j] += a[i][k] * b[k][j];
      }
}

/* compute x = a*b' where x is m by o, a is m by n, b is o by n */
void _matrix_mult_tr(float *X, const float *A, const float *B,
                    int m, int n, int o)
{
   /* could be optimized, but this is used for simplicity */
   float bt[n*o];
   matrix_transpose(bt, B, o, n);
   matrix_mult(X, A, bt, m, n, o);
}

/* compute x = a*b*a' where x is m by m, a is m by n, b is n by n */
void _matrix_mult_triple(float *X, const float *A, const float *B,
                         int m, int n)
{
   /* could be optimized, but this is used for simplicity */
   float t[m][n];
   matrix_mult(t, A, B, m, n, n);
   matrix_mult_tr(X, t, A, m, n, m);
}

/* transpose X=Y' where x is m by n */
void _matrix_transpose(float *X, const float *Y, int m, int n)
{
   /* if a self-transpose, copy */
   if(X == Y) {
      float t[m*n];
      matrix_transpose(t, Y, m, n);
      memcpy(X, t, sizeof t);
      return;
   }

   float (*x)[n] = (float (*)[n])X;
   const float (*y)[m] = (const float (*)[m])Y;
   int i, j;
   for(i = 0; i<m; i++)
      for(j = 0; j<n; j++)
         x[i][j] = y[j][i];
}

/* return the trace of n by n matrix X */
float _matrix_trace(float *X, int n)
{
   float (*x)[n] = (float (*)[n])X;
   float total = 0;
   int i;
   for(i = 0; i<n; i++)
      total += x[i][i];
   return total;
}

/* set X to the n by n identity matrix */
void _matrix_identity(float *X, int n)
{
   float (*x)[n] = (float (*)[n])X;
   int i, j;
   for(i = 0; i<n; i++)
      for(j = 0; j<n; j++)
         x[i][j] = i == j;
}

/* multiply each element in X by mag where X is n by m */
void _matrix_scale(float mag, float *X, int n, int m)
{
   float (*x)[n] = (float (*)[n])X;
   int i, j;
   for(i = 0; i<n; i++)
      for(j = 0; j<n; j++)
         x[i][j] *= mag;
}

/* in place matrix inversion for b, where b is n by n
   return 0 on success, or 1 if b is singular */
int _matrix_invert(float *A, int n)
{
   float (*a)[n] = (float (*)[n])A;

   int i, j, k;
   // main cycle for columns of A matrix
   for ( k = 0; k < n; k++ ) {

      // make current element 1
      float aa = a[k][k];
      if ( aa == 0.0f )
         return 1;

      float aainv = 1.0f/aa;

      a[k][k] = 1.0f;
      for ( i = 0; i < n; i++ )
         a[k][i] *= aainv;

      // make all rows zero
      for ( j = 0; j < n; j++ ) {
         if ( j == k )
            continue;

         aa = a[j][k];
         a[j][k] = 0.0f;

         for ( i = 0; i < n; i++ )
            a[j][i] -= a[k][i]*aa;
      }
   }
   return 0;
}

void _matrix_print(const float *A, int n, int m)
{
   const float (*a)[m] = (const float (*)[m])A;
   int i, j;

   printf("[");
   for(i = 0; i<n; i++) {
      for(j = 0; j<m; j++) {
         printf("%.3g", a[i][j]);
         if(j < m-1)
            printf(",");
      }
      if(i < n-1)
         printf(";");
      else
         printf("]");
      printf("\n");
   }
}

/* takes matrix X which is nxn and vector D which is nx1
   and sets X to zeros except with the diagonal set to D
*/
void _matrix_diag(float *X, const float D[], int n)
{
   float (*x)[n] = (float (*)[n])X;
   int i, j;
   for(i = 0; i<n; i++)
      for(j = 0; j<n; j++)
         if(i == j)
            x[i][i] = D[i];
         else
            x[i][j] = 0;
}
