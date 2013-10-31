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

/* to make it possible to pass 2D arrays without casting
   these functions are macros with builtin casts */
#define MAKE_SIMPLE_MATRIX_DECLS(NAME) \
   void _matrix_##NAME(float *X, const float *A, const float *B, \
                   int m, int n);

#define matrix_add(X, A, B, n, m) \
    _matrix_add((float*)(X), (const float*)(A), (const float*)(B), n, m)
#define matrix_sub(X, A, B, n, m) \
    _matrix_sub((float*)(X), (const float*)(A), (const float*)(B), n, m)

MAKE_SIMPLE_MATRIX_DECLS(add)
MAKE_SIMPLE_MATRIX_DECLS(sub)

#define matrix_mult(X, A, B, n, m, o) \
    _matrix_mult((float*)(X), (const float*)(A), (const float*)(B), n, m, o)
void _matrix_mult(float *X, const float *A, const float *B,
                     int n, int m, int o);

#define matrix_mult_tr(X, A, B, n, m, o) \
    _matrix_mult_tr((float*)(X), (const float*)(A), (const float*)(B), n, m, o)
void _matrix_mult_tr(float *X, const float *A, const float *B,
                     int n, int m, int o);

#define matrix_mult_triple(X, A, B, n, m) \
    _matrix_mult_triple((float*)(X), (const float*)(A), (const float*)(B), n, \
                        m)
void _matrix_mult_triple(float *X, const float *A, const float *B,
                     int n, int m);

#define matrix_transpose(X, Y, m, n) \
    _matrix_transpose((float*)(X), (const float*)(Y), n, m)
void _matrix_transpose(float *X, const float *Y, int m, int n);

#define matrix_trace(X, n) \
    _matrix_trace((float*)(X), n)
float _matrix_trace(float *X, int n);

#define matrix_identity(X, n) \
    _matrix_identity((float*)(X), n)
void _matrix_identity(float *X, int n);

#define matrix_scale(mag, X, n, m) \
    _matrix_scale(mag, (float*)(X), n, m)
void _matrix_scale(float mag, float *X, int n, int m);

#define matrix_invert(A, n) \
    _matrix_invert((float*)(A), n)
int _matrix_invert(float *A, int n);

#define matrix_print(A, n, m) \
   _matrix_print((const float*)(A), n, m)
void _matrix_print(const float *A, int n, int m);

#define matrix_diag(X, D, n) \
    _matrix_diag((float*)(X), D, n)
void _matrix_diag(float *X, const float D[], int n);
