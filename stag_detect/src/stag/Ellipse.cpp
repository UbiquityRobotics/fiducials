#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

inline double FastSinHP(double x);
inline double FastCosHP(double x);

#include "stag/Ellipse.h"


#define pi 3.14159265
#define BOOKSTEIN 0  // method1
#define FPF 1        // method2

////////////////////////////////////////////////////
/// INTERNAL FUNCTIONS OF THE EllipseFit - BEGIN ///
////////////////////////////////////////////////////
double **AllocateMatrix(int noRows, int noColumns) {
  double **m = new double *[noRows];
  // double **m = (double **) malloc(noRows*sizeof(double*));

  for (int i = 0; i < noRows; i++) {
    m[i] = new double[noColumns];
    memset(m[i], 0, sizeof(double) * noColumns);
  }

  return m;
}

void DeallocateMatrix(double **m, int noRows) {
  for (int i = 0; i < noRows; i++) delete[] m[i];
  delete[] m;
}

void multMatrix(double **m, double **g, double **mg) {
  // First clear the return array
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++) mg[i][j] = 0;

  // Perform the matrix math
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++)
      for (int k = 0; k < 4; k++) mg[i][j] = mg[i][j] + (m[i][k] * g[k][j]);
}

void ROTATE(double **a, int i, int j, int k, int l, double tau, double s) {
  double g, h;
  g = a[i][j];
  h = a[k][l];
  a[i][j] = g - s * (h + g * tau);
  a[k][l] = h + s * (g - h * tau);
}

void jacobi(double **a, int n, double d[], double **v, int nrot) {
  int j, iq, ip, i;
  double tresh, theta, tau, t, sm, s, h, g, c;

  double *b = new double[n + 1];
  double *z = new double[n + 1];
  memset(b, 0, sizeof(double) * (n + 1));
  memset(z, 0, sizeof(double) * (n + 1));

  for (ip = 1; ip <= n; ip++) {
    for (iq = 1; iq <= n; iq++) v[ip][iq] = 0.0;
    v[ip][ip] = 1.0;
  }
  for (ip = 1; ip <= n; ip++) {
    b[ip] = d[ip] = a[ip][ip];
    z[ip] = 0.0;
  }
  nrot = 0;
  for (i = 1; i <= 50; i++) {
    sm = 0.0;
    for (ip = 1; ip <= n - 1; ip++) {
      for (iq = ip + 1; iq <= n; iq++) sm += fabs(a[ip][iq]);
    }
    if (sm == 0.0) {
      delete b;
      delete z;
      return;
    }
    if (i < 4)
      tresh = 0.2 * sm / (n * n);
    else
      tresh = 0.0;
    for (ip = 1; ip <= n - 1; ip++) {
      for (iq = ip + 1; iq <= n; iq++) {
        g = 100.0 * fabs(a[ip][iq]);
        //				if (i > 4 && fabs(d[ip]) + g ==
        //fabs(d[ip])
        //				&& fabs(d[iq]) + g == fabs(d[iq]))

        if (i > 4 && g == 0.0)
          a[ip][iq] = 0.0;
        else if (fabs(a[ip][iq]) > tresh) {
          h = d[iq] - d[ip];
          if (g == 0.0)
            t = (a[ip][iq]) / h;
          else {
            theta = 0.5 * h / (a[ip][iq]);
            t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
            if (theta < 0.0) t = -t;
          }
          c = 1.0 / sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          a[ip][iq] = 0.0;
          for (j = 1; j <= ip - 1; j++) {
            ROTATE(a, j, ip, j, iq, tau, s);
          }
          for (j = ip + 1; j <= iq - 1; j++) {
            ROTATE(a, ip, j, j, iq, tau, s);
          }
          for (j = iq + 1; j <= n; j++) {
            ROTATE(a, ip, j, iq, j, tau, s);
          }
          for (j = 1; j <= n; j++) {
            ROTATE(v, j, ip, j, iq, tau, s);
          }
          ++nrot;
        }
      }
    }
    for (ip = 1; ip <= n; ip++) {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }
  // printf("Too many iterations in routine JACOBI");
  delete b;
  delete z;
}

///-----------------------------------------------------------
/// Perform the Cholesky decomposition
/// Return the lower triangular L  such that L*L'=A
void choldc(double **a, int n, double **l) {
  int i, j, k;
  double sum;
  double *p = new double[n + 1];
  memset(p, 0, sizeof(double) * (n + 1));

  for (i = 1; i <= n; i++) {
    for (j = i; j <= n; j++) {
      for (sum = a[i][j], k = i - 1; k >= 1; k--) sum -= a[i][k] * a[j][k];
      if (i == j) {
        if (sum <= 0.0)
        // printf("\nA is not poitive definite!");
        {
        } else
          p[i] = sqrt(sum);
      } else {
        a[j][i] = sum / p[i];
      }
    }
  }
  for (i = 1; i <= n; i++) {
    for (j = i; j <= n; j++) {
      if (i == j)
        l[i][i] = p[i];
      else {
        l[j][i] = a[j][i];
        l[i][j] = 0.0;
      }
    }
  }

  delete p;
}

int inverse(double **TB, double **InvB, int N) {
  int k, i, j, p, q;
  double mult;
  double D, temp;
  double maxpivot;
  int npivot;
  double **B = AllocateMatrix(N + 1, N + 2);
  double **A = AllocateMatrix(N + 1, 2 * N + 2);
  double **C = AllocateMatrix(N + 1, N + 1);
  double eps = 10e-20;

  for (k = 1; k <= N; k++)
    for (j = 1; j <= N; j++) B[k][j] = TB[k][j];

  for (k = 1; k <= N; k++) {
    for (j = 1; j <= N + 1; j++) A[k][j] = B[k][j];
    for (j = N + 2; j <= 2 * N + 1; j++) A[k][j] = (float)0;
    A[k][k - 1 + N + 2] = (float)1;
  }

  for (k = 1; k <= N; k++) {
    maxpivot = fabs((double)A[k][k]);
    npivot = k;
    for (i = k; i <= N; i++) {
      if (maxpivot < fabs((double)A[i][k])) {
        maxpivot = fabs((double)A[i][k]);
        npivot = i;
      }
    }

    if (maxpivot >= eps) {
      if (npivot != k)
        for (j = k; j <= 2 * N + 1; j++) {
          temp = A[npivot][j];
          A[npivot][j] = A[k][j];
          A[k][j] = temp;
        }

      D = A[k][k];
      for (j = 2 * N + 1; j >= k; j--) A[k][j] = A[k][j] / D;

      for (i = 1; i <= N; i++) {
        if (i != k) {
          mult = A[i][k];
          for (j = 2 * N + 1; j >= k; j--) A[i][j] = A[i][j] - mult * A[k][j];
        }
      }
    } else {  // printf("\n The matrix may be singular !!") ;

      DeallocateMatrix(B, N + 1);
      DeallocateMatrix(A, N + 1);
      DeallocateMatrix(C, N + 1);

      return (-1);
    }
  }

  for (k = 1, p = 1; k <= N; k++, p++)
    for (j = N + 2, q = 1; j <= 2 * N + 1; j++, q++) InvB[p][q] = A[k][j];

  DeallocateMatrix(B, N + 1);
  DeallocateMatrix(A, N + 1);
  DeallocateMatrix(C, N + 1);

  return (0);
}

void AperB(double **_A, double **_B, double **_res, int _righA, int _colA,
           int _righB, int _colB) {
  int p, q, l;
  for (p = 1; p <= _righA; p++)
    for (q = 1; q <= _colB; q++) {
      _res[p][q] = 0.0;
      for (l = 1; l <= _colA; l++)
        _res[p][q] = _res[p][q] + _A[p][l] * _B[l][q];
    }
}

void A_TperB(double **_A, double **_B, double **_res, int _righA, int _colA,
             int _righB, int _colB) {
  int p, q, l;
  for (p = 1; p <= _colA; p++)
    for (q = 1; q <= _colB; q++) {
      _res[p][q] = 0.0;
      for (l = 1; l <= _righA; l++)
        _res[p][q] = _res[p][q] + _A[l][p] * _B[l][q];
    }
}

void AperB_T(double **_A, double **_B, double **_res, int _righA, int _colA,
             int _righB, int _colB) {
  int p, q, l;
  for (p = 1; p <= _colA; p++)
    for (q = 1; q <= _colB; q++) {
      _res[p][q] = 0.0;
      for (l = 1; l <= _righA; l++)
        _res[p][q] = _res[p][q] + _A[p][l] * _B[q][l];
    }
}

////////////////////////////////////////////////////
///  INTERNAL FUNCTIONS OF THE EllipseFit - END  ///
////////////////////////////////////////////////////

void customEllipse::InitParams() {
  A1 = B1 = C1 = D1 = E1 = F1 = 0;

  fitError = 0;
  rmsError = 0;
  perimeter = 0;

  xPoints = NULL;
  yPoints = NULL;
  fitPoints = NULL;
  drawPoints = NULL;
  estRadians = NULL;
  closePoints = NULL;
}
customEllipse::customEllipse(pix *points, int noPnts) {
  // Initializations
  InitParams();

  noPoints = noPnts;

  fitPoints = (pix *)malloc(noPoints * sizeof(pix));
  memcpy(fitPoints, points, noPoints * sizeof(pix));

  // coordinate system transform
  for (int i = 0; i < noPoints; i++) fitPoints[i].y = -fitPoints[i].y;

  double **D = AllocateMatrix(noPoints + 1, 7);
  double **S = AllocateMatrix(7, 7);
  double **Const = AllocateMatrix(7, 7);
  double **temp = AllocateMatrix(7, 7);
  double **L = AllocateMatrix(7, 7);
  double **C = AllocateMatrix(7, 7);

  double **invL = AllocateMatrix(7, 7);
  double *d = new double[7];
  double **V = AllocateMatrix(7, 7);
  double **sol = AllocateMatrix(7, 7);
  double tx, ty;
  int nrot = 0;

  memset(d, 0, sizeof(double) * 7);

  int mode = FPF;
  switch (mode) {
    case (FPF):
      // fprintf(stderr, "FitEllipse: FPF mode");
      Const[1][3] = -2;
      Const[2][2] = 1;
      Const[3][1] = -2;
      break;
    case (BOOKSTEIN):
      // fprintf(stderr, "FitEllipse: BOOKSTEIN mode");
      Const[1][1] = 2;
      Const[2][2] = 1;
      Const[3][3] = 2;
  }

  if (noPoints < 6) return;  // false;

  // Now first fill design matrix
  for (int i = 1; i <= noPoints; i++) {
    tx = (double)fitPoints[i - 1].x;
    ty = (double)fitPoints[i - 1].y;

    D[i][1] = tx * tx;
    D[i][2] = tx * ty;
    D[i][3] = ty * ty;
    D[i][4] = tx;
    D[i][5] = ty;
    D[i][6] = 1.0;
  }

  A_TperB(D, D, S, noPoints, 6, noPoints, 6);
  choldc(S, 6, L);
  inverse(L, invL, 6);
  AperB_T(Const, invL, temp, 6, 6, 6, 6);
  AperB(invL, temp, C, 6, 6, 6, 6);
  jacobi(C, 6, d, V, nrot);
  A_TperB(invL, V, sol, 6, 6, 6, 6);

  // Now normalize them
  for (int j = 1; j <= 6; j++) /* Scan columns */
  {
    double mod = 0.0;
    for (int i = 1; i <= 6; i++) mod += sol[i][j] * sol[i][j];
    for (int i = 1; i <= 6; i++) sol[i][j] /= sqrt(mod);
  }

  double zero = 10e-20;
  double minev = 10e+20;
  int solind = 0;
  int i;
  switch (mode) {
    case (BOOKSTEIN):  // smallest eigenvalue
      for (i = 1; i <= 6; i++)
        if (d[i] < minev && fabs(d[i]) > zero) solind = i;
      break;

    case (FPF):
      for (i = 1; i <= 6; i++)
        if (d[i] < 0 && fabs(d[i]) > zero) solind = i;
  }

  // if (solind == 0) return false;

  // Now fetch the right solution
  // for (int j = 1; j <= 6; j++) pResult->coeff[j] = sol[j][solind];

  A1 = sol[1][solind];
  B1 = sol[2][solind];
  C1 = sol[3][solind];
  D1 = sol[4][solind];
  E1 = sol[5][solind];
  F1 = sol[6][solind];

  // CONVERTING THE CONIC EQUATION TO THE ELLIPSE EQUATION
  // Normalize Coefficients
  B1 /= A1;
  C1 /= A1;
  D1 /= A1;
  E1 /= A1;
  F1 /= A1;
  A1 /= A1;

  if (B1 == 0)  // Then not need to rotate the axes
  {
    A2 = A1;
    B2 = B1;
    C2 = C1;
    D2 = D1;
    E2 = E1;
    F2 = F1;

    // bRotation = false;
  } else if (B1 != 0)  // Rotate the axes
  {
    // bRotation = true;

    // Determine the rotation angle (in radians)
    rotation = atan(B1 / (A1 - C1)) / 2;

    // Compute the coefficients wrt the new coordinate system
    A2 = 0.5 * (A1 * (1 + cos(2 * rotation) + B1 * sin(2 * rotation) +
                      C1 * (1 - cos(2 * rotation))));
    B2 = (C1 - A1) * sin(2 * rotation) +
         B1 * cos(2 * rotation);  // B2 should turn to be zero?
    C2 = 0.5 * (A1 * (1 - cos(2 * rotation) - B1 * sin(2 * rotation) +
                      C1 * (1 + cos(2 * rotation))));
    D2 = D1 * cos(rotation) + E1 * sin(rotation);
    E2 = -D1 * sin(rotation) + E1 * cos(rotation);
    F2 = F1;
    // printf("Rotation in degrees: %.2f\n\n", rotation * 180 / pi);
  }

  // Transform the conic equation into the ellipse form
  D3 = D2 / A2;  // normalize x term's coef
  // A3 = 1;     //A2 / A2

  E3 = E2 / C2;  // normalize y term's coef
  // C3 = 1;     //C2 / C2

  cX = -(D3 / 2);  // center X
  cY = -(E3 / 2);  // center Y

  F3 = A2 * (cX * cX) + C2 * (cY * cY) - F2;

  // semimajor axis
  a = sqrt(F3 / A2);
  // semiminor axis
  b = sqrt(F3 / C2);

  // SET a2 - b2 !!!
  a2_b2 = a * a - b * b;

  // Center coordinates have to be re-transformed if rotation is applied!
  if (rotation != 0) {
    double tmpX = cX, tmpY = cY;
    cX = tmpX * cos(rotation) - tmpY * sin(rotation);
    cY = tmpX * sin(rotation) + tmpY * cos(rotation);
  }

  DeallocateMatrix(D, noPoints + 1);
  DeallocateMatrix(S, 7);
  DeallocateMatrix(Const, 7);
  DeallocateMatrix(temp, 7);
  DeallocateMatrix(L, 7);
  DeallocateMatrix(C, 7);
  DeallocateMatrix(invL, 7);
  delete d;
  DeallocateMatrix(V, 7);
  DeallocateMatrix(sol, 7);
}

customEllipse::customEllipse(double *pX, double *pY, int noPnts) {
  // Initializations
  InitParams();

  noPoints = noPnts;

  xPoints = (double *)malloc(noPoints * sizeof(double));
  memcpy(xPoints, pX, noPoints * sizeof(double));

  yPoints = (double *)malloc(noPoints * sizeof(double));
  memcpy(yPoints, pY, noPoints * sizeof(double));

  // coordinate system transform
  for (int i = 0; i < noPoints; i++) yPoints[i] = -yPoints[i];

  double **D = AllocateMatrix(noPoints + 1, 7);
  double **S = AllocateMatrix(7, 7);
  double **Const = AllocateMatrix(7, 7);
  double **temp = AllocateMatrix(7, 7);
  double **L = AllocateMatrix(7, 7);
  double **C = AllocateMatrix(7, 7);

  double **invL = AllocateMatrix(7, 7);
  double *d = new double[7];
  double **V = AllocateMatrix(7, 7);
  double **sol = AllocateMatrix(7, 7);
  double tx, ty;
  int nrot = 0;

  memset(d, 0, sizeof(double) * 7);

  int mode = FPF;
  switch (mode) {
    case (FPF):
      // fprintf(stderr, "FitEllipse: FPF mode");
      Const[1][3] = -2;
      Const[2][2] = 1;
      Const[3][1] = -2;
      break;
    case (BOOKSTEIN):
      // fprintf(stderr, "FitEllipse: BOOKSTEIN mode");
      Const[1][1] = 2;
      Const[2][2] = 1;
      Const[3][3] = 2;
  }

  if (noPoints < 6) return;  // false;

  // Now first fill design matrix
  for (int i = 1; i <= noPoints; i++) {
    tx = xPoints[i - 1];
    ty = yPoints[i - 1];

    D[i][1] = tx * tx;
    D[i][2] = tx * ty;
    D[i][3] = ty * ty;
    D[i][4] = tx;
    D[i][5] = ty;
    D[i][6] = 1.0;
  }

  A_TperB(D, D, S, noPoints, 6, noPoints, 6);
  choldc(S, 6, L);
  inverse(L, invL, 6);
  AperB_T(Const, invL, temp, 6, 6, 6, 6);
  AperB(invL, temp, C, 6, 6, 6, 6);
  jacobi(C, 6, d, V, nrot);
  A_TperB(invL, V, sol, 6, 6, 6, 6);

  // Now normalize them
  for (int j = 1; j <= 6; j++) /* Scan columns */
  {
    double mod = 0.0;
    for (int i = 1; i <= 6; i++) mod += sol[i][j] * sol[i][j];
    for (int i = 1; i <= 6; i++) sol[i][j] /= sqrt(mod);
  }

  double zero = 10e-20;
  double minev = 10e+20;
  int solind = 0;
  int i;
  switch (mode) {
    case (BOOKSTEIN):  // smallest eigenvalue
      for (i = 1; i <= 6; i++)
        if (d[i] < minev && fabs(d[i]) > zero) solind = i;
      break;

    case (FPF):
      for (i = 1; i <= 6; i++)
        if (d[i] < 0 && fabs(d[i]) > zero) solind = i;
  }

  // if (solind == 0) return false;

  // Now fetch the right solution
  // for (int j = 1; j <= 6; j++) pResult->coeff[j] = sol[j][solind];

  A1 = sol[1][solind];
  B1 = sol[2][solind];
  C1 = sol[3][solind];
  D1 = sol[4][solind];
  E1 = sol[5][solind];
  F1 = sol[6][solind];

  // CONVERTING THE CONIC EQUATION TO THE ELLIPSE EQUATION
  // Normalize Coefficients
  B1 /= A1;
  C1 /= A1;
  D1 /= A1;
  E1 /= A1;
  F1 /= A1;
  A1 /= A1;

  if (B1 == 0)  // Then not need to rotate the axes
  {
    A2 = A1;
    B2 = B1;
    C2 = C1;
    D2 = D1;
    E2 = E1;
    F2 = F1;

    // bRotation = false;
  } else if (B1 != 0)  // Rotate the axes
  {
    // bRotation = true;

    // Determine the rotation angle (in radians)
    rotation = atan(B1 / (A1 - C1)) / 2;

    // Compute the coefficients wrt the new coordinate system
    A2 = 0.5 * (A1 * (1 + cos(2 * rotation) + B1 * sin(2 * rotation) +
                      C1 * (1 - cos(2 * rotation))));
    B2 = (C1 - A1) * sin(2 * rotation) +
         B1 * cos(2 * rotation);  // B2 should turn to be zero?
    C2 = 0.5 * (A1 * (1 - cos(2 * rotation) - B1 * sin(2 * rotation) +
                      C1 * (1 + cos(2 * rotation))));
    D2 = D1 * cos(rotation) + E1 * sin(rotation);
    E2 = -D1 * sin(rotation) + E1 * cos(rotation);
    F2 = F1;
    // printf("Rotation in degrees: %.2f\n\n", rotation * 180 / pi);
  }

  // Transform the conic equation into the ellipse form
  D3 = D2 / A2;  // normalize x term's coef
  // A3 = 1;     //A2 / A2

  E3 = E2 / C2;  // normalize y term's coef
  // C3 = 1;     //C2 / C2

  cX = -(D3 / 2);  // center X
  cY = -(E3 / 2);  // center Y

  F3 = A2 * (cX * cX) + C2 * (cY * cY) - F2;

  // semimajor axis
  a = sqrt(F3 / A2);
  // semiminor axis
  b = sqrt(F3 / C2);

  // SET a2 - b2 !!!
  a2_b2 = a * a - b * b;

  // Center coordinates have to be re-transformed if rotation is applied!
  if (rotation != 0) {
    double tmpX = cX, tmpY = cY;
    cX = tmpX * cos(rotation) - tmpY * sin(rotation);
    cY = tmpX * sin(rotation) + tmpY * cos(rotation);
  }

  DeallocateMatrix(D, noPoints + 1);
  DeallocateMatrix(S, 7);
  DeallocateMatrix(Const, 7);
  DeallocateMatrix(temp, 7);
  DeallocateMatrix(L, 7);
  DeallocateMatrix(C, 7);
  DeallocateMatrix(invL, 7);
  delete d;
  DeallocateMatrix(V, 7);
  DeallocateMatrix(sol, 7);
}

customEllipse::customEllipse(double coefs[6]) {
  // Initializations
  InitParams();

  A1 = coefs[0];
  B1 = coefs[1];
  C1 = coefs[2];
  D1 = coefs[3];
  E1 = coefs[4];
  F1 = coefs[5];

  // CONVERTING THE CONIC EQUATION TO THE ELLIPSE EQUATION
  // Normalize Coefficients
  B1 /= A1;
  C1 /= A1;
  D1 /= A1;
  E1 /= A1;
  F1 /= A1;
  A1 /= A1;

  if (B1 == 0)  // Then not need to rotate the axes
  {
    A2 = A1;
    B2 = B1;
    C2 = C1;
    D2 = D1;
    E2 = E1;
    F2 = F1;
  } else if (B1 != 0)  // Rotate the axes
  {
    // Determine the rotation angle (in radians)
    rotation = atan(B1 / (A1 - C1)) / 2;

    // Compute the coefficients wrt the new coordinate system
    A2 = 0.5 * (A1 * (1 + cos(2 * rotation) + B1 * sin(2 * rotation) +
                      C1 * (1 - cos(2 * rotation))));
    B2 = (C1 - A1) * sin(2 * rotation) +
         B1 * cos(2 * rotation);  // B2 should turn to be zero?
    C2 = 0.5 * (A1 * (1 - cos(2 * rotation) - B1 * sin(2 * rotation) +
                      C1 * (1 + cos(2 * rotation))));
    D2 = D1 * cos(rotation) + E1 * sin(rotation);
    E2 = -D1 * sin(rotation) + E1 * cos(rotation);
    F2 = F1;
    // printf("Rotation in degrees: %.2f\n\n", rotation * 180 / pi);
  }

  // Transform the conic equation into the ellipse form
  D3 = D2 / A2;  // normalize x term's coef
  // A3 = 1;     //A2 / A2

  E3 = E2 / C2;  // normalize y term's coef
  // C3 = 1;     //C2 / C2

  cX = -(D3 / 2);  // center X
  cY = -(E3 / 2);  // center Y

  F3 = A2 * (cX * cX) + C2 * (cY * cY) - F2;

  // semimajor axis
  a = sqrt(F3 / A2);
  // semiminor axis
  b = sqrt(F3 / C2);

  // SET a2 - b2 !!!
  a2_b2 = a * a - b * b;

  // Center coordinates have to be re-transformed if rotation is applied!
  if (rotation != 0) {
    double tmpX = cX, tmpY = cY;
    cX = tmpX * cos(rotation) - tmpY * sin(rotation);
    cY = tmpX * sin(rotation) + tmpY * cos(rotation);
  }
}

customEllipse::~customEllipse() {
  if (xPoints != NULL) delete[] xPoints;
  if (yPoints != NULL) delete[] yPoints;
  // if (fitPoints != NULL) delete[] fitPoints;
  if (drawPoints != NULL) delete[] drawPoints;
  if (estRadians != NULL) delete[] estRadians;
  if (closePoints != NULL) delete[] closePoints;
}

pix customEllipse::GetCenter() {
  pix tmp;
  tmp.x = (int)cX;
  tmp.y = (int)-cY;
  return tmp;
}

double customEllipse::GetCenterX() { return cX; }
double customEllipse::GetCenterY() {
  // dont forget to re-transform the y axis!
  return -cY;
}
double customEllipse::GetSemiMajorAxis() { return a; }
double customEllipse::GetSemiMinorAxis() { return b; }

double customEllipse::GetRotation() { return rotation; }
void customEllipse::GetCoefficients(double *coefs) {
  coefs[0] = A1;
  coefs[1] = B1;
  coefs[2] = C1;
  coefs[3] = D1;
  coefs[4] = E1;
  coefs[5] = F1;
}
double customEllipse::GetPerimeter() {
  // if perimeter has already been computed,
  // then just return it!
  if (perimeter != 0) return perimeter;

  // Perimeter Computation(s)
  double h = pow((a - b), 2.0) / pow((a + b), 2.0);

  // Ramajunan II ellipse perimeter estimation
  perimeter = pi * (a + b) * (1 + 3 * h / (10 + sqrt(4 - 3 * h)));

  return perimeter;
}
pix *customEllipse::DrawEllipse(int resolution) {
  if (drawPoints != NULL) return drawPoints;

  drawPoints = (pix *)malloc(resolution * sizeof(pix));

  if (resolution % 2) resolution--;
  int npts = resolution / 2;

  double **u = AllocateMatrix(3, npts + 1);     // new double[3][npts + 1];
  double **Aiu = AllocateMatrix(3, npts + 1);   // new double[3][npts + 1];
  double **L = AllocateMatrix(3, npts + 1);     // new double[3][npts + 1];
  double **B = AllocateMatrix(3, npts + 1);     // new double[3][npts + 1];
  double **Xpos = AllocateMatrix(3, npts + 1);  // new double[3][npts + 1];
  double **Xneg = AllocateMatrix(3, npts + 1);  // new double[3][npts + 1];
  double **ss1 = AllocateMatrix(3, npts + 1);   // new double[3][npts + 1];
  double **ss2 = AllocateMatrix(3, npts + 1);   // new double[3][npts + 1];
  double *lambda = new double[npts + 1];
  double **uAiu = AllocateMatrix(3, npts + 1);  // new double[3][npts + 1];
  double **A = AllocateMatrix(3, 3);
  double **Ai = AllocateMatrix(3, 3);
  double **Aib = AllocateMatrix(3, 2);
  double **b = AllocateMatrix(3, 2);
  double **r1 = AllocateMatrix(2, 2);
  double Ao, Ax, Ay, Axx, Ayy, Axy;

  // double pi = 3.14781;
  double theta;
  int i;
  int j;
  double kk;

  memset(lambda, 0, sizeof(double) * (npts + 1));

  Ao = F1;   // pvec[6];
  Ax = D1;   // pvec[4];
  Ay = E1;   // pvec[5];
  Axx = A1;  // pvec[1];
  Ayy = C1;  // pvec[3];
  Axy = B1;  // pvec[2];

  A[1][1] = Axx;
  A[1][2] = Axy / 2;
  A[2][1] = Axy / 2;
  A[2][2] = Ayy;
  b[1][1] = Ax;
  b[2][1] = Ay;

  // Generate normals linspace
  for (i = 1, theta = 0.0; i <= npts; i++, theta += (pi / npts)) {
    u[1][i] = cos(theta);
    u[2][i] = sin(theta);
  }

  inverse(A, Ai, 2);

  AperB(Ai, b, Aib, 2, 2, 2, 1);
  A_TperB(b, Aib, r1, 2, 1, 2, 1);
  r1[1][1] = r1[1][1] - 4 * Ao;

  AperB(Ai, u, Aiu, 2, 2, 2, npts);
  for (i = 1; i <= 2; i++)
    for (j = 1; j <= npts; j++) uAiu[i][j] = u[i][j] * Aiu[i][j];

  for (j = 1; j <= npts; j++) {
    if ((kk = (r1[1][1] / (uAiu[1][j] + uAiu[2][j]))) >= 0.0)
      lambda[j] = sqrt(kk);
    else
      lambda[j] = -1.0;
  }

  // Builds up B and L
  for (j = 1; j <= npts; j++) L[1][j] = L[2][j] = lambda[j];
  for (j = 1; j <= npts; j++) {
    B[1][j] = b[1][1];
    B[2][j] = b[2][1];
  }

  for (j = 1; j <= npts; j++) {
    ss1[1][j] = 0.5 * (L[1][j] * u[1][j] - B[1][j]);
    ss1[2][j] = 0.5 * (L[2][j] * u[2][j] - B[2][j]);
    ss2[1][j] = 0.5 * (-L[1][j] * u[1][j] - B[1][j]);
    ss2[2][j] = 0.5 * (-L[2][j] * u[2][j] - B[2][j]);
  }

  AperB(Ai, ss1, Xpos, 2, 2, 2, npts);
  AperB(Ai, ss2, Xneg, 2, 2, 2, npts);

  for (j = 1; j <= npts; j++) {
    if (lambda[j] == -1.0) {
      drawPoints[j - 1].x = -1;
      drawPoints[j - 1].y = -1;
      drawPoints[j - 1 + npts].x = -1;
      drawPoints[j - 1 + npts].y = -1;
    } else {
      drawPoints[j - 1].x = (int)(Xpos[1][j]);
      drawPoints[j - 1].y = (int)(Xpos[2][j]);
      drawPoints[j - 1 + npts].x = (int)(Xneg[1][j]);
      drawPoints[j - 1 + npts].y = (int)(Xneg[2][j]);
    }
  }

  // transform the coordinate system -y to y
  for (int i = 0; i < resolution; i++) drawPoints[i].y = -drawPoints[i].y;

  DeallocateMatrix(u, 3);
  DeallocateMatrix(Aiu, 3);
  DeallocateMatrix(L, 3);
  DeallocateMatrix(B, 3);
  DeallocateMatrix(Xpos, 3);
  DeallocateMatrix(Xneg, 3);
  DeallocateMatrix(ss1, 3);
  DeallocateMatrix(ss2, 3);
  delete lambda;
  DeallocateMatrix(uAiu, 3);
  DeallocateMatrix(A, 3);
  DeallocateMatrix(Ai, 3);
  DeallocateMatrix(Aib, 3);
  DeallocateMatrix(b, 3);
  DeallocateMatrix(r1, 2);

  return drawPoints;
}

inline double customEllipse::DistanceBwPoints(pix p1, pix p2) {
  int xDist, yDist;
  double distance;

  xDist = p1.x - p2.x;
  yDist = p1.y - p2.y;

  distance = xDist * xDist + yDist * yDist;
  return sqrt(distance);
}

inline double customEllipse::DistanceBwPoints(double x1, double y1, double x2,
                                              double y2) {
  double xDist, yDist, distance;

  xDist = x1 - x2;
  yDist = y1 - y2;

  distance = xDist * xDist + yDist * yDist;
  return sqrt(distance);
}
inline double customEllipse::SquaredDistanceBwPoints(pix p1, pix p2) {
  int xDist, yDist;
  double distance;

  xDist = p1.x - p2.x;
  yDist = p1.y - p2.y;

  distance = xDist * xDist + yDist * yDist;
  return distance;
}

inline double customEllipse::SquaredDistanceBwPoints(double x1, double y1,
                                                     double x2, double y2) {
  double xDist, yDist, distance;

  xDist = x1 - x2;
  yDist = y1 - y2;

  distance = xDist * xDist + yDist * yDist;
  return distance;
}

// Returns (F / F') for one NR iteration
inline double customEllipse::NewtonRaphsonIteration(double theta, double x,
                                                    double y) {
  // compute the frequently used terms for the sake of performance
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);

  // dikkat:there is also 'r' in the term but is always 1 in our case
  double fTheta = a2_b2 * cosTheta * sinTheta;
  fTheta = fTheta - x * a * sinTheta + y * b * cosTheta;

  double fPrimeTheta = a2_b2 * (cosTheta * cosTheta - sinTheta * sinTheta);
  fPrimeTheta = fPrimeTheta - x * a * cosTheta - y * b * sinTheta;

  return (fTheta / fPrimeTheta);
}

// the num of iterations can be varied for higher accuracies
double customEllipse::NewtonRaphsonThetaEstimation(double th0, double x,
                                                   double y) {
  double th1 = th0 - NewtonRaphsonIteration(th0, x, y);
  double th2 = th1 - NewtonRaphsonIteration(th1, x, y);
  // double th3 = th2 - NewtonRaphsonIteration(th2, x, y);
  // double th4 = th3 - NewtonRaphsonIteration(th3, x, y);

  return th2;
}

double customEllipse::GetDistance(pix point, double &estimation) {
  // for different initial values for 4 estimations (in radians)
  double th0[4], estimations[4];

  // closest point candidates and distances for different estimations
  double distances[4], x, y;
  pix candidates[4];
  pix transPnt;  // translated and rotated point

  // translate & rotate candidate point coordinates
  // Burak - added typecast to supress warning
  transPnt.x = (int)(point.x - cX);
  transPnt.y = (int)(point.y - cY);

  if (rotation != 0) {
    x = transPnt.x * cos(-rotation) - transPnt.y * sin(-rotation);
    y = transPnt.x * sin(-rotation) + transPnt.y * cos(-rotation);

    transPnt.x = (int)x;
    transPnt.y = (int)y;
  }

  th0[0] = atan(a * y / b * x);  // ERROR IHTIMALI b*x=0?
  th0[1] = th0[0] + 1.5707;      // pi * 0.5;
  th0[2] = th0[0] + pi;          // pi * 1.0
  th0[3] = th0[0] + 4.7123;      // pi * 1.5

  int minDistPnt = 0;
  for (int i = 0; i < 4; i++) {
    estimations[i] = NewtonRaphsonThetaEstimation(th0[i], x, y);

    candidates[i].x = (int)(a * cos(estimations[i]));
    candidates[i].y = (int)(b * sin(estimations[i]));

    distances[i] = DistanceBwPoints(transPnt, candidates[i]);

    if (distances[i] < distances[minDistPnt]) minDistPnt = i;
  }
  estimation = estimations[minDistPnt];
  return distances[minDistPnt];
}

double customEllipse::GetDistance(double pX, double pY, double &estimation) {
  // for different initial values for 4 estimations (in radians)
  double th0[4], estimations[4];

  // closest point candidates and distances for different estimations
  double distances[4], x, y;
  double candidatesX[4], candidatesY[4];
  double transPntX, transPntY;  // translated and rotated point

  // translate & rotate candidate point coordinates
  transPntX = pX - cX;
  transPntY = pY - cY;

  if (rotation != 0) {
    x = transPntX * cos(-rotation) - transPntY * sin(-rotation);
    y = transPntX * sin(-rotation) + transPntY * cos(-rotation);

    transPntX = x;
    transPntY = y;
  }

  th0[0] = atan(a * y / b * x);  // ERROR IHTIMALI b*x=0?
  th0[1] = th0[0] + 1.5707;      // pi * 0.5;
  th0[2] = th0[0] + pi;          // pi * 1.0
  th0[3] = th0[0] + 4.7123;      // pi * 1.5

  int minDistPnt = 0;
  for (int i = 0; i < 4; i++) {
    estimations[i] = NewtonRaphsonThetaEstimation(th0[i], x, y);

    candidatesX[i] = a * cos(estimations[i]);
    candidatesY[i] = b * sin(estimations[i]);

    distances[i] =
        DistanceBwPoints(transPntX, transPntY, candidatesX[i], candidatesY[i]);

    if (distances[i] < distances[minDistPnt]) minDistPnt = i;
  }
  estimation = estimations[minDistPnt];
  return distances[minDistPnt];
}

double customEllipse::GetSquaredDistance(pix point, double &estimation) {
  // for different initial values for 4 estimations (in radians)
  double th0[4], estimations[4];

  // closest point candidates and distances for different estimations
  double sqDistances[4], x, y;
  pix candidates[4];
  pix transPnt;  // translated and rotated point

  // translate & rotate candidate point coordinates
  // Burak - added typecast to supress warning
  transPnt.x = (int)(point.x - cX);
  transPnt.y = (int)(point.y - cY);

  if (rotation != 0) {
    x = transPnt.x * cos(-rotation) - transPnt.y * sin(-rotation);
    y = transPnt.x * sin(-rotation) + transPnt.y * cos(-rotation);

    transPnt.x = (int)x;
    transPnt.y = (int)y;
  }

  th0[0] = atan(a * y / b * x);  // ERROR IHTIMALI b*x=0?
  th0[1] = th0[0] + 1.5707;      // pi * 0.5;
  th0[2] = th0[0] + pi;          // pi * 1.0
  th0[3] = th0[0] + 4.7123;      // pi * 1.5

  int minDistPnt = 0;
  for (int i = 0; i < 4; i++) {
    estimations[i] = NewtonRaphsonThetaEstimation(th0[i], x, y);

    candidates[i].x = (int)(a * cos(estimations[i]));
    candidates[i].y = (int)(b * sin(estimations[i]));

    sqDistances[i] = SquaredDistanceBwPoints(transPnt, candidates[i]);

    if (sqDistances[i] < sqDistances[minDistPnt]) minDistPnt = i;
  }
  estimation = estimations[minDistPnt];
  return sqDistances[minDistPnt];
}
double customEllipse::GetSquaredDistance(double pX, double pY,
                                         double &estimation) {
  // for different initial values for 4 estimations (in radians)
  double th0[4], estimations[4];

  // closest point candidates and distances for different estimations
  double distances[4], x, y;
  double candidatesX[4], candidatesY[4];
  double transPntX, transPntY;  // translated and rotated point

  // translate & rotate candidate point coordinates
  transPntX = pX - cX;
  transPntY = pY - cY;

  if (rotation != 0) {
    x = transPntX * cos(-rotation) - transPntY * sin(-rotation);
    y = transPntX * sin(-rotation) + transPntY * cos(-rotation);

    transPntX = x;
    transPntY = y;
  }

  th0[0] = atan(a * y / b * x);  // ERROR IHTIMALI b*x=0?
  th0[1] = th0[0] + 1.5707;      // pi * 0.5;
  th0[2] = th0[0] + pi;          // pi * 1.0
  th0[3] = th0[0] + 4.7123;      // pi * 1.5

  int minDistPnt = 0;
  for (int i = 0; i < 4; i++) {
    estimations[i] = NewtonRaphsonThetaEstimation(th0[i], x, y);

    candidatesX[i] = a * cos(estimations[i]);
    candidatesY[i] = b * sin(estimations[i]);

    distances[i] = SquaredDistanceBwPoints(transPntX, transPntY, candidatesX[i],
                                           candidatesY[i]);

    if (distances[i] < distances[minDistPnt]) minDistPnt = i;
  }
  estimation = estimations[minDistPnt];
  return distances[minDistPnt];
}

double customEllipse::GetAverageFittingError() {
  if (fitError != 0) return fitError;

  double tmpEst;  // estimated radian value for the closest point comp.
  estRadians = (double *)malloc(noPoints * sizeof(double));

  // total distance of fitting points to the ellipse curve
  for (int i = 0; i < noPoints; i++) {
    if (fitPoints != NULL)  // integer form
    {
      fitError += GetDistance(fitPoints[i], tmpEst);
      estRadians[i] = tmpEst;
    } else  // double form
    {
      fitError += GetDistance(xPoints[i], yPoints[i], tmpEst);
      estRadians[i] = tmpEst;
    }
  }

  // average fitting error per pixel
  fitError /= noPoints;

  return fitError;
}

double customEllipse::GetRmsFittingError() {
  if (rmsError != 0) return rmsError;

  double tmpEst;  // estimated radian value for the closest point computation
  estRadians = (double *)malloc(noPoints * sizeof(double));

  // total distance of fitting points to the ellipse curve
  for (int i = 0; i < noPoints; i++) {
    if (fitPoints != NULL)  // integer form
    {
      rmsError += GetSquaredDistance(fitPoints[i], tmpEst);
      estRadians[i] = tmpEst;
    } else  // double form
    {
      rmsError += GetSquaredDistance(xPoints[i], yPoints[i], tmpEst);
      estRadians[i] = tmpEst;
    }
  }

  // rmse fitting error per pixel
  rmsError = sqrt(rmsError / noPoints);

  return rmsError;
}

pix *customEllipse::GetClosestPoints() {
  if (closePoints != NULL) return closePoints;
  closePoints = (pix *)malloc(noPoints * sizeof(pix));

  // below function has to be invoked to obtain the estRadians
  //(it wont be executed if it already has been)
  GetAverageFittingError();

  pix tmp;
  double tmpX, tmpY;
  double sinTheta = sin(rotation);
  double cosTheta = cos(rotation);
  for (int i = 0; i < noPoints; i++) {
    tmpX = a * cos(estRadians[i]);
    tmpY = b * sin(estRadians[i]);

    tmp.x = (int)(tmpX * cosTheta - tmpY * sinTheta + cX);
    // re-transform th coordinate system : y = -y
    tmp.y = -(int)(tmpX * sinTheta + tmpY * cosTheta + cY);

    closePoints[i] = tmp;
  }
  return closePoints;
}

double customEllipse::GetClosestPointAndDistance(pix test, pix &closest) {
  double distance, estimation;

  // reflect y- axis
  test.y = -test.y;

  distance = GetDistance(test, estimation);

  double tmpX, tmpY;
  tmpX = a * cos(estimation);
  tmpY = b * sin(estimation);

  closest.x = (int)(tmpX * cos(rotation) - tmpY * sin(rotation) + cX);
  // re-transform th coordinate system : y = -y
  closest.y = -(int)(tmpX * sin(rotation) + tmpY * cos(rotation) + cY);

  return distance;
}
double customEllipse::GetClosestPointAndDistance(double testX, double testY,
                                                 pix &closest) {
  double distance, estimation;

  // reflect y axis
  testY = -testY;

  distance = GetDistance(testX, testY, estimation);

  double tmpX, tmpY;
  tmpX = a * cos(estimation);
  tmpY = b * sin(estimation);

  closest.x = (int)(tmpX * cos(rotation) - tmpY * sin(rotation) + cX);
  // re-transform th coordinate system : y = -y
  closest.y = -(int)(tmpX * sin(rotation) + tmpY * cos(rotation) + cY);

  return distance;
}

///---------------------------------------------------------------------
/// Fits a circle to a given set of points. There must be at least 2 points
/// The circle equation is of the form: (x-xc)^2 + (y-yc)^2 = r^2
/// Returns true if there is a fit, false in case no circles can be fit
///
bool CircleFit(const vector<double> &Xs, const vector<double> &Ys,
               double &centerX, double &centerY, double &radius) {
  int N = Xs.size();

  if (N < 3) return false;

  double xAvg = 0;
  double yAvg = 0;

  for (int i = 0; i < N; i++) {
    xAvg += Xs[i];
    yAvg += Ys[i];
  }

  xAvg /= N;
  yAvg /= N;

  double Suu = 0;
  double Suv = 0;
  double Svv = 0;
  double Suuu = 0;
  double Suvv = 0;
  double Svvv = 0;
  double Svuu = 0;

  for (int i = 0; i < N; i++) {
    double u = Xs[i] - xAvg;
    double v = Ys[i] - yAvg;

    Suu += u * u;
    Suv += u * v;
    Svv += v * v;
    Suuu += u * u * u;
    Suvv += u * v * v;
    Svvv += v * v * v;
    Svuu += v * u * u;
  }

  // Now, we solve for the following linear system of equations
  // Av = b, where v = (uc, vc) is the center of the circle
  //
  // |N    Suv| |uc| = |b1|
  // |Suv  Svv| |vc| = |b2|
  //
  // where b1 = 0.5*(Suuu+Suvv) and b2 = 0.5*(Svvv+Svuu)
  //

  double detA = Suu * Svv - Suv * Suv;
  if (detA == 0) return false;

  double b1 = 0.5 * (Suuu + Suvv);
  double b2 = 0.5 * (Svvv + Svuu);

  double uc = (Svv * b1 - Suv * b2) / detA;
  double vc = (Suu * b2 - Suv * b1) / detA;

  radius = sqrt(uc * uc + vc * vc + (Suu + Svv) / N);

  centerX = uc + xAvg;
  centerY = vc + yAvg;

  // Compute mean square error
  double error = 0;
  for (int i = 0; i < N; i++) {
    double dx = Xs[i] - centerX;
    double dy = Ys[i] - centerY;
    double d = sqrt(dx * dx + dy * dy) - radius;
    error += d * d;
  }
  error /= N;

  // this checks if the input pixels are circle-shaped
  // radius is 0.4, if we tolerate 5% deviation, MSE must be at most 0.4 * 0.05
  // = 0.02 NOTE: I changed this 0.02 -> 0.0002, because observed deviation
  // smaller than expected
  if (error > 0.0002) return false;

  return true;
}  // end-CircleFit

void customEllipse::getEllipseSamples(int noOfSamples, vector<double> &xPoints,
                                      vector<double> &yPoints) {
  xPoints = vector<double>(noOfSamples);
  yPoints = vector<double>(noOfSamples);

  double angleStep = (double)360 / noOfSamples;

  double angle = 0;

  double sinRot = sin(rotation);
  double cosRot = cos(rotation);

  // Calculate unrotated ellipse Points and then rotate them
  for (int i = 0; i < noOfSamples; i++) {
    xPoints[i] = a * FastSinHP((angle * pi / 180));
    yPoints[i] = b * FastCosHP((angle * pi / 180));

    if (rotation != 0) {
      double x, y;

      x = cX + xPoints[i] * cosRot - yPoints[i] * sinRot;
      y = cY + xPoints[i] * sinRot + yPoints[i] * cosRot;

      xPoints[i] = x;
      yPoints[i] = y;
    }

    yPoints[i] = -yPoints[i];
    angle += angleStep;
  }
}

inline double FastSinHP(double x)  // High presicion 14x faster
{
  double res = 0;

  // always wrap input angle to -PI..PI
  if (x < -3.14159265)
    x += 6.28318531;
  else if (x > 3.14159265)
    x -= 6.28318531;

  // compute sine
  if (x < 0) {
    res = 1.27323954 * x + .405284735 * x * x;

    if (res < 0)
      res = .225 * (res * -res - res) + res;
    else
      res = .225 * (res * res - res) + res;
  } else {
    res = 1.27323954 * x - 0.405284735 * x * x;

    if (res < 0)
      res = .225 * (res * -res - res) + res;
    else
      res = .225 * (res * res - res) + res;
  }

  return res;
}

inline double FastCosHP(double x) {
  double res = 0;

#if 1  // safer
  x += 1.57079632;
  // always wrap input angle to -PI..PI
  if (x < -3.14159265)
    x += 6.28318531;
  else if (x > 3.14159265)
    x -= 6.28318531;

#else  // slightly faster
  x += 1.57079632;
  if (x > 3.14159265) x -= 6.28318531;

#endif

  if (x < 0) {
    res = 1.27323954 * x + 0.405284735 * x * x;

    if (res < 0)
      res = .225 * (res * -res - res) + res;
    else
      res = .225 * (res * res - res) + res;
  } else {
    res = 1.27323954 * x - 0.405284735 * x * x;

    if (res < 0)
      res = .225 * (res * -res - res) + res;
    else
      res = .225 * (res * res - res) + res;
  }

  return res;
}