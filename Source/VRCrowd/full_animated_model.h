#pragma once
//#define RK4 1
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
#ifndef WALKER_PARAMS_DEFINED
#define WALKER_PARAMS_DEFINED
struct WalkerParams {
  float a;
  float al;
  float p;
  float pl;
  float omega;
  float omegal;
  float nu;
  float lambda;
  float lambdal;
  float m;
  float L;
  float fwdspeed;
};
#endif
typedef double REAL;
REAL sq(REAL x) { return x * x; }
REAL random_normal(REAL mu, REAL sigma) {
  static std::default_random_engine generator(0);
  std::normal_distribution<REAL> d(mu, sigma);
  return d(generator);
}

class System {
private:
  REAL hv, omegav, hl, omegal, M;

  std::vector<REAL> F;
public:
  int X_INDEX(int i) { return i * 4 + 4; }
  int V_INDEX(int i) { return i * 4 + 5; }
  int Y_INDEX(int i) { return i * 4 + 6; }
  int U_INDEX(int i) { return i * 4 + 7; }
  REAL foot_force(REAL X, REAL xDot, WalkerParams w) {
    if (X < 0) {
      return w.lambdal * xDot *
                 (xDot * xDot - w.nu * w.nu * ((X + w.pl) * (X + w.pl)) +
                  w.nu * w.nu * w.al * w.al) -
             (w.omegal * w.omegal) * (X + w.pl);
    } else {
      return w.lambdal * xDot *
                 (xDot * xDot - w.nu * w.nu * ((X - w.pl) * (X - w.pl)) +
                  w.nu * w.nu * w.al * w.al) -
             (w.omegal * w.omegal) * (X - w.pl);
    }
  }

  void f(REAL *y, REAL *dy, REAL t, WalkerParams *ws, float *u,
         bool full_vertical_model, int n) {

    REAL k = omegav * omegav;
    REAL g = 9.81;

    REAL x = y[0];
    REAL xdot = y[1];
    REAL H = 0, sum2 = 0;
    REAL msum = 0;
    for (int i = 0; i < n; ++i) {
      msum += ws[i].m;
    }
    REAL Fsum = 0.0;
    if (full_vertical_model) {
        for (int i = 0; i < n; ++i) {
            REAL p = ws[i].p;
            REAL m = ws[i].m;
            REAL a = ws[i].a;
            REAL phi = y[X_INDEX(i)];
            REAL phidot = y[V_INDEX(i)];
            int quotient = int(phi / p);
            phi = fmod(phi, p);
            phi -= p * (quotient % 2 == 1);

            REAL dot_phi = phidot;
            REAL dot_x = xdot;

            REAL w = ws[i].omega;
            REAL mu = m / (M + m);
            //     REAL l = ws[i].L;
            REAL l = std::sqrt(
                std::max(sq(ws[i].L) - sq(y[Y_INDEX(i)] - u[i]), (REAL)0.0));
            REAL L = l;
            REAL lambda = ws[i].lambda * m * l * l;

            REAL w_g = pow(w, 2.0) / g;
            REAL mwg = mu * w_g;
            sum2 += (sq(m) * sq(L)) * sq(sin(phi));
            REAL Mfunc = lambda *
                (sq(phidot) / 2 - sq(w) * (1 - cos(phi)) - sq(a) / 2) *
                dot_phi;

            H += -sq(m) * g * sq(L) * sq(sin(phi)) + m * L * sin(phi) * Mfunc -
                m * L * phidot * sq(cos(phi));

            dy[X_INDEX(i)] = y[V_INDEX(i)];
        }
    }
    H *= full_vertical_model;
    if (full_vertical_model) {
        dy[1] =
            (H - xdot * hv * (msum + M) - k * (msum + M) * x) / (M + msum - sum2);


        REAL xddot = dy[1];
        dy[0] = y[1];
        for (int i = 0; i < n; ++i) {
            REAL m = ws[i].m;
            REAL a = ws[i].a;
            REAL p = ws[i].p;
            REAL phi = y[X_INDEX(i)];
            REAL phidot = y[V_INDEX(i)];
            int quotient = int(phi / p);
            phi = fmod(phi, p);
            phi -= p * (quotient % 2 == 1);
            REAL dot_phi = phidot;
            REAL dot_x = xdot;
            REAL w = ws[i].omega;
            REAL mu = m / (M + m);
            REAL l =
                std::sqrt(std::max(sq(ws[i].L) - sq(y[Y_INDEX(i)] - u[i]), (REAL)0));
            if (l == 0) {
                l = ws[i].L;
            }
            REAL L = l;
            REAL lambda = ws[i].lambda * m * l * l;
            REAL w_g = (w * w) / g;
            REAL mwg = mu * w_g;
            REAL Mfunc = lambda *
                (sq(phidot) / 2 - sq(w) * (1 - cos(phi)) - sq(a) / 2) *
                dot_phi;

            REAL phiddot = (m * g - m * xddot) * L * sin(phi) - Mfunc;

            dy[V_INDEX(i)] = phiddot;
        }
    }
	F.resize(n);
    Fsum = 0;
    for (int i = 0; i < n; ++i) {
      dy[Y_INDEX(i)] = y[U_INDEX(i)];
      REAL Y = y[Y_INDEX(i)];
      REAL Omegap = std::sqrt(9.81 / ws[i].L);
      F[i] = Omegap * Omegap * (u[i] - Y);
      Fsum += ws[i].m * F[i] / (M);
    }
    dy[3] = (Fsum - 2 * hl * omegal * y[3] - omegal * omegal * y[2]);
    for (int i = 0; i < n; ++i) {
      dy[U_INDEX(i)] = -F[i] - dy[3];
    }
    dy[2] = y[3];
  }

  std::vector<REAL> x1, x00, x0;
  std::vector<REAL> K[4], s[4];
  void integrate(REAL T, std::vector<WalkerParams> parms, REAL hv_,
                 REAL omegav_, REAL M_, REAL hl_, REAL omegal_, float *y0,
                 float *u, float *bmin, float *tnext, REAL t0, bool adapt,
                 bool vertical, int n) {
    this->hv = hv_;
    this->omegav = omegav_;
    this->omegal = omegal_;
    this->hl = hl_;
    this->M = M_;
    int n0 = x0.size() / 4 - 1;
    x1.resize(n * 4 + 4);
    x0.resize(n * 4 + 4);
    x00.resize(n * 4 + 4);
    for (int i = 0; i < n * 4 + 4; ++i) {
      x1[i] = y0[i];
      x0[i] = y0[i];
      x00[i] = y0[i];
    }
    for (int j = 0; j < 4; ++j) {

      K[j].resize(n * 4 + 4);
      s[j].resize(n * 4 + 4);
    };
    constexpr REAL h0 = 1e-2;
    constexpr REAL hmin = 1e-3;
    REAL hmax = T;
    REAL h = h0;
    REAL tolerance = 1e-3;
    REAL t = 0;
    while (t < T) {
      float tnextmin = FLT_MAX;
      for (int i = 0; i < n * 4 + 4; ++i) {
          tnextmin = std::min(tnextmin, tnext[i]);
      }
      h = std::min(hmax, std::max(hmin, (tnextmin - t-t0) / 10.0));
      while (t + t0 < tnextmin) {
          
          for (int i = 0; i < n * 4 + 4; ++i) {

              for (int j = 0; j < 4; ++j) {
                  s[j][i] = 0;
                  K[j][i] = 0;
              }
          }
#ifdef RK4
          f(x1.data(), K[0].data(), t, parms.data(), u, vertical, n);
          for (int i = 0; i < n * 4 + 4; ++i)
              s[0][i] = x1[i] + K[0][i] * h / 2.0;
          f(s[0].data(), K[1].data(), t + h / 2, parms.data(), u, vertical, n);
          for (int i = 0; i < n * 4 + 4; ++i)
              s[1][i] = x1[i] + K[1][i] * h / 2.0;
          f(s[1].data(), K[2].data(), t + h / 2, parms.data(), u, vertical, n);
          for (int i = 0; i < n * 4 + 4; ++i)
              s[2][i] = x1[i] + K[2][i] * h;
          f(s[2].data(), K[3].data(), t + h, parms.data(), u, vertical, n);
          for (int i = 0; i < n * 4 + 4; ++i) {
              x1[i] += h / 6 * (K[0][i] + 2 * (K[1][i] + K[2][i]) + K[3][i]);
          }
#else
          if (t == 0) {
              f(x0.data(), K[0].data(), t, parms.data(), u, vertical, n);
              for (int i = 0; i < n * 4 + 4; ++i)
                  s[0][i] = x1[i] + 0.5 * K[0][i] * h;
          }
          f(s[0].data(), K[1].data(), t + h * 0.5, parms.data(), u, vertical, n);
          for (int i = 0; i < n * 4 + 4; ++i)
              s[1][i] = x1[i] + 0.75 * h * (K[1][i]);

          f(s[1].data(), K[2].data(), t + h * 3 / 4.0, parms.data(), u, vertical,
              n);
          for (int i = 0; i < n * 4 + 4; ++i)
              s[0][i] = x1[i] +
              h * (K[0][i] * 2 / 9.0 + K[1][i] / 3.0 + 4.0 / 9.0 * K[2][i]);

          f(s[0].data(), K[0].data(), t + h, parms.data(), u, vertical, n);

          REAL err = 0.0;
          for (int i = 0; i < n * 4 + 4; ++i) {
              REAL wi = s[0][i];
              REAL zi = x1[i] + h * (7 / 24.0 * K[0][i] + 0.25 * K[1][i] +
                  1 / 3.0 * K[2][i] + K[0][i] / 8.0);
              err = std::max(std::abs(zi - wi), err);
          }
          h *= std::min(std::max(std::sqrt(tolerance / err), (REAL)0.2), (REAL)5.0);
          h = std::min(hmax, std::max(h, hmin));
          if (err > 2.0 * tolerance && h > 2.0 * hmin) {
              continue;
          }
          for (int i = 0; i < n * 4 + 4; ++i) {
              x0[i] = x1[i];
              x1[i] = s[0][i];
          }
#endif

         
          t += h;
          if (t >= T) {
              break;
          }
      }
      for (int i = 0; i < n; ++i) {
        REAL Omegap = std::sqrt(9.81 / parms[i].L);
        REAL p = parms[i].p;
        REAL pl = parms[i].pl;
        REAL al = parms[i].al;
        bool is_reset = 0;
        if (vertical) {
            is_reset = std::fmod(x1[X_INDEX(i)] - p, p) < 0.0 &&
                std::fmod(x0[X_INDEX(i)] - p, p) > 0.0;
        }
        else {
            is_reset = tnext[i] <= t + t0 + 1e-6;
        }
        if (is_reset) {
          tnext[i] += 0.5 / parms[i].omegal + adapt*0.5/parms[i].omegal*std::max(-0.99,(92.0/2000.0)*(92/2000.0)-(x1[Y_INDEX(i)]-u[i])*(x1[Y_INDEX(i)]-u[i]))/(4*parms[i].fwdspeed*parms[i].fwdspeed);
          u[i] = x1[Y_INDEX(i)] + x1[U_INDEX(i)] / Omegap + bmin[i];
          bmin[i] = -bmin[i];
        }
      }
      t += 1e-6;
    }
    for (int i = 0; i < n * 4 + 4; ++i) {
      y0[i] = x1[i];
    }
  }
};
#ifdef TEST___INT
void test_integral_performance() {
  System s;
  WalkerParams p = {0.02, 0.02, 0.04, 0.04, 1.2, 0.8, 1, 1, 1, 70, 1, 1};
  int NTEST = 300;
  float *state = new float[4 * NTEST + 4];
  float *u = new float[NTEST], *bmin = new float[NTEST],
        *tnext = new float[NTEST];
  std::vector<WalkerParams> parms;
  parms.resize(NTEST);
  state[0] = 0;
  state[1] = 0;
  state[2] = 0;
  state[3] = 0;
  srand(9);
  for (int i = 0; i < NTEST; ++i) {
    parms[i] = p;
    parms[i].omega +=
        0.01 * (rand() % 100) * parms[i].omega - 0.005 * parms[i].omega;
    u[i] = 0.23;
    state[i * 4 + 4] = 0;
    state[i * 4 + 5] = 0.01;
    state[i * 4 + 6] = 0.0;
    state[i * 4 + 7] = 0.01;
    bmin[i] = (1 - 2 * (rand() % 2)) * 0.0157;
    tnext[i] = 0.5;
  }
  auto start = std::chrono::steady_clock::now();
  for (int frame = 0; frame < 120; ++frame) {

    //    printf("%f ", (float) state[0]);
    s.integrate(1 / 120.0, parms, 0.05, 4, 15000, 0.05, 4, state, u, bmin,
                tnext, frame / 120.0, true, true, NTEST);
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> T = end - start;
  printf("time: %lf, bridge: %f %f\n", T.count(), state[0], state[1]);
}
#endif
