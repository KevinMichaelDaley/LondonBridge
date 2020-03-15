#include "WalkerParams.h"
#include <algorithm>
#include <cstdio>
#include <random>
#include <vector>
double sq(double x) { return pow(double(x), double(2)); }
double random_normal(double mu, double sigma) {
    static std::default_random_engine generator(0);
    std::normal_distribution<double> d(mu, sigma);
    return d(generator);
}

class System {
private:
  double hv, omegav, hl, omegal, M;

public:
  int X_INDEX(int i) { return i * 4 + 4; }
  int V_INDEX(int i) { return i * 4 + 5; }
  int Y_INDEX(int i) { return i * 4 + 6; }
  int U_INDEX(int i) { return i * 4 + 7; }
  double foot_force(double X, double xDot, WalkerParams w) {
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

  void f(double* y, double* dy, double t,
         WalkerParams* ws, float* u, bool full_vertical_model, int n) {

    double k = omegav * omegav;
    double g = 9.81;

    double x = y[0];
    double xdot = y[1];
    double H = 0, sum2 = 0;
    double msum = 0;
    for (int i = 0; i < n; ++i) {
      msum += ws[i].m;
    }
    double F[n];
    double Fsum = 0.0;
    for (int i = 0; i < n; ++i) {
      double p = ws[i].p;
      double m = ws[i].m;
      double a = ws[i].a;
      double phi = y[X_INDEX(i)];
      double phidot = y[V_INDEX(i)];
      int quotient = int(phi / p);
      phi = fmod(phi, p);
      phi-=p*(quotient % 2 == 1) ;

      double dot_phi = phidot;
      double dot_x = xdot;

      double w = ws[i].omega;
      double mu = m / (M + m);
//     double l = ws[i].L;
      double l = std::sqrt(std::max(sq(ws[i].L)-sq(y[Y_INDEX(i)]-u[i]),0.0));
      if(l==0){
          l=ws[i].L;
      }
      double L = l;
      double lambda = 0.9 * m * l * l;

      double w_g = pow(w, 2.0) / g;
      double mwg = mu * w_g;
      sum2 += (sq(m) * sq(L)) * sq(sin(phi));
      double Mfunc = lambda *
                    (sq(phidot) / 2 - sq(w) * (1 - cos(phi)) - sq(a) / 2) *
                    dot_phi;

      H += -sq(m) * g * sq(L) * sq(sin(phi)) + m * L * sin(phi) * Mfunc -
           m * L * phidot * sq(cos(phi));

      dy[X_INDEX(i)] = y[V_INDEX(i)];
    }
    H*=full_vertical_model;

    dy[1] =
        (H - xdot * hv * (msum + M) - k * (msum + M) * x) / (M + msum - sum2);
    

    double xddot = dy[1];
    dy[0] = y[1];
    for (int i = 0; i < n; ++i) {
      double m = ws[i].m;
      double a = ws[i].a;
      double p = ws[i].p;
      double phi = y[X_INDEX(i)];
      double phidot = y[V_INDEX(i)];
      int quotient = int(phi / p);
      phi = fmod(phi, p);
      phi-=p*(quotient % 2 == 1) ;
      double dot_phi = phidot;
      double dot_x = xdot;
      double w = ws[i].omega;
      double mu = m / (M + m);
      double l = std::sqrt(std::max(sq(ws[i].L)-sq(y[Y_INDEX(i)]-u[i]),0.0));
      if(l==0){
          l=ws[i].L;
      }
      double L = l;
      double lambda = 0.9 * m * l * l;
      double w_g =(w*w) / g;
      double mwg = mu * w_g;
      double Mfunc = lambda *
                    (sq(phidot) / 2 - sq(w) * (1 - cos(phi)) - sq(a) / 2) *
                    dot_phi;

      double phiddot = (m * g - m * xddot) * L * sin(phi) - Mfunc;

      dy[V_INDEX(i)] = 0.01*phiddot;
    }
    

    for (int i = 0; i < n; ++i) {
      dy[Y_INDEX(i)] = y[U_INDEX(i)];
      double Y=y[Y_INDEX(i)];
      double Omegap=std::sqrt(9.81/ws[i].L);
      F[i] = Omegap*Omegap*(u[i]-Y);
      Fsum += ws[i].m * F[i] / (M + msum);
    }
    double r = (msum / (M + msum));
    dy[3] = (Fsum - 2 * hl *omegal * y[3] - omegal * omegal * y[2]);
    for (int i = 0; i < n; ++i) {
      dy[U_INDEX(i)] = -F[i] - dy[3];
    }
    dy[2] = y[3];
  }

  std::vector<double> x1, x00, x0;
  std::vector<double> K[4], s[4];
  void integrate(double T, std::vector<WalkerParams> parms, double hv_,
                 double omegav_, double M_, double hl_, double omegal_,
                 float* y0, float* u, float* bmin, float* tnext, double t0, bool adapt, bool vertical, int n){
    this->hv = hv_;
    this->omegav = omegav_;
    this->omegal = omegal_;
    this->hl = hl_;
    this->M = M_;
    x1.resize(n * 4 + 4);
    x0.resize(n * 4 + 4);
    for (int i = 0; i < n * 4 + 4; ++i) {
      x1[i] = y0[i];
      x0[i] = y0[i];
    }
    for (int j = 0; j < 4; ++j) {

          K[j].resize(n * 4 + 4);
          s[j].resize(n * 4 + 4);
    };
    constexpr double h = 1e-4;
    const int hpert=T/h;
    for (int ti = 0; ti < hpert; ti++) {
      double t=h*ti;
      for (int i = 0; i < n * 4 + 4; ++i) {

        x0[i] = x1[i];
        for(int j=0; j<4; ++j){
            s[j][i]=0;
            K[j][i]=0;
        }
      }
      f(x1.data(), K[0].data(), t, parms.data(),u,vertical,n);
      for (int i = 0; i < n * 4 + 4; ++i)
        s[0][i] = x1[i] + K[0][i] * h / 2.0;
      f(s[0].data(), K[1].data(), t + h / 2, parms.data(),u,vertical,n);
      for (int i = 0; i < n * 4 + 4; ++i)
        s[1][i] = x1[i] + K[1][i] * h / 2.0;
      f(s[1].data(), K[2].data(), t + h / 2, parms.data(),u,vertical,n);
      for (int i = 0; i < n * 4 + 4; ++i)
        s[2][i] = x1[i] + K[2][i] * h;
      f(s[2].data(), K[3].data(), t + h / 3, parms.data(),u,vertical,n);
      for (int i = 0; i < n * 4 + 4; ++i){
        x1[i] += h / 6 * (K[0][i] + 2 * (K[1][i] + K[2][i]) + K[3][i]);
      }
      for (int i = 0; i < n; ++i) {
          double Omegap=std::sqrt(9.81/parms[i].L);
          double p=parms[i].p;
          double pl=parms[i].pl;
          double al=parms[i].al;
          bool is_reset=std::fmod(x1[X_INDEX(i)]-p,p)<0.0 && std::fmod(x0[X_INDEX(i)]-p,p)>0.0;
          if(is_reset){
              tnext[i]+=0.5/parms[i].omegal;
              u[i]=x1[Y_INDEX(i)]+x1[U_INDEX(i)]/Omegap+bmin[i];
              

              bmin[i]=-bmin[i];
          }
      }
    }
    for (int i = 0; i < n * 4 + 4; ++i) {
      y0[i] = x1[i];
    }
  }
};
