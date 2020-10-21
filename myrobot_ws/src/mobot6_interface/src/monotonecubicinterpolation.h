#ifndef MONOTONE_CUBIC_HERMITE_INTERPOLATION_H
#define MONOTONE_CUBIC_HERMITE_INTERPOLATION_H

#include <Eigen/Eigen>
#include <assert.h>
#include <iostream>
#define     DIM     7

typedef     double                                                  XType;
typedef     Eigen::Matrix<double, Eigen::Dynamic, 1>                YType;
typedef     Eigen::Matrix<double, 1, Eigen::Dynamic>                XVectorType;
typedef     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>   YVectorType;
typedef     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>   CoefficientVectorType;

typedef     Eigen::Matrix<double, Eigen::Dynamic, 1 >               colvecX;


//#####################################
//###  1. define the class - MCHI   ###
//#####################################
class   MCHI
{
public:
    MCHI()
        : m_dim(DIM)
        , m_xs(XVectorType::Zero(1, 1))
        , m_ys(YVectorType::Zero(DIM, 1))
        , m_c1s(CoefficientVectorType::Zero(DIM, 1))
        , m_c2s(CoefficientVectorType::Zero(DIM, 1))
        , m_c3s(CoefficientVectorType::Zero(DIM, 1)){}

    MCHI(const XVectorType& xs, const YVectorType& ys, const CoefficientVectorType& c1s,
         const CoefficientVectorType& c2s, const CoefficientVectorType& c3s)
        : m_dim(ys.rows()), m_xs(xs), m_ys(ys), m_c1s(c1s), m_c2s(c2s), m_c3s(c3s) {}

    MCHI(const MCHI& cubic)
        : m_dim(cubic.m_dim), m_xs(cubic.m_xs), m_ys(cubic.m_ys),
          m_c1s(cubic.m_c1s), m_c2s(cubic.m_c2s), m_c3s(cubic.m_c3s) {}

    YType const operator() (const double u);

    int span(const double u);

    int Span(const double u, const XVectorType& xs);

    static double InterpolateFunc(const double x, const double xl, const double yl,
                                  const double c1s, const double c2s, const double c3s);

private:
    int                     m_dim;
    XVectorType             m_xs;
    YVectorType             m_ys;
    CoefficientVectorType   m_c1s;
    CoefficientVectorType   m_c2s;
    CoefficientVectorType   m_c3s;
};


int MCHI::Span(const double u, const XVectorType& xs)
{
    if (u <= xs(0))
        return 0;
    if (u >= xs(xs.size() - 1))
        return xs.size() - 1;
    const double* pos = std::upper_bound(xs.data(), xs.data()+xs.size(), u);
    return static_cast<int> (std::distance(xs.data(), pos) - 1);
}

double MCHI::InterpolateFunc( const double x, const double xl, const double yl,
                              const double c1s, const double c2s, const double c3s)
{
    double  diff = x - xl;
    double  diffSq = diff*diff;
    return  yl + c1s*diff + c2s*diffSq + c3s*diff*diffSq;
}

int MCHI::span(const double u)
{
    const int size = Span(u, m_xs);
    return size;
}

YType const MCHI::operator() (const double u)
{
    const int klower = span(u);

    if (klower >= m_ys.cols() - 1)
        return m_ys.col(m_ys.cols() - 1);

    const   XType                    xl = m_xs(klower);
    const   YType                   _yl = m_ys.col(klower);
    const   CoefficientVectorType   _c1 = m_c1s.col(klower);
    const   CoefficientVectorType   _c2 = m_c2s.col(klower);
    const   CoefficientVectorType   _c3 = m_c3s.col(klower);
    YType   ret(m_dim, 1);
    for (int i = 0; i < m_dim; ++i)
    {
        double  yl = _yl(i);
        double  c1 = _c1(i);
        double  c2 = _c2(i);
        double  c3 = _c3(i);
        ret(i) = MCHI::InterpolateFunc(u, xl, yl, c1, c2, c3);
    }

    return ret;
}


//###############################################
//###  2. define the function - Interpolate   ###
//###############################################
MCHI Interpolate(const Eigen::MatrixXd& ys, const XVectorType& xs)
{
    // check inputs
    assert(ys.cols() == xs.size());

    // init constants
    const int length = ys.cols();
    const int dim = ys.rows();

    // init variables
    CoefficientVectorType c1s, c2s, c3s;
    XVectorType dxs;
    YVectorType dys;
    YVectorType ms;
    c1s.resize(dim, length);
    c2s.resize(dim, length - 1);
    c3s.resize(dim, length - 1);
    dxs.resize(1, length - 1);
    dys.resize(dim, length - 1);
    ms.resize(dim, length - 1);

    // get consecutive differences and slopes
    for (int k = 0; k < length - 1; ++k)
    {
        dxs(k) = xs(k + 1) - xs(k);
        dys.col(k) = ys.col(k + 1) - ys.col(k);
        for (int i = 0; i < dim; ++i)
            ms(i, k) = dys(i, k) / dxs(k);
    }

    // get degree-1 coefficients
    c1s.col(0) = ms.col(0);
    for (int k = 0; k < length - 1 - 1; ++k)
    {
        double dx = dxs(k);
        double dxNext = dxs(k + 1);
        double common = dx + dxNext;
        for (int i = 0; i < dim; ++i)
        {
            double m = ms(i, k);
            double mNext = ms(i, k + 1);
            if (m*mNext <= 0)
                c1s(i, k + 1) = 0;
            else
                c1s(i, k + 1) = 3.0 * common / ((common + dxNext) / m + (common + dx) / mNext);
        }
    }
    c1s.col(length - 1) = ms.col(length - 1 - 1);

    // get degree-2 and degree-3 coefficients
    for (int k = 0; k < length - 1; ++k)
    {
        for (int i = 0; i < dim; ++i)
        {
            double  c1 = c1s(i, k);
            double  c1Next = c1s(i, k + 1);
            double  m = ms(i, k);
            double  invDx = 1.0 / dxs(k);
            double  common = c1 + c1Next - m - m;
            c2s(i, k) = (m - c1 - common)*invDx;
            c3s(i, k) = common*invDx*invDx;
        }
    }

    // return the MCHI class
    return MCHI(xs, ys, c1s, c2s, c3s);
}

MCHI Interpolate(const Eigen::MatrixXd& pts)
{
    XVectorType chord_lengths( pts.cols() );
    for (int i = 0; i < chord_lengths.size(); ++i)
        chord_lengths(i) = static_cast<double> (i) / (chord_lengths.size() - 1);
    return Interpolate(pts, chord_lengths);
}


//########################################
//###  3. define the function - lspb   ###
//########################################
colvecX const lspb(const double q0, const double q1, const double tf,
                   const double step, const double v)
{
    // check the desired velocity
    assert(v >= (q1-q0)/tf);
    assert(v <= 2.0*(q1-q0)/tf);
    int size = std::floor(tf/step)+1;
    // check the angular position
    if (fabs(q1-q0) <= 1e-3)
    {
        colvecX ret;
        ret.resize(size);
        ret.fill(q0);
        return ret;
    }

    // calculate the lspb sequence
    colvecX ret;
    ret.resize(size);
    double tb = (q0-q1+v*tf)/v;
    double a = v/tb;
    for (int i=0; i<ret.size(); ++i)
    {
        double tt = step*i;
        if (tt < tb)
        {
            ret(i) = q0 + a / 2.0 * tt * tt;
        }
        else if (tt <= tf - tb)
        {
            ret(i) = (q1 + q0 - v * tf) / 2.0 + v * tt;
        }
        else
        {
            ret(i) = q1 - a / 2.0 * tf * tf + a * tf * tt - a / 2.0 * tt * tt;
        }
    }
    return ret;
}

colvecX const lspb(const double q0, const double q1, const double tf,
                   const double step)
{
        double V = (q1 - q0) / tf * 1.5;
        return lspb(q0, q1, tf, step, V);
}


//############################################
//###  4. define the function - mcspline   ###
//############################################
Eigen::MatrixXd const mcspline(const Eigen::MatrixXd& ps, const colvecX& rs)
{
    // check rs equals to [rmin, rmax], rmax > rmin
    const double rmin = rs(0);
    const double rmax = rs(rs.size() - 1);
    const double rl = rmax - rmin;

    assert(fabs(rl) >= 1e-3);
    for (int i = 1; i < rs.size(); ++i)
        assert(rs(i) > rs(i - 1));

    // check ps less than 2
    assert(ps.cols() >= 2);

    // spline
    MCHI spl = Interpolate(ps);

    // interpolate
    Eigen::MatrixXd ret(ps.rows(), rs.size());
    for (int i = 0; i < rs.size(); ++i)
    {
        YType temp = spl((rs(i) - rmin) / rl);
        ret.col(i) = temp;
    }
    return  ret;
}


#endif // MONOTONECUBICINTERPOLATION_H

