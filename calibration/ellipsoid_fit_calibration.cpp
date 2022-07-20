#include "ellipsoid_fit_calibration.h"

#include <Eigen/Dense>

using namespace imu::calibration;
using namespace Eigen;

std::tuple<Offset, RotationMatrix> imu::calibration::ellipsoid_fit::calibrate(std::unique_ptr<imu::calibration::CalibrationData> &data)
{
    Offset offset;
    RotationMatrix rot;

    MatrixXd D(10, data->size());

    auto [x, y, z] = data->get();

    double xd, yd, zd;
    for (int i = 0; i < data->size(); i++)
    {
        xd = static_cast<double>(x.at(i));
        yd = static_cast<double>(y.at(i));
        zd = static_cast<double>(z.at(i));
        D(0, i) = xd * xd;
        D(1, i) = yd * yd;
        D(2, i) = zd * zd;
        D(3, i) = 2 * yd * zd;
        D(4, i) = 2 * xd * zd;
        D(5, i) = 2 * xd * yd;
        D(6, i) = 2 * xd;
        D(7, i) = 2 * yd;
        D(8, i) = 2 * zd;
        D(9, i) = 1;
    }

    Matrix<double, Dynamic, 10> Dt = D.transpose();

    Matrix<double, 10, 10> S = D * Dt;

    Matrix<double, 6, 6> S11;
    S11(0, 0) = S(0, 0);
    S11(0, 1) = S(0, 1);
    S11(0, 2) = S(0, 2);
    S11(0, 3) = S(0, 3);
    S11(0, 4) = S(0, 4);
    S11(0, 5) = S(0, 5);
    S11(1, 0) = S(1, 0);
    S11(1, 1) = S(1, 1);
    S11(1, 2) = S(1, 2);
    S11(1, 3) = S(1, 3);
    S11(1, 4) = S(1, 4);
    S11(1, 5) = S(1, 5);
    S11(2, 0) = S(2, 0);
    S11(2, 1) = S(2, 1);
    S11(2, 2) = S(2, 2);
    S11(2, 3) = S(2, 3);
    S11(2, 4) = S(2, 4);
    S11(2, 5) = S(2, 5);
    S11(3, 0) = S(3, 0);
    S11(3, 1) = S(3, 1);
    S11(3, 2) = S(3, 2);
    S11(3, 3) = S(3, 3);
    S11(3, 4) = S(3, 4);
    S11(3, 5) = S(3, 5);
    S11(4, 0) = S(4, 0);
    S11(4, 1) = S(4, 1);
    S11(4, 2) = S(4, 2);
    S11(4, 3) = S(4, 3);
    S11(4, 4) = S(4, 4);
    S11(4, 5) = S(4, 5);
    S11(5, 0) = S(5, 0);
    S11(5, 1) = S(5, 1);
    S11(5, 2) = S(5, 2);
    S11(5, 3) = S(5, 3);
    S11(5, 4) = S(5, 4);
    S11(5, 5) = S(5, 5);

    Matrix<double, 6, 4> S12;
    S12(0, 0) = S(0, 6);
    S12(0, 1) = S(0, 7);
    S12(0, 2) = S(0, 8);
    S12(0, 3) = S(0, 9);
    S12(1, 0) = S(1, 6);
    S12(1, 1) = S(1, 7);
    S12(1, 2) = S(1, 8);
    S12(1, 3) = S(1, 9);
    S12(2, 0) = S(2, 6);
    S12(2, 1) = S(2, 7);
    S12(2, 2) = S(2, 8);
    S12(2, 3) = S(2, 9);
    S12(3, 0) = S(3, 6);
    S12(3, 1) = S(3, 7);
    S12(3, 2) = S(3, 8);
    S12(3, 3) = S(3, 9);
    S12(4, 0) = S(4, 6);
    S12(4, 1) = S(4, 7);
    S12(4, 2) = S(4, 8);
    S12(4, 3) = S(4, 9);
    S12(5, 0) = S(5, 6);
    S12(5, 1) = S(5, 7);
    S12(5, 2) = S(5, 8);
    S12(5, 3) = S(5, 9);

    Matrix<double, 4, 6> S21;
    S21(0, 0) = S(6, 0);
    S21(0, 1) = S(6, 1);
    S21(0, 2) = S(6, 2);
    S21(0, 3) = S(6, 3);
    S21(0, 4) = S(6, 4);
    S21(0, 5) = S(6, 5);
    S21(1, 0) = S(7, 0);
    S21(1, 1) = S(7, 1);
    S21(1, 2) = S(7, 2);
    S21(1, 3) = S(7, 3);
    S21(1, 4) = S(7, 4);
    S21(1, 5) = S(7, 5);
    S21(2, 0) = S(8, 0);
    S21(2, 1) = S(8, 1);
    S21(2, 2) = S(8, 2);
    S21(2, 3) = S(8, 3);
    S21(2, 4) = S(8, 4);
    S21(2, 5) = S(8, 5);
    S21(3, 0) = S(9, 0);
    S21(3, 1) = S(9, 1);
    S21(3, 2) = S(9, 2);
    S21(3, 3) = S(9, 3);
    S21(3, 4) = S(9, 4);
    S21(3, 5) = S(9, 5);

    Matrix<double, 4, 4> S22;
    S22(0, 0) = S(6, 6);
    S22(0, 1) = S(6, 7);
    S22(0, 2) = S(6, 8);
    S22(0, 3) = S(6, 9);
    S22(1, 0) = S(7, 6);
    S22(1, 1) = S(7, 7);
    S22(1, 2) = S(7, 8);
    S22(1, 3) = S(7, 9);
    S22(2, 0) = S(8, 6);
    S22(2, 1) = S(8, 7);
    S22(2, 2) = S(8, 8);
    S22(2, 3) = S(8, 9);
    S22(3, 0) = S(9, 6);
    S22(3, 1) = S(9, 7);
    S22(3, 2) = S(9, 8);
    S22(3, 3) = S(9, 9);

    Matrix<double, 4, 4> S22inv = S22.inverse();

    Matrix<double, 4, 6> S12t = S12.transpose();

    Matrix<double, 4, 6> S22a = S22inv * S12t;

    Matrix<double, 6, 6> S22b = S12 * S22a;

    Matrix<double, 6, 6> SS = S11 - S22b;

    Matrix<double, 6, 6> Cinv;
    Cinv(0, 0) = 0;
    Cinv(0, 1) = 0.5;
    Cinv(0, 2) = 0.5;
    Cinv(0, 3) = 0;
    Cinv(0, 4) = 0;
    Cinv(0, 5) = 0;
    Cinv(1, 0) = 0.5;
    Cinv(1, 1) = 0;
    Cinv(1, 2) = 0.5;
    Cinv(1, 3) = 0;
    Cinv(1, 4) = 0;
    Cinv(1, 5) = 0;
    Cinv(2, 0) = 0.5;
    Cinv(2, 1) = 0.5;
    Cinv(2, 2) = 0;
    Cinv(2, 3) = 0;
    Cinv(2, 4) = 0;
    Cinv(2, 5) = 0;
    Cinv(3, 0) = 0;
    Cinv(3, 1) = 0;
    Cinv(3, 2) = 0;
    Cinv(3, 3) = -0.25;
    Cinv(3, 4) = 0;
    Cinv(3, 5) = 0;
    Cinv(4, 0) = 0;
    Cinv(4, 1) = 0;
    Cinv(4, 2) = 0;
    Cinv(4, 3) = 0;
    Cinv(4, 4) = -0.25;
    Cinv(4, 5) = 0;
    Cinv(5, 0) = 0;
    Cinv(5, 1) = 0;
    Cinv(5, 2) = 0;
    Cinv(5, 3) = 0;
    Cinv(5, 4) = 0;
    Cinv(5, 5) = -0.25;

    Matrix<double, 6, 6> E = Cinv * SS;

    EigenSolver<MatrixXd> es(E);
    Matrix<double, 6, 1> values = (es.eigenvalues()).real();
    Matrix<double, 6, 6> vectors = (es.eigenvectors()).real();

    int index = 0;
    double maxval = values(0);

    for (int i = 1; i < 6; i++)
    {
        if (values(i) > maxval)
        {
            maxval = values(i);
            index = i;
        }
    }

    Matrix<double, 6, 1> v1;
    if (vectors(0, index) > 0)
    {
        v1(0, 0) = vectors(0, index);
        v1(1, 0) = vectors(1, index);
        v1(2, 0) = vectors(2, index);
        v1(3, 0) = vectors(3, index);
        v1(4, 0) = vectors(4, index);
        v1(5, 0) = vectors(5, index);
    }
    else
    {
        v1(0, 0) = -vectors(0, index);
        v1(1, 0) = -vectors(1, index);
        v1(2, 0) = -vectors(2, index);
        v1(3, 0) = -vectors(3, index);
        v1(4, 0) = -vectors(4, index);
        v1(5, 0) = -vectors(5, index);
    }

    Matrix<double, 4, 1> v2;

    v2 = S22a * v1;

    Matrix<double, 10, 1> v;
    v(0, 0) = v1(0, 0);
    v(1, 0) = v1(1, 0);
    v(2, 0) = v1(2, 0);
    v(3, 0) = v1(3, 0);
    v(4, 0) = v1(4, 0);
    v(5, 0) = v1(5, 0);
    v(6, 0) = -v2(0, 0);
    v(7, 0) = -v2(1, 0);
    v(8, 0) = -v2(2, 0);
    v(9, 0) = -v2(3, 0);

    Matrix<double, 3, 3> Q;
    Q(0, 0) = v(0, 0);
    Q(0, 1) = v(5, 0);
    Q(0, 2) = v(4, 0);
    Q(1, 0) = v(5, 0);
    Q(1, 1) = v(1, 0);
    Q(1, 2) = v(3, 0);
    Q(2, 0) = v(4, 0);
    Q(2, 1) = v(3, 0);
    Q(2, 2) = v(2, 0);

    Vector3d U;
    U(0) = v(6, 0);
    U(1) = v(7, 0);
    U(2) = v(8, 0);

    Matrix<double, 3, 3> Qinv = Q.inverse();

    Matrix<double, 3, 1> B = Qinv * U;

    B(0,0) = -B(0,0);
    B(1,0) = -B(1,0);
    B(2,0) = -B(2,0);

    Matrix<double, 1, 3> Bt = B.transpose();

    Matrix<double, 1, 1> btqb;
    btqb = Bt * Q * B;

    double hmb = sqrt(btqb(0,0) - v(9,0));

    SelfAdjointEigenSolver<MatrixXd> es_q(Q);
    Matrix<double, 3, 3> SQ =  es_q.operatorSqrt();

    double norm_coef = 10000 / hmb;

    rot.m00 = SQ(0,0) * norm_coef;     rot.m01 = SQ(0,1) * norm_coef;     rot.m02 = SQ(0,2) * norm_coef;
    rot.m10 = SQ(1,0) * norm_coef;     rot.m11 = SQ(1,1) * norm_coef;     rot.m12 = SQ(1,2) * norm_coef;
    rot.m20 = SQ(2,0) * norm_coef;     rot.m21 = SQ(2,1) * norm_coef;     rot.m22 = SQ(2,2) * norm_coef;

    offset.x = B(0,0);
    offset.y = B(1,0);
    offset.z = B(2,0);

    return std::tuple<Offset, RotationMatrix>{offset, rot};
}