
#include "class_one_dimen_poly.h"


namespace quintic_polynomial
{

ClassOneDimenPoly::ClassOneDimenPoly(const double x_init, const double v_init, const double a_init, 
                                     const double x_goal, const double v_goal, const double a_goal,
                                     const double t)
{
    state_init_.x = x_init;
    state_init_.v = v_init;
    state_init_.a = a_init;

    state_goal_.x = x_goal;
    state_goal_.v = v_goal;
    state_goal_.a = a_goal;

    duration_ = std::abs(t);

    A_ = state_init_.x;
    B_ = state_init_.v;
    C_ = state_init_.a / 2.0;

    t1_ = duration_;
    t2_ = t1_ * t1_;
    t3_ = t2_ * t1_;
    t4_ = t3_ * t1_;
    t5_ = t4_ * t1_;

    Eigen::MatrixXd temp(3, 3);
    temp << t3_,     t4_,      t5_,
            3 * t2_, 4 * t3_,  5 * t4_,
            6 * t1_, 12 * t2_, 20 * t3_;
    
    Eigen::VectorXd temp2(3);
    temp2 << state_goal_.x - A_ - B_ * t1_ - C_ * t2_,
             state_goal_.v - B_ - 2 * C_ * t1_,
             state_goal_.a - 2 * C_;
    
    Eigen::VectorXd DEF = temp.inverse() * temp2;
    D_ = DEF(0); E_ = DEF(1); F_ = DEF(2);
}

double ClassOneDimenPoly::get_x(double t) const
{
    t = std::min(std::max(t, 0.0), duration_);
    return A_+B_*t+C_*t*t+D_*t*t*t+E_*t*t*t*t+F_*t*t*t*t*t;
}

double ClassOneDimenPoly::get_v(double t) const
{
    t = std::min(std::max(t, 0.0), duration_);
    return B_+2*C_*t+3*D_*t*t+4*E_*t*t*t+5*F_*t*t*t*t;
}

double ClassOneDimenPoly::get_a(double t) const
{
    t = std::min(std::max(t, 0.0), duration_);
    return 2*C_+6*D_*t+12*E_*t*t+20*F_*t*t*t;
}

double ClassOneDimenPoly::get_j(double t) const
{
    t = std::min(std::max(t, 0.0), duration_);
    return 6*D_+24*E_*t+60*F_*t*t;
}



} // namespace quintic_polynomial
