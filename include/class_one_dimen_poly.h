#ifndef CLASS_ONE_DIMEN_POLY_H
#define CLASS_ONE_DIMEN_POLY_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>


namespace quintic_polynomial
{

struct State
{
    double x;
    double v;
    double a;
};


class ClassOneDimenPoly
{
private:
    State state_init_, state_goal_;
    double duration_;

    double A_, B_, C_, D_, E_, F_;
    double t1_, t2_, t3_, t4_, t5_;


public:
    ClassOneDimenPoly(const double x_init, const double v_init, const double a_init, 
                      const double x_goal, const double v_goal, const double a_goal,
                      const double t);
    
    double get_x(double t) const;
    double get_v(double t) const;
    double get_a(double t) const;
    double get_j(double t) const;

};


} // namespace quintic_polynomial


#endif // CLASS_ONE_DIMEN_POLY_H