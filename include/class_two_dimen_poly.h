#ifndef CLASS_TWO_DIMEN_POLY_H
#define CLASS_TWO_DIMEN_POLY_H

#include <iostream>
#include "class_one_dimen_poly.h"
#include <memory>


namespace quintic_polynomial
{

class ClassTwoDimenPoly
{
private:
    std::unique_ptr<ClassOneDimenPoly> x_poly_ptr_;
    std::unique_ptr<ClassOneDimenPoly> y_poly_ptr_;

    double duration_;

    bool found_solution_;

public:
    ClassTwoDimenPoly(const double x_init, const double y_init, const double yaw_init, const double v_init, const double a_init,
                      const double x_goal, const double y_goal, const double yaw_goal, const double v_goal, const double a_goal);
    
    bool found_solution() const;
    double get_duration() const;

    double get_x(double t) const;
    double get_y(double t) const;



};




} // namespace quintic_polynomial


#endif // CLASS_TWO_DIMEN_POLY_H