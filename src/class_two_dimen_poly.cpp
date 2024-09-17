
#include "class_two_dimen_poly.h"


namespace quintic_polynomial
{

ClassTwoDimenPoly::ClassTwoDimenPoly(const double x_init, const double y_init, const double yaw_init, const double v_init, const double a_init,
                                    const double x_goal, const double y_goal, const double yaw_goal, const double v_goal, const double a_goal)
{
    // divide the x and y directions
    double sin_init = std::sin(yaw_init);
    double cos_init = std::cos(yaw_init);
    double sin_goal = std::sin(yaw_goal);
    double cos_goal = std::cos(yaw_goal);

    double max_acc = 15.0;
    double max_jerk = 15;

    // try different time durations until the acceleration is low enough
    found_solution_ = false;
    for (double time = 0.9; time < 20; time += 0.4)
    {
        // std::cout << "try time: " << time << std::endl;
        x_poly_ptr_ = std::make_unique<ClassOneDimenPoly>(x_init, v_init * cos_init, a_init * cos_init, 
                                                        x_goal, v_goal * cos_goal, a_goal * cos_goal, time);   
        y_poly_ptr_ = std::make_unique<ClassOneDimenPoly>(y_init, v_init * sin_init, a_init * sin_init, 
                                                        y_goal, v_goal * sin_goal, a_goal * sin_goal, time);
        bool motion_valid = true;
        double ax, ay, a, jx, jy, j;
        for (double t = 0.0; t < time; t += 0.3)
        {
            ax = x_poly_ptr_->get_a(t);
            ay = y_poly_ptr_->get_a(t);
            a = std::sqrt(ax * ax + ay * ay);
            jx = x_poly_ptr_->get_j(t);
            jy = y_poly_ptr_->get_j(t);
            j = std::sqrt(jx * jx + jy * jy);
            if (a > max_acc || j > max_jerk)
            {
                motion_valid = false;
                break;
            }
        }
        if (motion_valid)
        {
            found_solution_ = true;
            duration_ = time;
            std::cout << "Found a valid path with time duration: " << time << std::endl;
            break;
        }
    }
    if (!found_solution_)
    {
        std::cerr << "Cannot find a valid path" << std::endl;
        exit(1);    
    }
    
}


bool ClassTwoDimenPoly::found_solution() const
{
    return found_solution_;
}

double ClassTwoDimenPoly::get_duration() const
{
    return duration_;
}

double ClassTwoDimenPoly::get_x(double t) const
{
    return x_poly_ptr_->get_x(t);
}

double ClassTwoDimenPoly::get_y(double t) const
{
    return y_poly_ptr_->get_x(t);
}



} // namespace quintic_polynomial


