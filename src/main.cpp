
#include <chrono>
#include "matplotlibcpp.h"
#include "class_two_dimen_poly.h"

namespace plt = matplotlibcpp;

int main(){

    // time the searching
    auto start = std::chrono::high_resolution_clock::now();

    double x_init = 0;
    double y_init = 0;
    double yaw_init = 0;
    double v_init = 4;
    double a_init = 0.0;

    double x_goal = 20;
    double y_goal = 12.5;
    double yaw_goal = -0.4;
    double v_goal = 3;
    double a_goal = 0.0;

    quintic_polynomial::ClassTwoDimenPoly poly(x_init, y_init, yaw_init, v_init, a_init, 
                                               x_goal, y_goal, yaw_goal, v_goal, a_goal);

    auto end = std::chrono::high_resolution_clock::now();
    
    auto time_solve = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Time to solve: " << time_solve.count() << "us" << std::endl;

    double t = 0;
    if (poly.found_solution()) t = poly.get_duration();
    else
    {
        std::cerr << "Cannot find a valid path." << std::endl;
        exit(1);
    }

    std::vector<double> x, y;
    for (double i = 0; i < t; i += 0.08)
    {
        x.push_back(poly.get_x(i));
        y.push_back(poly.get_y(i));
    }
    
    // draw 2 line segments to represent the initial and goal poses, 
    // coz the arrow function is not working with my version of matplotlibcpp    
    std::vector<double> x_init_line = {x_init, x_init + 0.8*std::cos(yaw_init)};
    std::vector<double> y_init_line = {y_init, y_init + 0.8*std::sin(yaw_init)};
    std::vector<double> x_goal_line = {x_goal, x_goal + 0.8*std::cos(yaw_goal)};
    std::vector<double> y_goal_line = {y_goal, y_goal + 0.8*std::sin(yaw_goal)};
    plt::plot(x_init_line, y_init_line, "r");
    plt::plot(x_goal_line, y_goal_line, "r");

    // draw the curve
    plt::scatter(x, y);

    plt::grid(true);
    plt::axis("equal");

    plt::show();
}

