#include <iostream>
#include <random>
#include <tuple>

using namespace std;

default_random_engine e(time(0));
double a = 2;
double b = 1;
double c = sqrt(a * a - b * b);
double t = 45;
vector<pair<double, double>> can;

double dis(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

pair<double, double> gene(){
    uniform_real_distribution<double> u_x(-a, a);
    uniform_real_distribution<double> u_y(-b, b);
    while (true) {
        double x = u_x(e);
        double y = u_y(e);
        if (x * x / a / a + y * y / b / b <= 1){
            return {x, y};
        }
    }
}

double get_theta(double x1, double y1, double x2, double y2){
    return atan2(y2 - y1, x2 - x1);
}

pair<double, double> gene_(double s_x, double s_y, double ex, double ey, double theta) {
    double theta1 = get_theta(s_x, s_y, ex, ey);
    double theta2 = (theta / 180.0 * M_PI) - theta1;
    lognormal_distribution<double> dis;
    double l = dis(e) + 0.5;
    return {s_x + l * cos(theta2), s_y + l * sin(theta2)};
}

int main() {
    double alpha = 1.5;
    double x, y;
    double pss_;
    double ps_e;
    double pee_;
    double ps_e_;
    freopen("/mnt/d/sbcy.txt", "w", stdout);
    for (int i = 0; i < 10000; i++){
        tie(x, y) = gene();

        ps_e = dis(c, 0, x, y);
        pss_ = dis(-c, 0, x, y);
        int count = 0;
        for (int j = 0; j < 10000; j++){
            double e_x, e_y;

            uniform_int_distribution<int> u_theta(t, 360 - t);
            double theta_ = u_theta(e);
            tie(e_x, e_y) = gene_(x, y, c, 0, theta_);
            pee_ = dis(c, 0, e_x, e_y);
            ps_e_ = dis(x, y, e_x, e_y);
            bool cond1 = pss_ + ps_e + pee_ <= alpha * ps_e_;
            bool cond2 = pss_ + ps_e_ + pee_ <= 2 * alpha * c;
            bool cond3 = pss_ + ps_e_ <= alpha * ps_e_;
            if (cond1 || cond2 && cond3){
                count++;
            }
        }

        printf("%f %f %d\n", x, y, count);

    }
    return 0;
}