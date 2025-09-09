/* piecewise liner fitting algrithem to convert adc reading to temperature*/
#include <stdio.h>
#include <math.h>

#define N_SEGMENTS 15
#define N_CHECK 16

typedef struct {
    double x_start;
    double x_end;
    double slope;
    double intercept;
} Segment;

Segment segments[N_SEGMENTS] = {
    {501.000000, 627.255572, 0.029508, 1.216293},
    {627.255572, 763.201308, 0.040170, -5.471393},
    {763.201308, 776.801195, 0.062038, -22.160905},
    {776.801195, 861.955228, 0.055010, -16.701379},
    {861.955228, 985.258211, 0.034212, 1.225793},
    {985.258211, 1073.848327, 0.039593, -4.076047},
    {1073.848327, 1112.870210, 0.039891, -4.396021},
    {1112.870210, 1113.811833, 0.056683, -23.083279},
    {1113.811833, 1280.274532, 0.059623, -26.358843},
    {1280.274532, 1342.996401, 0.089497, -64.605109},
    {1342.996401, 1408.000123, 0.067858, -35.544605},
    {1408.000123, 1473.999932, 0.075758, -46.666662},
    {1473.999932, 1583.500026, 0.087622, -64.154618},
    {1583.500026, 1675.005538, 0.105624, -92.661051},
    {1675.005538, 1757.500000, 0.130193, -133.814634}
};

double check_x[N_CHECK] = {501.000000, 627.255572, 763.201308, 776.801195, 861.955228, 985.258211, 1073.848327, 1112.870210, 1113.811833, 1280.274532, 1342.996401, 1408.000123, 1473.999932, 1583.500026, 1675.005538, 1757.500000};
double y_actual[N_CHECK] = {16.000000, 19.662779, 25.192951, 25.862575, 30.557587, 34.906592, 38.120260, 39.993511, 40.031224, 49.954658, 55.999460, 60.000007, 64.999994, 74.500003, 84.188192, 95.000000};
double piecewise_fit(double x) {
    if(x <= segments[0].x_start) return segments[0].slope*x + segments[0].intercept;
    if(x >= segments[N_SEGMENTS-1].x_end) return segments[N_SEGMENTS-1].slope*x + segments[N_SEGMENTS-1].intercept;
    int i = 0;
    for( i=0;i<N_SEGMENTS;i++) {
        if(x>=segments[i].x_start && x<=segments[i].x_end) return segments[i].slope*x + segments[i].intercept;
    }
    return 200.0;
}

int main() {
     printf("He orld!\n");
    int i = 0;
    for(  i=0;i<N_SEGMENTS;i++) {
        printf("Segment %d: x_start=%f, x_end=%f, slope=%f, intercept=%f\n", i, segments[i].x_start, segments[i].x_end, segments[i].slope, segments[i].intercept);
    }

    printf("--- Predicted Values & Errors at Breakpoints/Anchors ---");
    for( i=0;i<N_CHECK;i++) {
        double y_pred = piecewise_fit(check_x[i]);
        double error = y_actual[i] - y_pred;
        printf("x=%f, y_actual=%f, y_pred=%f, error=%f\n", check_x[i], y_actual[i], y_pred, error);
    }

    double test_x[] = {793, 839, 1139, 1627, 1707, 1771, 1801};
    int n_test=7;
    printf("--- Arbitrary x evaluation ---");

    for( i=0;i<n_test;i++) {
       printf("x=%f, y=%f\n", test_x[i], piecewise_fit(test_x[i]));
    }
    return 0;
}
