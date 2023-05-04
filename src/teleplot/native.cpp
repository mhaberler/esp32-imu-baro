#include "Teleplot.h"
#include <math.h>
#include <stdlib.h>

// macOS, Linux

Teleplot teleplot;

int main(int argc, char *argv[]) {
  float i = 0;
  int state_arr_length = 3;
  std::string state_arr[3] = {"standing", "sitting", "walking"};

  int heights_arr_length = 6;
  double heights_arr[6] = {20, 5, 8, 4, 1, 2};

  switch (argc) {
  case 1:
    teleplot.begin(1); // stdout
    break;
  case 2:
    teleplot.begin(atoi(argv[1])); // any other fd
    break;
  case 3:
    // ip address, port
    teleplot.begin(argv[1], atoi(argv[2]));
    break;
  default:
    exit(1);
  }
  for (;;) {
    // Use instanciated object
    teleplot.update("sin", sin(i), "km²");
    teleplot.update("cos", cos(i), "");
    teleplot.update("state", state_arr[rand() % state_arr_length], "", "t");

    teleplot.update3D(
        ShapeTeleplot("mysquare", "cube")
            .setCubeProperties(heights_arr[rand() % heights_arr_length])
            .setPos(sin(i) * 10, cos(i) * 10));

    // Use static localhost object
    Teleplot::localhost().update("tan", tan(i), "");

    usleep(100 * 1000);

    i += 0.1;
  }
  return 0;
}
