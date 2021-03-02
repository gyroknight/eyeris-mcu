#include <unistd.h>

#include <iostream>

#include "EyerisController.hh"

int main(int, char*[]) {
  EyerisController controller;

  controller.start();

  while (true) {
    uint16_t distOne = controller.getDistance(0);
    uint16_t distTwo = controller.getDistance(1);
    uint16_t distThree = controller.getDistance(2);

    std::cout << distOne << " " << distTwo << " " << distThree << std::endl;
    usleep(5000);
  }

  return 0;
}