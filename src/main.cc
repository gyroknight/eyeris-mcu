#include <Gobbledegook.h>
#include <unistd.h>
#include <wiringPi.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <ratio>

#include "EyerisController.hh"

namespace {
constexpr int buckEnable = 13;
constexpr int speakerEnable = 25;
}  // namespace

int main(int, char*[]) {
  // Set up enable pins
  wiringPiSetup();
  pinMode(buckEnable, OUTPUT);
  pinMode(speakerEnable, OUTPUT);
  // Reset sequence (in case they're still on)
  digitalWrite(buckEnable, false);
  digitalWrite(speakerEnable, false);
  usleep(5000);
  digitalWrite(buckEnable, true);
  digitalWrite(speakerEnable, true);
  usleep(5000);

  EyerisController controller;

  controller.start();

  while (true) {
    uint16_t distOne = controller.getDistance(0);
    uint16_t distTwo = controller.getDistance(1);
    uint16_t distThree = controller.getDistance(2);

    std::cout << distOne << " " << distTwo << " " << distThree << std::endl;
  }

  // Return the final server health status as a success (0) or error (-1)
  return ggkGetServerHealth() == EOk ? 0 : 1;
}
