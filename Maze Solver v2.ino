// Include the library header file
#include "./lib.h"

// Create an instance of the Solver class with a maximum size of 31
Solver<31> solver;

// Setup function that sets pin 13 as input with pullup resistor
void setup() {
  pinMode(13, INPUT_PULLUP);
}

// Loop function that checks if button connected to pin 13 is pressed
// If it is, the robot checks the directions it can move in and moves in the second direction in the array
// The first direction is always backwards since it is always possible to move backwards
void loop() {
  if (digitalRead(13)){
    delay(1000);
    Dir dirs[4];
    unsigned char size = 0;
    solver.checkDir(dirs, size);
    solver.move(dirs[1]);
  }
}
