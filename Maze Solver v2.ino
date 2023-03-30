// Include the library header file
#include "./lib.h"


// Due to memory limitations, the maze can only reach a maximum size of 31 x 31. 
// To optimize its efficiency, I utilized chars, which only occupy 1 byte of memory, instead of ints that take up 4 bytes. 
// Interestingly, in C++, bool and char have the same memory size of 1 byte, making it the minimal amount of storage that can be used.
// This means that the additional 3 bits in bool are essentially unused.

// Create an instance of the Solver class with a maximum size of 31
Solver<31> solver;
// sjsjsjsj

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
    solver.checkDir(dirs);
    solver.move(dirs[1]);
  }
}
