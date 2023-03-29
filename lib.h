#include "Arduino.h"
#include "Stepper.h"

// Sensor Pins
#define forwardPin 3
#define rightPin 2
#define leftPin 4

// Motor 1
#define AIN1_1 10
#define AIN2_1 9
#define BIN1_1 11
#define BIN2_1 12

// Motor 2
#define AIN1_2 6
#define AIN2_2 5
#define BIN1_2 7
#define BIN2_2 8

#define STEPS 210
#define SPEED 60
// Step miktarlarını hesaplaman gerek
#define forwardStep 210
#define rotateStep 165

Stepper motor1(STEPS, AIN2_1, AIN1_1, BIN1_1, BIN2_1);
Stepper motor2(STEPS, AIN2_2, AIN1_2, BIN1_2, BIN2_2);

enum Dir : char {
  forward,
  right,
  backward,
  left
};

template<const unsigned int MAX_SIZE>
class Solver {
public:
  unsigned char visitCounts[MAX_SIZE][MAX_SIZE];
  Dir absDirMap[MAX_SIZE][MAX_SIZE];
  unsigned int x = 0;
  unsigned int y = 0;
  Dir robAbsDir = forward;

// Function to convert relative direction to absolute direction
  Dir relDir2AbsDir(Dir &relDir) {
    return (Dir)((robAbsDir + relDir) % 4);
  }

// Function to convert absolute direction to relative direction-+
  Dir absDir2RelDir(Dir &absDir) {
    return (Dir)((absDir - robAbsDir + 4) % 4);
  }

  // !!!!!!!!!!!!!!!!!
  bool isFinished() {
    return x == 2 && y == 2;
  }

/* This is a function called "move" that takes a reference to a variable of type "Dir" as input.
* The function then performs different actions based on the value of the "relDir" variable.
*/
  void move(Dir &relDir) {
    // Use a switch statement to check the value of "relDir"
    switch (relDir) {
      // If "relDir" is equal to "right"
      case right:
        // Move the motors in a certain way to make the robot move right
        for (auto i = 0; i < rotateStep; i++) {
          motor1.step(1);
          motor2.step(-1);
        }
        break;
      // If "relDir" is equal to "left"
      case left:
        // Move the motors in a different way to make the robot move left
        for (auto i = 0; i < rotateStep; i++) {
          motor1.step(-1);
          motor2.step(1);
        }
        break;
      // If "relDir" is equal to "backward"
      case backward:
        // Move the motors in yet another way to make the robot move backward
        for (auto i = 0; i < 2 * rotateStep; i++) {
          motor1.step(1);
          motor2.step(1);
        }
        break;
      // If "relDir" is anything else
      default:
        // Move the motors in a different way to make the robot move forward
        for (auto i = 0; i < forwardStep; i++) {
          motor1.step(-1);
          motor2.step(-1);
        }
    }
  }


/* This is a function called "checkDir" that takes an array of "Dir" variables and a reference to an unsigned char variable as input.
* The function reads the values of three digital pins and populates the array "relDirs" with the directions that the robot can move in.
* It also modifies the "size" variable to indicate the number of valid directions in the array.
*/
  void checkDir(Dir (&relDirs)[4], unsigned char &size) {
    // Set the first element of "relDirs" to "backward"
    relDirs[0] = backward;
    // Initialize the "index" variable to 1
    auto index = 1;
    // Check if the "forwardPin" is high
    if (digitalRead(forwardPin)) {
      // If it is, add "forward" to the next available index in "relDirs" and increment "index"
      relDirs[index] = forward;
      index++;
    }
    // Check if the "rightPin" is high
    if (digitalRead(rightPin)) {
      // If it is, add "right" to the next available index in "relDirs" and increment "index"
      relDirs[index] = right;
      index++;
    }
    // Check if the "leftPin" is high
    if (digitalRead(leftPin)) {
      // If it is, add "left" to the next available index in "relDirs" and increment "index"
      relDirs[index] = left;
      index++;
    }
    // Set the "size" variable to the final value of "index"
    size = index;
  }

/* This is a function called "iterate" that implements a pathfinding algorithm for the robot.
* It first calls the "checkDir" function to determine the valid directions that the robot can move in.
* It then updates the "visitCounts" array to keep track of how many times each cell has been visited.
* Next, it calculates the potential moves that the robot can make based on the "relDirs" array and updates the "absDirMap" array to store the absolute directions that the robot is facing.
* The function then calls the "move" function to move the robot in the selected direction and updates the "x" and "y" variables to reflect its new position.
* Finally, the function checks whether the robot has reached its destination and, if not, calls itself recursively to continue the pathfinding algorithm.
*/
  void iterate() {
    // Initialize an array of "Dir" variables called "relDirs" and an unsigned char variable called "size"
    Dir relDirs[4];
    unsigned char size = 0;
    // Call the "checkDir" function to populate "relDirs" and "size"
    checkDir(relDirs, size);
    // Update the "visitCounts" array to keep track of how many times each cell has been visited
    // If the current cell is a dead end (size == 1), the algorithm marks it as visited four times to ensure that it is not revisited in the future.
    visitCounts[y][x] += size == 1 ? 4 : 1;

    // Initialize temporary variables "_x", "_y", "minVisit", and "selectedRelDir"
    auto _x = x;
    auto _y = y;
    unsigned char minVisit = 5;
    Dir selectedRelDir = forward;
    // Iterate over the "relDirs" array and check the potential moves that the robot can make
    for (auto i = 0; i < size; i++) {
      // Convert the relative direction to an absolute direction
      auto absDir = relDir2AbsDir(relDirs[i]);
      // Calculate the new x and y coordinates based on the absolute direction
      auto __x = absDir == left ? x - 1 : absDir == right ? x + 1
                                                          : x;
      auto __y = absDir == backward ? y - 1 : absDir == forward ? y + 1
                                                                : y;
      // Check if the new x coordinate is out of bounds and adjust it if necessary
      __x += __x > MAX_SIZE ? -1 : __x < 0 ? 1
                                           : 0;
      // Check if the new y coordinate is out of bounds and adjust it if necessary
      __y += __y > MAX_SIZE ? -1 : __y < 0 ? 1
                                           : 0;

      // Check if the new cell has been visited fewer times than the current minimum
      if (visitCounts[__y][__x] < minVisit) {
        // If it has, update the temporary variables to reflect the new minimum
        _x = __x;
        _y = __y;
        minVisit = visitCounts[__y][__x];
        selectedRelDir = absDir2RelDir(absDir);
      }
    }

    // Update the "absDirMap" array to store the absolute direction that the robot is facing
    absDirMap[y][x] = relDir2AbsDir(selectedRelDir);
    // Move the robot in the selected direction and update its position
    move(selectedRelDir);
    x = _x;
    y = _y;
    // Check if the robot has reached its destination and, if not, call the "iterate" function recursively
    if (!isFinished()) iterate();
  }

/**
* Initializes a new Solver object.
* Sets the forwardPin, rightPin, and leftPin as inputs.
* Sets the speed of motor1 and motor2 to SPEED.
* Initializes the visitCounts array with zeros.
*/
  Solver() {
    pinMode(forwardPin, INPUT);
    pinMode(rightPin, INPUT);
    pinMode(leftPin, INPUT);

    motor1.setSpeed(SPEED);
    motor2.setSpeed(SPEED);

    memset(visitCounts, 0, sizeof(visitCounts));
  }
};
