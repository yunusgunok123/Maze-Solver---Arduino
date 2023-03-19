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

  Dir relDir2AbsDir(Dir &relDir) {
    return (Dir)((robAbsDir + relDir) % 4);
  }
  Dir absDir2RelDir(Dir &absDir) {
    return (Dir)((absDir - robAbsDir + 4) % 4);
  }

  // !!!!!!!!!!!!!!!!!
  bool isFinished() {
    return x == 2 && y == 2;
  }

  void move(Dir &relDir) {
    switch (relDir) {
      case right:
        for (auto i = 0; i < rotateStep; i++) {
          motor1.step(1);
          motor2.step(-1);
        }
        break;
      case left:
        for (auto i = 0; i < rotateStep; i++) {
          motor1.step(-1);
          motor2.step(1);
        }
        break;
      case backward:
        for (auto i = 0; i < 2 * rotateStep; i++) {
          motor1.step(1);
          motor2.step(1);
        }
        break;
      default:
        for (auto i = 0; i < forwardStep; i++) {
          motor1.step(-1);
          motor2.step(-1);
        }
    }
  }

  void checkDir(Dir (&relDirs)[4], unsigned char &size) {
    relDirs[0] = backward;
    auto index = 1;
    if (digitalRead(forwardPin)) {
      relDirs[index] = forward;
      index++;
    }
    if (digitalRead(rightPin)) {
      relDirs[index] = right;
      index++;
    }
    if (digitalRead(leftPin)) {
      relDirs[index] = left;
      index++;
    }
    size = index;
  }

  void iterate() {
    Dir relDirs[4];
    unsigned char size = 0;
    checkDir(relDirs, size);
    visitCounts[y][x] += size == 1 ? 4 : 1;

    auto _x = x;
    auto _y = y;
    unsigned char minVisit = 5;
    Dir selectedRelDir = forward;
    for (auto i = 0; i < size; i++) {
      auto absDir = relDir2AbsDir(relDirs[i]);
      auto __x = absDir == left ? x - 1 : absDir == right ? x + 1
                                                          : x;
      auto __y = absDir == backward ? y - 1 : absDir == forward ? y + 1
                                                                : y;
      __x += __x > MAX_SIZE ? -1 : __x < 0 ? 1
                                           : 0;
      __y += __y > MAX_SIZE ? -1 : __y < 0 ? 1
                                           : 0;

      if (visitCounts[__y][__x] < minVisit) {
        _x = __x;
        _y = __y;
        minVisit = visitCounts[__y][__x];
        selectedRelDir = absDir2RelDir(absDir);
      }
    }

    absDirMap[y][x] = relDir2AbsDir(selectedRelDir);
    move(selectedRelDir);
    x = _x;
    y = _y;
    if (!isFinished()) iterate();
  }

  Solver() {
    pinMode(forwardPin, INPUT);
    pinMode(rightPin, INPUT);
    pinMode(leftPin, INPUT);

    motor1.setSpeed(SPEED);
    motor2.setSpeed(SPEED);

    memset(visitCounts, 0, sizeof(visitCounts));
    // for (auto i = 0; i < MAX_SIZE; i++)
    //   for (auto j = 0; j < MAX_SIZE; j++) {
    //     visitCounts[i][j] = 0;
    //   }
  }
};
