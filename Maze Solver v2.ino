#include "./lib.h"

// Kodu kontrol ederken fark ettim mikrokontroller çok büyük array kullanamıyor
// Yani labirentin ölçüleri mak 31 x 31 olmalı
Solver<20> solver;
void setup() {
  Serial.begin(9600);
}

void loop() {
  Dir dir = left;
  solver.move(dir);
}