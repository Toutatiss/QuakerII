#include <Quaker_Library.h>

Quaker quaker;

// Run Quaker setup code
void setup() {
  quaker.begin();
}

// Run main Quaker Tasks
void loop() {
  quaker.runUI();
}

// Drive motor mostly independently over on the second core
void loop1() {
  quaker.runMotor();
}
