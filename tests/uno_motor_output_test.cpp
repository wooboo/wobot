#include <assert.h>

#include "../firmware/uno-controller/motor_output.h"

int main() {
  MotorOutputController output;

  MotorPwm stopped = output.update(0, 0, 0);
  assert(stopped.left == 0 && stopped.right == 0);

  MotorPwm starting = output.update(100, 100, 100);
  assert(starting.left == 210 && starting.right == 210);

  MotorPwm settled = output.update(401, 100, 100);
  assert(settled.left == 26 && settled.right == 26);

  // A tight turn can stop one side, but bringing it back must not retrigger
  // the whole 300 ms startup kick while the robot is already moving.
  MotorPwm tightTurn = output.update(450, 0, 500);
  assert(tightTurn.left == 0 && tightTurn.right == 127);
  MotorPwm leavingTurn = output.update(500, 100, 500);
  assert(leavingTurn.left == 26 && leavingTurn.right == 127);

  output.update(600, 0, 0);
  MotorPwm restarting = output.update(700, 100, 100);
  assert(restarting.left == 210 && restarting.right == 210);
}
