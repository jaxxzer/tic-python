from tic import TicUdp
import time

def waitForPosition(targetPosition):
    while (tic.getCurrentPosition() != targetPosition):
        tic.resetCommandTimeout()
        time.sleep(0.05)

tic = TicUdp("0.0.0.0", 4444)

# Set the Tic's current position to 0, so that when we command
# it to move later, it will move a predictable amount.
tic.haltAndSetPosition(0)

# Tells the Tic that it is OK to start driving the motor.  The
# Tic's safe-start feature helps avoid unexpected, accidental
# movement of the motor: if an error happens, the Tic will not
# drive the motor again until it receives the Exit Safe Start
# command.  The safe-start feature can be disbled in the Tic
# Control Center.
tic.exitSafeStart()

time.sleep(0.01)
for i in range(10):
  tic.setTargetPosition(-100)
  waitForPosition(-100)
  tic.setTargetPosition(100)
  waitForPosition(100)
