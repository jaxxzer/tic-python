from tic import TicUdp
import time

def waitForPosition(targetPosition):
    while (tic.getCurrentPosition() != targetPosition):
        print(tic.getCurrentPosition())
        tic.resetCommandTimeout()
        time.sleep(0.1)

tic = TicUdp("192.168.1.226", 4444)

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
  while(tic.getTargetPosition() != 100):
    tic.resetCommandTimeout()
    tic.setTargetPosition(100)
    time.sleep(0.1)
  waitForPosition(100)
  while(tic.getTargetPosition() != 0):
    tic.resetCommandTimeout()
    tic.setTargetPosition(0)
    time.sleep(0.1)
  waitForPosition(0)

