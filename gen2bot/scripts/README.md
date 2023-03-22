The Scripts folder contains all the pyton files for a package. 

This folder so far has:

  autoInputs.py: The code that takes inputs from results in functions in trencherOperationsClass.cpp and
  outputs to move_base_client.py so that it can navigate automatically

  controllerInputs.py: The code that takes inputs from a controller and spews out the 
  percent output for wheel motor speeds and function calls for mining operations

  move_base_client.py: Sends goal to move_base for navigation, inputs and updates to robot_process

  twistConvertor.py: Converts twists from geometry.msgs move_base topic to
  twistWithCovariancePose which is the topic type robot_localization accepts

  motorTester.py: Script made to test motor commands and find ideal position, velocity, and PID

Button layout for controllerInputs and motorTester:

      a = message.buttons[0]
      b = message.buttons[1]
      x = message.buttons[3]
      y = message.buttons[4]
      Up DPad = message.axes[7] (1.0)
      Down DPad = message.axes[7] (-1.0)
      Left DPad = message.axes[6] (1.0)
      Right DPad = message.axes[6] (-1.0)
      RB = message.buttons[6]
      LB = message.buttons[7]
      menu = message.buttons[10]
      start = message.buttons[11]
      Xbox = message.buttons[12]
      LT = message.axes[5]
      RT = message.axes[4]
