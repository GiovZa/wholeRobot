The Scripts folder contains all the pyton files for a package. 

This folder so far has:

  robot_mux_control.py: The control unit that communicates with all other files to decide what operation to run next

  move_base_client.py: Sends goal to move_base for navigation, inputs and updates to robot_process to run the following motor script

  twistConvertor.py: Converts twists from geometry.msgs move_base topic to
  twistWithCovariancePose which is the topic type robot_localization accepts

Button layout for robot_mux_control.py:

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

TODO:
  TEST AUTONOMY!!! TEST robot_mux_control.py WITH ALL MODES!!!