package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Better configured XBox Controller to permit passing method references. */
public class CustomXBox extends XboxController {
  private static final double minJoystickValue = 0.1;

  public CustomXBox(int port) {
    super(port);
  }

  public double getLeftX() {
    return super.getX(Hand.kLeft);
  }

  public double getRightX() {
    return super.getX(Hand.kRight);
  }

  public double getLeftY() {
    return super.getY(Hand.kLeft);
  }

  public double getRightY() {
    return super.getY(Hand.kRight);
  }
}
