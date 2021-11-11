package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Better configured XBox Controller to permit passing method references.
 */
public class CustomXBox extends XboxController {
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
        return super.getX(Hand.kLeft);
    }

    public double getRightY() {
        return super.getX(Hand.kRight);
    }
}
