package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Better configured XBox Controller to permit passing method references.
 */
public class CustomXBox extends XboxController {
    private static final double minJoystickValue = 0.1;

    public CustomXBox(int port) {
        super(port);
    }

    public double getLeftX() {
        return super.getX(Hand.kLeft);
        // double val = super.getX(Hand.kLeft);
        // if (Math.abs(val) < minJoystickValue) {
        // return val;
        // } else {
        // return 0;
        // }
    }

    public double getRightX() {
        return super.getX(Hand.kRight);
        // double val = super.getX(Hand.kRight);
        // if (Math.abs(val) < minJoystickValue) {
        // return val;
        // } else {
        // return 0;
        // }
    }

    public double getLeftY() {
        return super.getY(Hand.kLeft);
        // double val = super.getY(Hand.kLeft);
        // if (Math.abs(val) < minJoystickValue) {
        // return val;
        // } else {
        // return 0;
        // }
    }

    public double getRightY() {
        return super.getY(Hand.kRight);
        // double val = super.getY(Hand.kRight);
        // if (Math.abs(val) < minJoystickValue) {
        // return val;
        // } else {
        // return 0;
        // }
    }
}
