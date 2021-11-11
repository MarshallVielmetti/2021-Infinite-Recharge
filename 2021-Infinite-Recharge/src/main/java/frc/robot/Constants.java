// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * TODO : NEED TO UPDATE TO MEET ACTUAL ROBOT SPECS The Constants class provides
 * a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything
 * functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * 
 * ALL UNITS ARE IN METERS (OR A DERIVATIVE) UNLESS OTHERWISE NOTED
 */
public final class Constants {
    public static class DriveConstants {
        public static final double kMaxSpeed = 3.0; // m/s
        public static final double kMaxAngularSpeed = 2 * Math.PI; // One roation per second

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters
        public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
        public static final int kEncoderResolution = 4096;

        public static final int kLeftMotor1ID = 11;
        public static final int kLeftMotor2ID = 13;
        public static final int kLeftMotor3ID = 15;

        public static final boolean kLeftMotorInverted = true;

        public static final int kRightMotor1ID = 12;
        public static final int kRightMotor2ID = 14;
        public static final int kRightMotor3ID = 16;

        public static final boolean kRightMotorInverted = false;

        // TODO - Figure out what this does
        // This doesn't make sense. We *need* shaft encoders.
        public static final double kPositionFactor = kWheelCircumference;
        public static final double kVelocityFactor = kWheelCircumference;

    }

    public static class HoldConstants {
        public static final int kHoldMotorID = 33;

        public static final boolean kHoldMotorInverted = false;

        public static final double kDefaultHoldPower = 0.33;

    }

    public static class IntakeConstants {
        public static final int kIntakeMotorID = 21;

        // Technically could still use full blown PID control
        // But it just isn't necessary
        public static final double kDefaultMotorPower = 0.33;

        public static final boolean kIntakeMotorInverted = false;
    }

    public static class ShooterConstants {

        public static final int kFlywheelMotorID = 31;
        public static final int kTurretMotorID = 34;
        public static final int kHoodMotorID = 32;

        public static final double kFlywheelMomentofIntertia = 0.00032; // kg / m^2
        // Reduction between motors and encoder, as output over input. If the flywheel
        // spins slower than
        // the motors, this number should be greater than one.
        public static final double kFlywheelGearing = 1.0;

        public static final double kFlywheelEncoderAccuracy = 0.01;

        public static final double kFlywheelModelAccuracy = 3.0;

        public static final double kFlywheelQelmsVTolerance = 3.0; // radians per second

        public static final double kFlywheelRelmsControlEffort = 12.0; // voltage

        public static final int kTurretEncoderMax = 1000;
        public static final int kTurretEncoderMin = -1000;

        // TODO PID constants for the turret
        public static final double kTurretKp = 0.1;
        public static final double kTurretKi = 0.0001;
        public static final double kTurretKd = 0.002;

        public static final double kHoodKp = 0.1;
        public static final double kHoodKi = 0.0001;
        public static final double kHoodKd = 0.002;
    }

    public static class OIConstants {
    }
}
