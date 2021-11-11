// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
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
    }

    public static class IntakeConstants {
    }

    public static class ShooterConstants {
    }

    public static class OIConstants {
    }
}
