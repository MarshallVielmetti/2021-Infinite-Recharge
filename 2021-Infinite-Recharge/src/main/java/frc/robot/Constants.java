// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * TODO : NEED TO UPDATE TO MEET ACTUAL ROBOT SPECS The Constants class provides a convenient place
 * for teams to hold robot-wide numerical or boolean constants. This class should not be used for
 * any other purpose. All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>ALL UNITS ARE IN METERS (OR A DERIVATIVE) UNLESS OTHERWISE NOTED
 */
public final class Constants {
  public static class DriveConstants {
    public static final double kMaxSpeed = 3.0; // m/s
    public static final double kMaxAngularSpeed = 2 * Math.PI; // One rotation per second

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

    // This needs to be the distance of movement provided by one motor rotation
    // Not the wheel circumference!
    public static final double kPositionFactor = kWheelCircumference;
    public static final double kVelocityFactor = kWheelCircumference;
  }

  public static class HoldConstants {
    public static final int kHoldMotorID = 22;

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

    public static final int kFlywheelMotor1ID = 24;
    public static final int kFlywheelMotor2ID = 25;

    public static final double kFlywheelMomentofIntertia = 0.00032; // kg / m^2
    // Reduction between motors and encoder, as output over input. If the flywheel
    // spins slower than
    // the motors, this number should be greater than one.
    public static final double kFlywheelGearing = 1.0; // There is not reduction on our flywheels???

    public static final double kFlywheelEncoderAccuracy = 0.01;

    public static final double kFlywheelModelAccuracy = 3.0;

    public static final double kFlywheelQelmsVTolerance = 3.0; // radians per second

    public static final double kFlywheelRelmsControlEffort = 12.0; // voltage

    public static final double kFlywheelEncoderVelocityConversion = 1;

    public static final double kSpinupRadPerSec = 1; // Radians per second speedup

    public static final double kDesiredVelocity = 15; // Radians per second ?

    public static final double kVelocityMargin =
        kDesiredVelocity * 0.05; // Allows a 5% margin - rads / second
  }

  public static class TurretConstants {
    public static final int kTurretMotorID = 27;
    // TODO PID constants for the turret
    public static double kP = 0.3;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.2;
    public static final double kMinOutput = -0.2;
    public static final double kmaxRPM = 5700; // RPM

    public static final double kMaxVel = 2000; // RPM
    public static final double kMinVel = 200;
    public static final double kMaxAcc = 1500; // RPM
    public static final double kAllowedErr = 0.02;

    public static final int kTurretEncoderMax = 1000;
    public static final int kTurretEncoderMin = -1000;

    // TODO - For the motor feedforward
    public static double kS = 2;
    public static double kV = 1;

    public static final double kTurretVMax = 0.2; // Angular velocity, rad/s
    public static final double kTurretAMax = 0.05; // Angular acceleartion rad/s^2

    // TODO - Figure out how to get encoder from TalonSRX
    public static final int[] kEncoderPorts = {0, 1};

    // TODO
    public static final double kEncoderDistancePerPulse = 0.001; // Radians / pulse

    public static final double kTurretPIDTolerance = 0.5; // TODO
    public static final double kTurretVisionXTolerance = 2; // Pixels?

    public static final double kPixelScalar = 0.001;
  }

  public static class HoodConstants {

    public static final int kHoodMotorID = 26;

    // TODO Find good values
    public static final double kP = 0.1;
    public static final double kI = 0.0001;
    public static final double kD = 0.002;
    public static final double kIz = 0;
    public static final double kFF = 0.0;
    public static final double kMaxOutput = 0.1;
    public static final double kMinOutput = -0.1;
    public static final double kMaxRPM = 150; // RPM

    public static final double kMaxVel = 2000; // RPM
    public static final double kMinVel = 200;
    public static final double kMaxAcc = 1500; // RPM
    public static final double kAllowedErr = 0.02;

    // PID coefficients

    // TODO - What do these factors do???
    public static final double kHoodEncoderPositionConversion = 1;
    public static final double kHoodEncoderVelocityConversion = 1;
  }

  public static class OIConstants {}
}
