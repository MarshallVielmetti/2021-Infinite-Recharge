package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private boolean m_isFront = true; // Variable to control which direction is forwards

    private final CANSparkMax m_motorL1 = new CANSparkMax(kLeftMotor1ID, MotorType.kBrushless);
    private final CANSparkMax m_motorL2 = new CANSparkMax(kLeftMotor2ID, MotorType.kBrushless);
    private final CANSparkMax m_motorL3 = new CANSparkMax(kLeftMotor3ID, MotorType.kBrushless);

    private final CANSparkMax m_motorR1 = new CANSparkMax(kRightMotor1ID, MotorType.kBrushless);
    private final CANSparkMax m_motorR2 = new CANSparkMax(kRightMotor2ID, MotorType.kBrushless);
    private final CANSparkMax m_motorR3 = new CANSparkMax(kRightMotor3ID, MotorType.kBrushless);

    // Not sure if kEncoderResolution is correct.
    // This doesn't makes sense. We need shaft encoders.
    private final CANEncoder m_leftEncoder = m_motorL1.getEncoder(EncoderType.kQuadrature, kEncoderResolution);
    private final CANEncoder m_rightEncoder = m_motorR1.getEncoder(EncoderType.kQuadrature, kEncoderResolution);

    private final SpeedControllerGroup m_leftMotorGroup = new SpeedControllerGroup(m_motorL1, m_motorL2, m_motorL3);
    private final SpeedControllerGroup m_rightMotorGroup = new SpeedControllerGroup(m_motorR1, m_motorR2, m_motorR3);

    private final Gyro m_gyro = new ADXRS450_Gyro(); // TODO Make sure actually on the robot

    private final PIDController m_leftPIDController = new PIDController(0.6, 0.00001, 0.003); // TODO Assign values
    private final PIDController m_rightPIDController = new PIDController(0.6, 0.00001, 0.003); // TODO Assign values

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private DifferentialDriveOdometry m_odometry;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 3);

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    public DriveSubsystem() {
        m_gyro.reset();

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotorGroup.setInverted(kRightMotorInverted);
        m_leftMotorGroup.setInverted(kLeftMotorInverted);

        m_motorL2.follow(m_motorL1);
        m_motorL3.follow(m_motorL1);

        m_motorR2.follow(m_motorR1);
        m_motorR3.follow(m_motorR1);

        // TODO Not sure if this works
        // NEED to set these valus for this drive system to work correctly!
        // Could potentially be causing the encoder errors??
        m_leftEncoder.setPositionConversionFactor(kPositionFactor);
        m_rightEncoder.setPositionConversionFactor(kPositionFactor);

        m_leftEncoder.setVelocityConversionFactor(kVelocityFactor);
        m_rightEncoder.setVelocityConversionFactor(kVelocityFactor);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(),
                speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(),
                speeds.rightMetersPerSecond);
        this.m_motorL1.setVoltage(leftOutput + leftFeedforward);
        this.m_motorR1.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }
}
