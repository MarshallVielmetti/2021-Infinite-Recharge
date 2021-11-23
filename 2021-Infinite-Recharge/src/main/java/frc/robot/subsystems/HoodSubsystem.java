package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Implements PID through the use of the built in rev robotics PID controller
 * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
 */
public class HoodSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(kHoodMotorID, MotorType.kBrushless);
  private final CANEncoder m_encoder = m_motor.getEncoder();
  private final CANPIDController m_pidController = m_motor.getPIDController();

  private final boolean debug = true;
  private double debugSetpos = 0;

  // If NOt in debug - comment this out
  private double kP = 5e-5;
  private double kI = 1e-6;
  private double kD = 0.00002;
  private double kIz = 0;
  private double kFF = 0.000156;
  private double kMaxOutput = 0.1;
  private double kMinOutput = -0.1;
  private double kMaxRPM = 5700;

  public HoodSubsystem() {
    if (this.debug) {
      this.initDebug();
    }

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     *
     * <p>- setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid controller in
     * Smart Motion mode - setSmartMotionMinOutputVelocity() will put a lower bound in RPM of the
     * pid controller in Smart Motion mode - setSmartMotionMaxAccel() will limit the acceleration in
     * RPM^2 of the pid controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError()
     * will set the max allowed error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);
  }

  @Override
  public void periodic() {

    // Check if hitting limit switch? and if so zero and make sure moving in the
    // opposite direction?
    if (this.debug) {
      this.doDebug();
    }
    verifyPosition();
  }

  public void zero() {
    // Drive until reaches non-existent limit switch
    // set encoder position to zero

  }

  public void setDesiredPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void verifyPosition() {
    if (m_encoder.getPosition() <= -1) {
      setDesiredPosition(-5);
    }
  }

  private void initDebug() {
    SmartDashboard.putNumber("Hood Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Hood Set Position", 0);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  private void doDebug() {
    double setPos = SmartDashboard.getNumber("Hood Set Position", 0);
    if (setPos != debugSetpos) {
      if (debugSetpos > 0) this.debugSetpos = setPos;
      else this.debugSetpos = 0;
      this.setDesiredPosition(debugSetpos);
    }

    SmartDashboard.putNumber("Hood Position", m_encoder.getPosition());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }
}
