package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor = new CANSparkMax(kTurretMotorID, MotorType.kBrushless);
  private final CANEncoder m_encoder = m_motor.getEncoder();
  private final CANPIDController m_pidController = m_motor.getPIDController();

  private final boolean debug = true;
  private double setPos = 0;

  public TurretSubsystem() {

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

  public void setDesiredPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void incrementSetpos(double pixels) {
    this.setDesiredPosition(this.setPos + pixels * kPixelScalar);
  }

  private void initDebug() {
    SmartDashboard.putNumber("Turret Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Turret Set Position", 0);
  }

  @Override
  public void periodic() {
    // Check if hitting limit switch? and if so zero and make sure moving in the
    // opposite direction?
    if (this.debug) {
      this.doDebug();
    }
  }

  public void zero() {
    // TODO move until limit switch
  }

  private void doDebug() {
    double setPos = SmartDashboard.getNumber("Turret Set Position", 0);
    if (setPos != this.setPos) {
      this.setPos = setPos;
      this.setDesiredPosition(this.setPos);
    }

    SmartDashboard.putNumber("Turret Position", m_encoder.getPosition());
  }
}
