package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import static frc.robot.Constants.TurretConstants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class VisionTurret extends PIDSubsystem {

  private final Limelight m_limelight = Limelight.getInstance();

  private final CANSparkMax m_motor = new CANSparkMax(kTurretMotorID, MotorType.kBrushless);

  private final CANEncoder m_encoder = m_motor.getEncoder();
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kS, kV);

  private final boolean debug = true;
  private double setPos = 0;

  public VisionTurret() {
    super(new PIDController(kP, kI, kD));

    if (this.debug) {
      this.initDebug();
    }

    getController().setTolerance(kTurretVisionXTolerance);
    setSetpoint(0); // Want it always pointed right at the turret
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    m_motor.setVoltage(output + m_feedforward.calculate(setpoint));
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return m_limelight.getX();
  }

  @Override
  public void periodic() {
    // Check if hitting limit switch? and if so zero and make sure moving in the
    // opposite direction?
    if (this.debug) {
      this.doDebug();
    }
  }

  /** Initializes debug mode */
  private void initDebug() {
    SmartDashboard.putNumber("Turret X", getMeasurement());
    SmartDashboard.putNumber("Turret Encoder", m_encoder.getPosition());

    SmartDashboard.putNumber("Turret P", kP);
    SmartDashboard.putNumber("Turret I", kI);
    SmartDashboard.putNumber("Turret D", kD);
    SmartDashboard.putNumber("Turret FFS", kS);
    SmartDashboard.putNumber("Turret FFV", kV);
  }

  /**
   * Puts the X measurement from the limelight on the dashboard Not a lot to do in the debug loop
   * here TODO Add PID tuning ability
   */
  private void doDebug() {
    SmartDashboard.putNumber("Turret X", getMeasurement());
    SmartDashboard.putNumber("Turret Encoder", m_encoder.getPosition());

    double p = SmartDashboard.getNumber("Turret P", kP);
    double i = SmartDashboard.getNumber("Turret I", kI);
    double d = SmartDashboard.getNumber("Turret D", kD);
    double s = SmartDashboard.getNumber("Turret FFS", kS);
    double v = SmartDashboard.getNumber("Turret FFV", kV);

    PIDController controller = getController();

    if (p != kP) {
      controller.setP(p);
      kP = p;
    }
    if (i != kI) {
      controller.setI(i);
      kI = i;
    }
    if (d != kD) {
      controller.setD(d);
      kD = d;
    }
    if (s != kS || v != kV) {
      this.m_feedforward = new SimpleMotorFeedforward(s, v);
      kS = s;
      kV = v;
    }
  }
}
