package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/**
 * interface with "enable", "disable", and "setgoal" Should make default command
 * to just maintain position
 */
public class OLDTurretSubsystem extends ProfiledPIDSubsystem {

    private final TalonSRX m_turretMotor = new TalonSRX(kTurretMotorID);
    private final Encoder m_turretEncoder = new Encoder(kEncoderPorts[0], kEncoderPorts[1]);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kTurretKs, kTurretKv, kTurretKa);

    private final boolean debug = true;
    private double debugSetpos = 0;

    public OLDTurretSubsystem() {
        super(new ProfiledPIDController(kTurretKp, kTurretKi, kTurretKd,
                new TrapezoidProfile.Constraints(kTurretVMax, kTurretAMax), 0.02));

        // This encoderDistancePerPulse should be in fractions of radians per pulse of
        // quad encoder
        m_turretEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
        if (debug) {
            this.initDebug();
        }
    }

    // TODO
    /**
     * Returns the current measurement of the process variable Returns the sensor
     * reading to be used as the process variable
     */
    public double getMeasurement() {
        return m_turretEncoder.getDistance();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the setpoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_turretMotor.set(ControlMode.Current, output + feedforward);
    }

    public void setVisionGoal(double tx) {

    }

    public void doDebug() {
        double setPos = SmartDashboard.getNumber("Turret Set Position", 0);
        if (setPos != debugSetpos)
            setTarget(setPos);
    }

    public void initDebug() {
        SmartDashboard.putNumber("Turret Position", getMeasurement());
        SmartDashboard.putNumber("Turret Set Position", 0);
    }

    public void setTarget(double target) {
        if (target < kTurretEncoderMax && target > kTurretEncoderMin) {
            m_controller.setGoal(target);
        }
    }

}
