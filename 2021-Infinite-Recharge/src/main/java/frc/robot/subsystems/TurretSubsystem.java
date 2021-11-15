package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/**
 * interface with "enable", "disable", and "setgoal" Should make default command
 * to just maintain position
 */
public class TurretSubsystem extends ProfiledPIDSubsystem {

    private final TalonSRX m_turretMotor = new TalonSRX(kTurretMotorID);
    private final Encoder m_turretEncoder = new Encoder(kEncoderPorts[0], kEncoderPorts[1]);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kTurretKs, kTurretKv, kTurretKa);

    public TurretSubsystem() {
        super(new ProfiledPIDController(kTurretKp, kTurretKi, kTurretKd,
                new TrapezoidProfile.Constraints(kTurretVMax, kTurretAMax), 0.02));

        m_turretEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
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
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_turretMotor.set(ControlMode.Current, output + feedforward);
    }

    public void setVisionGoal(double tx) {

    }
}
