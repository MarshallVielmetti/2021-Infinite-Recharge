package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class TurretSubsystem2 extends PIDSubsystem {

    private final TalonSRX m_motor = new TalonSRX(kTurretMotorID);
    private final SensorCollection m_encoder = m_motor.getSensorCollection();

    public TurretSubsystem2() {
        super(new PIDController(kTurretKp, kTurretKi, kTurretKd));
        getController().setTolerance(kTurretPIDTolerance);
        m_encoder.get();
    }

    /**
     * The getMeasurement method returns the current measurement of the process
     * variable. The PIDSubsystem will automatically call this method from its
     * periodic() block, and pass its value to the control loop.
     */
    public double getMeasurement() {
        return m_encoder.getDistance();
    }
}
