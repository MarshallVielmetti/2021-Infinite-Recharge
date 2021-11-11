package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.HoldConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoldSubsystem extends SubsystemBase {
    private final CANSparkMax m_holdMotor = new CANSparkMax(kHoldMotorID, MotorType.kBrushless);

    // TODO Add pulsing functionality
    // TODO Add reversibility

    public HoldSubsystem() {
        m_holdMotor.setInverted(kHoldMotorInverted);
        m_holdMotor.setClosedLoopRampRate(1.5);
        m_holdMotor.setSmartCurrentLimit(80, 80);
        m_holdMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Realized that I probably don't need a complex control system for this either
     * At some point maybe make it pulse but why
     * 
     * @param percentPower between -1 and 1
     */
    private void setHoldSpeed(double percentPower) {
        m_holdMotor.set(percentPower);
    }

    public void defaultHold() {
        this.setHoldSpeed(kDefaultHoldPower);
    }

    public void stop() {
        this.setHoldSpeed(0);
    }
}
