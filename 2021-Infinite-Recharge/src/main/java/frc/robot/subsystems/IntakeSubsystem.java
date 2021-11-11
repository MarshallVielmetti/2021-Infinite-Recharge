package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intakeMotor = new CANSparkMax(kIntakeMotorID, MotorType.kBrushless);

    private final DoubleSolenoid intakePiston = new DoubleSolenoid(2, 3);

    private boolean isExtended;

    public IntakeSubsystem() {
        m_intakeMotor.setInverted(kIntakeMotorInverted);
        m_intakeMotor.setClosedLoopRampRate(1);
        m_intakeMotor.setSmartCurrentLimit(80, 80);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);

        this.isExtended = false;
    }

    /**
     * Super simple controller
     * 
     * @param percentPower between -1 and 1
     */
    private void setIntakeSpeed(double percentPower) {
        if (isExtended) {
            m_intakeMotor.set(percentPower);
        } else {
            // Don't necessarily want to spin while the intake is up
            m_intakeMotor.set(0);
        }

    }

    public void defaultIntake() {
        this.setIntakeSpeed(kDefaultMotorPower);
    }

    /**
     * Only if it gets jammed or something
     */
    public void defaultExhaust() {
        this.setIntakeSpeed(kDefaultMotorPower * -1);
    }

    public void stop() {
        this.setIntakeSpeed(0);
    }

    public boolean getIsExtended() {
        return this.isExtended;
    }

    // TODO Verify that these are the right values
    public void toggleIntake() {
        if (this.isExtended) {
            intakePiston.set(Value.kForward);
        } else {
            intakePiston.set(Value.kReverse);
        }
        this.isExtended = !this.isExtended;
    }

    private void setState(boolean isExtended) {
        if (this.isExtended != isExtended)
            this.toggleIntake();
    }

    public void setDown() {
        this.setState(true);
    }

    public void setUp() {
        this.setState(false);
    }

}
