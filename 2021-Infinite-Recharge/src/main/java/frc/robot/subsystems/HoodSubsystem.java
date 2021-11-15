package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Implements PID through the use of the built in rev robotics PID controller
 * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
 */
public class HoodSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kHoodMotorID, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_pidController = m_motor.getPIDController();

    public HoodSubsystem() {
        m_pidController.setP(kHoodKp);
        m_pidController.setI(kHoodKi);
        m_pidController.setD(kHoodKd);
        m_pidController.setIZone(kHoodkIz);
        m_pidController.setFF(kHoodKff);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        /**
         * Smart Motion coefficients are set on a CANPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid
         * controller in Smart Motion mode - setSmartMotionMinOutputVelocity() will put
         * a lower bound in RPM of the pid controller in Smart Motion mode -
         * setSmartMotionMaxAccel() will limit the acceleration in RPM^2 of the pid
         * controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError() will
         * set the max allowed error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        m_pidController.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);
    }

}
