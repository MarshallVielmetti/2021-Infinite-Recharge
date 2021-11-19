package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
        private final CANSparkMax m_motor1 = new CANSparkMax(kFlywheelMotor1ID, MotorType.kBrushless);
        private final CANSparkMax m_motor2 = new CANSparkMax(kFlywheelMotor1ID, MotorType.kBrushless);

        private final CANEncoder m_encoder = m_motor1.getEncoder();

        private double m_targetSpeed;

        // The plant holds a state-space model of our flywheel. This system has the
        // following properties:
        // States: [velocity], in radians per second.
        // Inputs (what we can "put in"): [voltage], in volts.
        // Outputs (what we can measure): [velocity], in radians per second.
        private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),
                        kFlywheelMomentofIntertia, kFlywheelGearing);

        // 1st Vec Builder - How accurate we think our model is
        // 2nd Vec Builder - How accurate we think our encoder
        // 0.02 - Default time between robot loops
        private final KalmanFilter<N1, N1, N1> m_flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(),
                        m_flywheelPlant, VecBuilder.fill(kFlywheelModelAccuracy),
                        VecBuilder.fill(kFlywheelEncoderAccuracy), 0.020);

        // 1s Vec Builder - qelms. Velocity error tolerance, in radians per second
        // Decrease this to more heavily penalize state excursion, or make the
        // controller behave more aggressively.

        // 2nd Vec Builder - relms. Control effort (voltage) tolerance. Decrease this to
        // more
        // heavily penalize control effort, or make the controller less aggressive. 12
        // is a good
        // starting point because that is the (approximate) maximum voltage of a
        // battery.

        private final LinearQuadraticRegulator<N1, N1, N1> m_flywheelController = new LinearQuadraticRegulator<>(
                        m_flywheelPlant, VecBuilder.fill(kFlywheelQelmsVTolerance),
                        VecBuilder.fill(kFlywheelRelmsControlEffort), 0.02);

        // The state-space loop combines a controller, observer, feedforward and plant
        // for easy control.
        private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(m_flywheelPlant,
                        m_flywheelController, m_flywheelObserver, 12.0, 0.020);

        public ShooterSubsystem() {
                m_motor1.setIdleMode(IdleMode.kCoast);
                m_motor2.setIdleMode(IdleMode.kCoast);
                m_motor2.follow(m_motor1);

                m_encoder.setPositionConversionFactor(2.0 * Math.PI * m_encoder.getCountsPerRevolution());
                m_targetSpeed = 0;
        }

        public void smartSpin() {
                m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));

        }

        public void smartStop() {
                m_loop.setNextR(VecBuilder.fill(0));
        }

        @Override
        public void periodic() {
                // Correct our Kalman filter's state vector estimate with encoder data.
                m_loop.correct(VecBuilder.fill(m_encoder.getVelocity()));

                // Update our LQR to generate new voltage commands and use the voltages to
                // predict the next
                // state with out Kalman filter.
                m_loop.predict(0.020);

                // Send the new calculated voltage to the motors.
                // voltage = duty cycle * battery voltage, so
                // duty cycle = voltage / battery voltage
                double nextVoltage = m_loop.getU(0);
                m_motor1.setVoltage(nextVoltage);
        }
}
