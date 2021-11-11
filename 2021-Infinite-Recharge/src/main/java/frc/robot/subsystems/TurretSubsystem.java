package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Includes StateSpace control for the flywheel
 * 
 */
public class TurretSubsystem extends SubsystemBase {

    private final CANSparkMax m_flywheelMotor = new CANSparkMax(kFlywheelMotorID, MotorType.kBrushless);
    private final CANSparkMax m_hoodMotor = new CANSparkMax(kHoodMotorID, MotorType.kBrushless);
    private final TalonSRX m_turretMotor = new TalonSRX(kTurretMotorID);

    private final SensorCollection m_turretEncoder = m_turretMotor.getSensorCollection();
    private final CANEncoder m_flywheelEncoder = m_flywheelMotor.getEncoder();
    private final CANEncoder m_hoodEncoder = m_hoodMotor.getEncoder();

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
    private final KalmanFilter<N1, N1, N1> m_flywheelObserver = new KalmanFilter<>(Nat.N1(), Nat.N1(), m_flywheelPlant,
            VecBuilder.fill(kFlywheelModelAccuracy), VecBuilder.fill(kFlywheelEncoderAccuracy), 0.020);

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
            m_flywheelPlant, VecBuilder.fill(kFlywheelQelmsVTolerance), VecBuilder.fill(kFlywheelRelmsControlEffort),
            0.02);

    // The state-space loop combines a controller, observer, feedforward and plant
    // for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_flywheelController,
            m_flywheelObserver, 12.0, 0.020);

    private final PIDController m_turretPIDController = new PIDController(kTurretKp, kTurretKi, kTurretKd);
    // private final PIDController m_hoodPIDController = new PIDController(kHoodKp,
    // kHoodKi, kHoodKd);
    private final CANPIDController m_hoodPIDController = m_hoodMotor.getPIDController();

    public TurretSubsystem() {
        // SET UP THE MOTORS
        m_turretMotor.setNeutralMode(NeutralMode.Brake);
        m_turretMotor.setInverted(false);

        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        m_flywheelMotor.setIdleMode(IdleMode.kCoast);

        // SET UP THE ENCODERS
        // Resets turret encoder position
        m_turretEncoder.setQuadraturePosition(0, 0);
        m_hoodEncoder.setPosition(0);

    }
}
