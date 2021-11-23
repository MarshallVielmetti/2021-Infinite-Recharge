package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TargetTurret extends CommandBase implements ShootingDependency {

    private TurretSubsystem m_turretSubsystem;

    private NetworkTable m_visionTable;
    private NetworkTableEntry m_visX;
    private NetworkTableEntry m_visY;
    private NetworkTableEntry m_visA;
    private NetworkTableEntry m_ledState;

    private double m_x;
    private double m_y;
    private double m_area;

    private double m_turretXTolerance = 15;

    public TargetTurret(TurretSubsystem turretSubsystem) {
        this.addRequirements(turretSubsystem);
        this.m_turretSubsystem = turretSubsystem;
    }

    @Override
    public void initialize() {
        m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_visX = m_visionTable.getEntry("tx");
        m_visY = m_visionTable.getEntry("ty");
        m_visA = m_visionTable.getEntry("ta");
        m_ledState = m_visionTable.getEntry("ledMode");

        // 320 x 240
        m_x = m_visX.getDouble(0.0);
        m_y = m_visY.getDouble(0.0);
        m_area = m_visA.getDouble(0.0);
        m_ledState.forceSetDouble(3.0);
    }

    @Override
    public void execute() {
        // GET LIMELIGHT VALUES
        m_x = m_visX.getDouble(0.0);
        m_y = m_visY.getDouble(0.0);
        m_area = m_visA.getDouble(0.0);

    }

    public boolean getReadyToShoot() {
        return Math.abs(m_x) <= m_turretXTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_turretSubsystem.zero();
    }
}
