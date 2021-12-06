package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoldSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Only feeds to shooter when conditions for shooting are met.
 */
public class SmartFeed extends CommandBase {

    private final HoldSubsystem m_hold;
    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter;
    private final HoodSubsystem m_hood;

    public SmartFeed(HoldSubsystem hold, TurretSubsystem turret, ShooterSubsystem shooter, HoodSubsystem hood) {
        this.m_hold = hold;
        this.m_turret = turret;
        this.m_shooter = shooter;
        this.m_hood = hood;

        addRequirements(hold); // Needs to access all these subsystems, but hold is only requirement.
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // if (m_turret.atSetpoint() && m_shooter.getIsAtSpeed() &&
        // m_hood.isAtPosition())

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
