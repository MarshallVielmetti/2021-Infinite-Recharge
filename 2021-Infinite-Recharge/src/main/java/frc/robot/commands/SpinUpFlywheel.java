package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpFlywheel extends CommandBase implements ShootingDependency {

    private ShooterSubsystem m_shooterSubsystem;

    public SpinUpFlywheel(ShooterSubsystem shooterSubsystem) {
        this.addRequirements(shooterSubsystem);
        this.m_shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.smartSpin();
    }

    @Override
    public void execute() {
        // Don't need to do anything here
    }

    /**
     * Returns true IFF flywheel is at speed
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }

    // TODO
    public boolean getReadyToShoot() {
        return m_shooterSubsystem.getIsAtSpeed();
    }
}
