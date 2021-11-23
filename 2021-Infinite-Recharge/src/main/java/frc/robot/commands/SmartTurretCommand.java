package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HoldSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * 1st - speeds up the fly wheel & targets turret w/Vision 2nd - Maintains
 * speeds, begins doing smart shoot
 */
public class SmartTurretCommand extends ParallelCommandGroup {

  private TurretSubsystem m_turretSubsystem;
  private HoodSubsystem m_hoodSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private HoldSubsystem m_holdSubsystem;

  /**
   * Smart Turret Command - Handles everything to do with shooting at a target.
   *
   * @param turretSubsystem  - turret subsystem
   * @param hoodSubsystem    - hood subsystem
   * @param shooterSubsystem - shooter subsystem
   * @param holdSubsystem    - hold subsystem
   *
   *                         This command does the entire process of spinning the
   *                         flywheel, and ligning turret and hood. Then it should
   *                         maintain those positions while also shooting balls /
   *                         spinning the spindexer and hold eventually graduate
   *                         this to doing the smart spin or whatever
   *
   *                         Parallel setup new parallel, one cmd grp maintains
   *                         and another cmd group runs hood
   *
   *                         Should run the second parallel command group until
   *                         the whileHeld condition is met.
   */
  public SmartTurretCommand(TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem, HoldSubsystem holdSubsystem) {

  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      new ResetMechs(m_turretSubsystem, m_hoodSubsystem, m_shooterSubsystem, m_holdSubsystem);
    } else {
      // Should have met end condition then.
    }
  }
}
