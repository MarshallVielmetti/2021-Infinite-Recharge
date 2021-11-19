package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HoldSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import static frc.robot.Constants.TurretConstants.kTurretVisionXTolerance;

/**
 * 1st - speeds up the fly wheel & targets turret w/Vision 2nd - Maintains
 * speeds, begins doing smart shoot
 */
public class SmartTurretCommand extends SequentialCommandGroup {
    private NetworkTable m_visionTable;
    private double m_visX;
    private double m_visY;
    private double m_visA;

    private double m_turretXTolerance = 15;

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
        addCommands(parallel(
                // Smart Spin turret to shooting velocity
                // "finishes" command when turret has reached margin of error around desired
                // velocity
                new FunctionalCommand(shooterSubsystem::smartSpin, null, (interrupted) -> {
                    // Stops energy being sent to flywheel only if interrupted
                    if (interrupted) {
                        shooterSubsystem.stop();
                    }
                }, shooterSubsystem::getIsAtSpeed, shooterSubsystem), new FunctionalCommand(
                        // Make sure turret subsystem enabled
                        turretSubsystem::enable,
                        // This might cause problems to be honest.
                        // Might end up swapping this statement to be first.
                        () -> turretSubsystem.setVisionGoal(m_visX),
                        // Stop moving at the end of the command
                        // Would rather not mess around with enable / disable.
                        // I'd like it to maintain it's position
                        (interrupted) -> {
                            // Only disable PID if interrupted
                            if (interrupted) {
                                turretSubsystem.disable();
                            }
                        },
                        // End condition
                        () -> Math.abs(m_visX) <= kTurretVisionXTolerance, turretSubsystem),
                new FunctionalCommand(hoodSubsystem::enable, onExecute, onEnd, isFinished, hoodSubsystem)));
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
