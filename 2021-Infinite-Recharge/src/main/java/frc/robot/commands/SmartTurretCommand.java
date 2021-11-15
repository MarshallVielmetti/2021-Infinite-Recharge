package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * 1st - speeds up the fly wheel & targets turret w/Vision
 * 
 */
public class SmartTurretCommand extends SequentialCommandGroup {
    private NetworkTable m_visionTable;
    private double m_visX;
    private double m_visY;
    private double m_visA;

    private double m_turretXTolerance = 15;

    /**
     * Smart Turret Command - Handles everything to do with shooting at a target.
     * 
     * @param turretSubsystem  - turret subsystem
     * @param hoodSubsystem    - hood subsystem
     * @param shooterSubsystem - shooter subsystem
     * 
     *                         This command does the entire process of spinning the
     *                         flywheel, Aligning turret and hood, and spinning up
     *                         flywheel Then it should maintain those positions
     *                         while also shooting balls / spinning the spindexer
     *                         and hold eventually graduate this to doing the smart
     *                         spin or whatever
     * 
     *                         Parallel setup new parallel, one cmd grp maintains
     *                         and another cmd group runs hood
     */
    public SmartTurretCommand(TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem) {
        addCommands(parallel(
                // Smart Spin turret to shooting velocity
                new InstantCommand(shooterSubsystem::smartSpin, shooterSubsystem), new FunctionalCommand(
                        // Set goal to vision on command start
                        () -> turretSubsystem.enable(),
                        // This might cause problems to be honest.
                        // Might end up swapping this statement to be first.
                        () -> turretSubsystem.setVisionGoal(m_visX),
                        // Stop moving at the end of the command
                        // Would rather not mess around with enable / disable.
                        // I'd like it to maintain it's position
                        (interrupted) -> turretSubsystem.disable(),
                        // End condition
                        () -> Math.abs(m_visX) <= m_turretXTolerance)));
    }

    @Override
    public void execute() {

    }
}
