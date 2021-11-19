package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HoldSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Stops the motion of everything used in the SmartTurret command Sends mechs to
 * limit switches,
 */
public class ResetMechs extends ParallelCommandGroup {

    public ResetMechs(TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem,
            HoldSubsystem holdSubsystem) {

    }

}
