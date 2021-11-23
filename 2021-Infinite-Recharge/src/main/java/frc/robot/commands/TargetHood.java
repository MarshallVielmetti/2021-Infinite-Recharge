package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TargetHood extends CommandBase implements ShootingDependency {

    public boolean getReadyToShoot() {
        return false; // TODO
    }
}
