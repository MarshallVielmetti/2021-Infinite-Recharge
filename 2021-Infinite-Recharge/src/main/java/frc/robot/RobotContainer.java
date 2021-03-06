// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ExampleCommand;
// import frc.robot.commands.ResetMechs;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HoldSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Limelight m_limelight = new Limelight();
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final HoldSubsystem m_holdSubsystem = new HoldSubsystem();
  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final CustomXBox m_xbox0 = new CustomXBox(0);
  private final CustomXBox m_xbox1 = new CustomXBox(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Drive controller gets drive control
    // m_driveSubsystem.setDefaultCommand(
    // new ArcadeDrive(m_driveSubsystem, m_xbox0::getLeftY, m_xbox0::getRightX));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // MECH CONTROLLER BINDINGS

    // INTAKE BINDINGS

    // A Button - Run Intake & Spin Hold
    new JoystickButton(m_xbox1, 1)
        .whenHeld(
            new StartEndCommand(
                () -> {
                  m_intakeSubsystem.setDown(); // Make sure extended
                  m_intakeSubsystem.defaultIntake();
                  m_holdSubsystem.defaultHold();
                },
                () -> {
                  m_holdSubsystem.stop();
                  m_intakeSubsystem.stop();
                },
                m_intakeSubsystem,
                m_holdSubsystem));

    // // X Button - Exhaust Intake
    new JoystickButton(m_xbox1, 3)
        .whenHeld(
            new StartEndCommand(
                () -> {
                  m_intakeSubsystem.setDown();
                  m_intakeSubsystem.defaultExhaust();
                },
                () -> {
                  m_intakeSubsystem.stop();
                },
                m_intakeSubsystem));

    // // DPad Up - Intake Up
    new POVButton(m_xbox1, 0)
        .whenPressed(new InstantCommand(m_intakeSubsystem::setUp, m_intakeSubsystem));

    // // DPad Down - Intake Down
    new POVButton(m_xbox1, 180)
        .whenPressed(new InstantCommand(m_intakeSubsystem::setDown, m_intakeSubsystem));

    // TURRET BINDINGS

    // Y Button

    new JoystickButton(m_xbox1, 4)
        .whenHeld(new InstantCommand(() -> m_shooterSubsystem.smartSpin(), m_shooterSubsystem));

    // new JoystickButton(m_xbox1, 4)
    // .whenHeld(new SmartTurretCommand(m_turretSubsystem, m_hoodSubsystem,
    // m_shooterSubsystem, m_holdSubsystem))
    // .whenReleased(new ResetMechs(m_turretSubsystem, m_hoodSubsystem,
    // m_shooterSubsystem, m_holdSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousMode() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
