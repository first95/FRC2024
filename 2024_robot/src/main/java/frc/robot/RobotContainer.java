// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveBase drivebase = new SwerveBase();
  private final Shooter shooter = new Shooter();
  private final TeleopDrive openRobotRel, closedRobotRel, openFieldRel, closedFieldRel;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Command rollerer = new RepeatCommand(
      new InstantCommand(
        () -> {m_exampleSubsystem.runRoller(m_driverController.getLeftTriggerAxis() - m_driverController.getRightTriggerAxis());}
      ));
    rollerer.addRequirements(m_exampleSubsystem);
    m_exampleSubsystem.setDefaultCommand(rollerer);

    openRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(m_driverController.getLeftY()) > 0.05) ? Math.pow(m_driverController.getLeftY(), 3) * 0.5 : 0,
      () -> (Math.abs(m_driverController.getLeftX()) > 0.05) ? Math.pow(m_driverController.getLeftX(),3) * 0.5 : 0,
      () -> Math.pow(m_driverController.getRightX(), 3) * 0.5, () -> false, true);
    
    closedRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(m_driverController.getLeftY()) > 0.05) ? Math.pow(m_driverController.getLeftY(), 3) * 0.5 : 0,
      () -> (Math.abs(m_driverController.getLeftX()) > 0.05) ? Math.pow(m_driverController.getLeftX(),3) * 0.5 : 0,
      () -> Math.pow(m_driverController.getRightX(), 3) * 0.5, () -> false, false);
    
    openFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(m_driverController.getLeftY()) > 0.05) ? Math.pow(m_driverController.getLeftY(), 3) * 0.5 : 0,
      () -> (Math.abs(m_driverController.getLeftX()) > 0.05) ? Math.pow(m_driverController.getLeftX(),3) * 0.5 : 0,
      () -> Math.pow(m_driverController.getRightX(), 3) * 0.5, () -> true, true);
    
    closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(m_driverController.getLeftY()) > 0.05) ? Math.pow(m_driverController.getLeftY(), 3) * 0.5 : 0,
      () -> (Math.abs(m_driverController.getLeftX()) > 0.05) ? Math.pow(m_driverController.getLeftX(),3) * 0.5 : 0,
      () -> Math.pow(m_driverController.getRightX(), 3) * 0.5, () -> true, false);
      drivebase.setDefaultCommand(closedFieldRel);

      shooter.setDefaultCommand(new ExampleCommand(
        shooter,
        () -> m_driverController.rightBumper().getAsBoolean(),
        () -> m_driverController.leftBumper().getAsBoolean(),
        () -> m_driverController.povUp().getAsBoolean(),
        () -> m_driverController.povDown().getAsBoolean(),
        () -> m_driverController.y().getAsBoolean(),
        () -> m_driverController.a().getAsBoolean(),
        () -> m_driverController.b().getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
