// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.NoteHandlerCommand;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake = new Intake();
  private final SwerveBase drivebase = new SwerveBase();
  private final Shooter shooter = new Shooter();
  private final TeleopDrive openRobotRel, closedRobotRel, openFieldRel, closedFieldRel;

  private final CommandJoystick driveController = new CommandJoystick(OperatorConstants.driveControllerPort);
  private final CommandJoystick headingController = new CommandJoystick(OperatorConstants.headingControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    openRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driveController.getY()) > 0.05) ? -driveController.getY() * 0.5 : 0,
      () -> (Math.abs(driveController.getX()) > 0.05) ? -driveController.getX() * 0.5 : 0,
      () -> headingController.getTwist() * 0.5, () -> false, true);
    
    closedRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driveController.getY()) > 0.05) ? -driveController.getY() * 0.5 : 0,
      () -> (Math.abs(driveController.getX()) > 0.05) ? -driveController.getX() * 0.5 : 0,
      () -> headingController.getTwist() * 0.5, () -> false, false);
    
    openFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driveController.getY()) > 0.05) ? -driveController.getY() * 0.5 : 0,
      () -> (Math.abs(driveController.getX()) > 0.05) ? -driveController.getX() * 0.5 : 0,
      () -> headingController.getTwist() * 0.5, () -> true, true);
    
    closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driveController.getY()) > 0.05) ? -driveController.getY() * 0.5 : 0,
      () -> (Math.abs(driveController.getX()) > 0.05) ? -driveController.getX() * 0.5 : 0,
      () -> headingController.getTwist() * 0.5, () -> true, false);
      drivebase.setDefaultCommand(closedFieldRel);

    NoteHandlerCommand noteManager = new NoteHandlerCommand(
      shooter,
      intake,
      () -> (operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis()),
      () -> operatorController.y().getAsBoolean(),
      () -> operatorController.rightBumper().getAsBoolean(),
      () -> operatorController.leftBumper().getAsBoolean()
    );
    
    shooter.setDefaultCommand(noteManager);

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
    //m_driverController.y().whileTrue(shooter.sysIdQuasiStarShooter(SysIdRoutine.Direction.kForward));
    //m_driverController.b().whileTrue(shooter.sysIdQuasiStarShooter(SysIdRoutine.Direction.kReverse));
    //m_driverController.a().whileTrue(shooter.sysIdDynStarShooter(SysIdRoutine.Direction.kForward));
    //m_driverController.x().whileTrue(shooter.sysIdDynStarShooter(SysIdRoutine.Direction.kReverse));
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
