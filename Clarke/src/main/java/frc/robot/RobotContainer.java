// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.NoteHandlerCommand;
import frc.robot.drivebase.AbsoluteDrive;
import frc.robot.drivebase.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteHandler;
import frc.robot.subsystems.SwerveBase;

import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private final SwerveBase drivebase = new SwerveBase();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final AbsoluteDrive absoluteDrive, closedAbsoluteDrive;
  private final TeleopDrive openFieldRel, openRobotRel, closedFieldRel, closedRobotRel;

  private final NoteHandlerCommand noteHandlerController;
  private final NoteHandler noteHandler;

  private Alliance alliance;

  private SendableChooser<Command> driveModeSelector;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(OperatorConstants.DRIVE_CONTROLLER_PORT);
  CommandJoystick rotationController = new CommandJoystick(OperatorConstants.ANGLE_CONTROLLER_PORT);
  CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Test code field:
    if (SmartDashboard.getNumber("ShooterSpeed", 123456789) == 123456789) {
      SmartDashboard.putNumber("ShooterSpeed", 0);
    }

    absoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), true);

    closedAbsoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), false);

    openRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, true);
    
    closedRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, false);
    
    openFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, true);

    closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, false);

    driveModeSelector = new SendableChooser<>();
    driveModeSelector.setDefaultOption("AbsoluteDrive", absoluteDrive);
    driveModeSelector.addOption("Field Relative", openFieldRel);
    driveModeSelector.addOption("Robot Relative", openRobotRel);
    driveModeSelector.addOption("Absolute (Closed)", closedAbsoluteDrive);
    driveModeSelector.addOption("Field Relative (Closed)", closedFieldRel);
    driveModeSelector.addOption("Robot Relative (Closed)", closedRobotRel);
    
    SmartDashboard.putData(driveModeSelector);

    //SmartDashboard.putData("SetModuleGains", new InstantCommand(drivebase::setVelocityModuleGains).ignoringDisable(true));
    SmartDashboard.putNumber("ANGLE", 0);
    //SmartDashboard.putData("setAngle", new InstantCommand(() -> drivebase.setGyro(new Rotation2d(SmartDashboard.getNumber("ANGLE", 0)))).ignoringDisable(true));
    SmartDashboard.putData("sendAlliance", new InstantCommand(() -> {
      try {
        drivebase.setAlliance(DriverStation.getAlliance().get());
      } catch (NoSuchElementException e) {
        drivebase.setAlliance(null);
        DriverStation.reportWarning("Alliance failed to send!", false);
      }
      
    }).ignoringDisable(true));

    noteHandler = new NoteHandler();
    noteHandlerController =
      new NoteHandlerCommand(
        noteHandler,
        () -> operatorController.getLeftTriggerAxis(),
        () -> driverController.button(1).getAsBoolean());
    
    Command shooterTester = new RepeatCommand(
      new InstantCommand(() -> {
        noteHandler.setShooterSpeed(
          operatorController.start().getAsBoolean() ? SmartDashboard.getNumber("ShooterSpeed", 0) : 0
        );
        SmartDashboard.putNumber("RealShooterRPM", noteHandler.getShooterRPM());
      })
    );
    noteHandler.setDefaultCommand(shooterTester);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    driverController.button(2).onTrue((new InstantCommand(drivebase::zeroGyro)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }
  public void prepareDriveForTeleop() {
    drivebase.setDefaultCommand(closedAbsoluteDrive);
    absoluteDrive.setHeading(drivebase.getPose().getRotation());
    closedAbsoluteDrive.setHeading(drivebase.getPose().getRotation());
  }
  public void prepareDriveForAuto() {
    drivebase.setDefaultCommand(new RepeatCommand(new InstantCommand(() -> {}, drivebase)));
  }
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
