// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveBase;

/** An example command that uses an example subsystem. */
public class AutoShoot extends Command {
  private final PIDController thetaController;
  private final SwerveBase swerve;
  private final double timeout;
  private final Timer timer;

  private double angle, omega;
  private Pose2d currentPose;
  private Translation3d speakerLocation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot(SwerveBase swerve) {
    this(swerve, -1);
  }
  public AutoShoot(SwerveBase swerve, double timeoutSeconds) {
    thetaController = new PIDController(
      Drivebase.HEADING_KP,
      Drivebase.HEADING_KI,
      Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Drivebase.HEADING_TOLERANCE);
    this.swerve = swerve;
    timeout = timeoutSeconds;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, false);
    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, true);

    if (timeout >= 0) {
      timer.reset();
      timer.start();
    }

    var alliance = swerve.getAlliance();
    if (alliance == Alliance.Blue) {
      speakerLocation = Vision.BLUE_SPEAKER_POS;
    } else if (alliance == Alliance.Red) {
      speakerLocation = Vision.RED_SPEAKER_POS;
    } else {
      DriverStation.reportError("AutoShoot aborted because no alliance is set!", false);
      cancel();
    }

    currentPose = swerve.getPose();
    var shoulderLocation = 
      // package current pose into translation3d
      new Translation3d(currentPose.getX(), currentPose.getY(), 0)
      // add mount location of arm, after converting to field-relative coordinates
      .plus(
        ShooterConstants.ARM_PIVOT_LOCATION.rotateBy(
          new Rotation3d(0, 0, -currentPose.getRotation().getRadians())
        )
      );  
    var posDelta = speakerLocation.minus(shoulderLocation);

    // Elevation (linear approximation)
    SmartDashboard.putNumber(Auton.ARM_ANGLE_KEY,
      Math.atan(
        posDelta.getZ() /
        posDelta.toTranslation2d().getNorm()
      )
    );

    // Azimuth:
    // Note that we shoot from the back of the robot, so it may seem like this is backwards what it should be.
    angle = new Rotation2d(posDelta.getX(), posDelta.getY())
      .plus(Auton.AUTO_SHOOT_AZIMUTH_ADJUSTMENT)
      .plus(Rotation2d.fromDegrees(180)).getRadians();

    thetaController.reset();

    SmartDashboard.putNumber("AutoShootHeading", angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    omega = thetaController.calculate(swerve.getPose().getRotation().getRadians(), angle);
    omega = (Math.abs(omega) < Drivebase.HEADING_MIN_ANGULAR_CONTROL_EFFORT) ? 0 : omega;
    swerve.drive(new Translation2d(), omega, true, false);

    if (thetaController.atSetpoint()) {
      SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, true);
      swerve.setDriveBrake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeout < 0) {
      return false;
    } else {
      return timer.get() >= timeout;
    }
  }
}
