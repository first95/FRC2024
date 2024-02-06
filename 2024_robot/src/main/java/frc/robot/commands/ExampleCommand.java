// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private Rotation2d ang;
  private double portSpeed, starboardSpeed;
  private BooleanSupplier upSup, downSup, portUpSup, portDownSup, sboardUpSup, sboardDownSup, loader;
  private boolean lastUp, lastDown, lastPortUp, lastPortDown, lastSboardUp, lastSboardDown;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(Shooter shooter, BooleanSupplier upSup, BooleanSupplier downSup, BooleanSupplier portUpSup, BooleanSupplier portDownSup, BooleanSupplier sboardUpSup, BooleanSupplier sboardDownSup, BooleanSupplier loader) {
    this.shooter = shooter;
    this.upSup = upSup;
    this.downSup = downSup;
    this.portDownSup = portDownSup;
    this.portUpSup = portUpSup;
    this.sboardDownSup = sboardDownSup;
    this.sboardUpSup = sboardUpSup;
    this.loader = loader;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ang = shooter.getArmAngle();
    shooter.setArmAngle(ang);
    starboardSpeed = 0;
    portSpeed = 0;
    lastDown = false;
    lastUp = false;
    lastPortDown = false;
    lastPortUp = false;
    lastSboardDown = false;
    lastSboardUp = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (upSup.getAsBoolean() && !lastUp && (ang.getRadians() <= ShooterConstants.ARM_UPPER_LIMIT.getRadians())) {
      ang = ang.plus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    } else if (downSup.getAsBoolean() && !lastDown) {
      ang = ang.minus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    }

    if (portUpSup.getAsBoolean() && !lastPortUp) {
      portSpeed += 0.05;
      //shooter.setShooterSpeed(portSpeed, starboardSpeed);
      shooter.setPortShooterRaw(portSpeed);
    } else if (portDownSup.getAsBoolean() && !lastPortDown && portSpeed > 0) {
      portSpeed -= 0.05;
      //shooter.setShooterSpeed(portSpeed, starboardSpeed);
      shooter.setPortShooterRaw(portSpeed);
    }

    if (sboardUpSup.getAsBoolean() && !lastSboardUp) {
      starboardSpeed += 0.05;
      //shooter.setShooterSpeed(portSpeed, starboardSpeed);
      shooter.setStarboardShooterRaw(starboardSpeed);
    } else if (sboardDownSup.getAsBoolean() && !lastSboardDown && starboardSpeed > 0) {
      starboardSpeed -= 0.05;
      //shooter.setShooterSpeed(portSpeed, starboardSpeed);
      shooter.setStarboardShooterRaw(starboardSpeed);
    }

    shooter.runLoader(loader.getAsBoolean() ? 0.8 : 0);

    lastDown = downSup.getAsBoolean();
    lastUp = upSup.getAsBoolean();
    lastPortDown = portDownSup.getAsBoolean();
    lastPortUp = portUpSup.getAsBoolean();
    lastSboardDown = sboardDownSup.getAsBoolean();
    lastSboardUp = sboardUpSup.getAsBoolean();

    SmartDashboard.putNumber("PortSpeed", portSpeed);
    SmartDashboard.putNumber("StarboardSpeed", starboardSpeed);
    SmartDashboard.putString("AngleCommand", ang.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
