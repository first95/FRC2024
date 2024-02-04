// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private Rotation2d ang;
  private BooleanSupplier upSup, downSup;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(Shooter shooter, BooleanSupplier upSup, BooleanSupplier downSup) {
    this.shooter = shooter;
    this.upSup = upSup;
    this.downSup = downSup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ang = shooter.getArmAngle();
    shooter.setArmAngle(ang);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (upSup.getAsBoolean()) {
      ang = ang.plus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    } else if (downSup.getAsBoolean()) {
      ang = ang.minus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    }
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
