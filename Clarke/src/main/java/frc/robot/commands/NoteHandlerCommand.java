// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteHandler;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NoteHandler noteHandler;
  private final DoubleSupplier intakeSpeedAxis;

  public NoteHandlerCommand(NoteHandler noteHandler, DoubleSupplier intakeSpeedAxis) {
    this.noteHandler = noteHandler;
    this.intakeSpeedAxis = intakeSpeedAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteHandler.setIntakeSpeed(intakeSpeedAxis.getAsDouble());
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
