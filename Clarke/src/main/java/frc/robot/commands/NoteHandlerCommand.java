// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.NoteHandler;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final NoteHandler noteHandler;
  private final DoubleSupplier intakeSpeedAxis;

  private enum State {
    SHOOTING, IDLE, INTAKING
  }

  private State currentState;
  private double intakeSpeed, shootingSpeed, loaderSpeed;
  private boolean sensorvalue;
  
  public NoteHandlerCommand(NoteHandler noteHandler, DoubleSupplier intakeSpeedAxis) {
    this.noteHandler = noteHandler;
    this.intakeSpeedAxis = intakeSpeedAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(noteHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorvalue = noteHandler.getLoaderSensor();

    switch(currentState){
      case IDLE:
      if(sensorvalue == true){//and user input
        currentState = State.SHOOTING;
        intakeSpeed = 0;
      }
      if(sensorvalue == false){//and user input
        currentState = State.INTAKING;
      }
      break;

      case SHOOTING:
        if (sensorvalue == false){
          currentState = State.IDLE;
        }
        loaderSpeed=Constants.NoteHandlerConstants.LOADERSPEED;
        shootingSpeed=Constants.NoteHandlerConstants.SHOOTERSPEED;
      break;

      case INTAKING:
        if (sensorvalue == true){
          currentState = State.IDLE;
        }
        intakeSpeed = intakeSpeedAxis.getAsDouble();
      break;
    }
    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setLoaderSpeed(loaderSpeed);
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
