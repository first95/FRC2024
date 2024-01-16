// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.NoteHandlerConstants;
import frc.robot.subsystems.NoteHandler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final NoteHandler noteHandler;
  private final DoubleSupplier intakeSpeedAxis;
  private final BooleanSupplier shooterButtonSupplier;
  private enum State {
    SHOOTING, IDLE, HOLDING, SPOOLING
  }

  private State currentState;
  private double intakeSpeed, shootingSpeed, loaderSpeed, commandedIntakeSpeed;
  private boolean sensorvalue, shooterbutton, shooterAtSpeed;
  
  public NoteHandlerCommand(NoteHandler noteHandler, DoubleSupplier intakeSpeedAxis, BooleanSupplier shooterButtonSupplier) {
    this.noteHandler = noteHandler;
    this.intakeSpeedAxis = intakeSpeedAxis;
    this.shooterButtonSupplier = shooterButtonSupplier;
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
    shooterbutton = shooterButtonSupplier.getAsBoolean();
    commandedIntakeSpeed = intakeSpeedAxis.getAsDouble();
    shooterAtSpeed = (Math.abs(NoteHandlerConstants.SHOOTER_SPEED - noteHandler.getShooterRPM())
    <= NoteHandlerConstants.SHOOTER_SPEED_TOLERANCE);

    switch(currentState){
      case IDLE:
      intakeSpeed = commandedIntakeSpeed;
      loaderSpeed = (commandedIntakeSpeed > 0) ? NoteHandlerConstants.LOADER_SPEED : 0;
      shootingSpeed = 0;
      if (sensorvalue){
        currentState = State.HOLDING;
      }
      break;

      case SPOOLING:
        shootingSpeed = NoteHandlerConstants.SHOOTER_SPEED;
        intakeSpeed = 0;
        loaderSpeed = 0;

        if (!shooterbutton){
          currentState = State.HOLDING;
        }
        if (shooterbutton && shooterAtSpeed){
          currentState = State.SHOOTING;
        }
      break;

      case SHOOTING:
        intakeSpeed = 0;
        loaderSpeed = NoteHandlerConstants.LOADER_SPEED;
        shootingSpeed = NoteHandlerConstants.SHOOTER_SPEED;

        if (sensorvalue == false){
          currentState = State.IDLE;
        }
      break;
      
      case HOLDING:
        intakeSpeed = 0;
        loaderSpeed = 0;
        shootingSpeed = 0;
        if (shooterbutton){
          currentState = State.SPOOLING;
        }
      break;
    }
    
    noteHandler.setIntakeSpeed(intakeSpeed);
    noteHandler.setShooterRPM(shootingSpeed);
    noteHandler.setLoaderSpeed(loaderSpeed);
    
    SmartDashboard.putString("currentState",currentState.toString());
    SmartDashboard.putNumber("intakeSpeed", intakeSpeed);
    SmartDashboard.putNumber("shooterSpeed", shootingSpeed);
    SmartDashboard.putNumber("loaderSpeed", loaderSpeed);
    SmartDashboard.putBoolean("sensor", sensorvalue);
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
