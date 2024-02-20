// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.Constants.NoteHandlerSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final DoubleSupplier intakeSpeedAxis;
  private final BooleanSupplier shooterButtonSupplier;

  private enum State {
    SHOOTING, IDLE, HOLDING, SPOOLING, AUTO_SPOOLING, AUTO_SHOOTING, INDEXING_REV, INDEXING_FWD
  }

  private State currentState;
  private double intakeSpeed, portShootingSpeed, starboardShootingSpeed, loaderSpeed, commandedIntakeSpeed, portSpeed, starboardSpeed;
  private Rotation2d autoArmAngle, armAngle;
  private boolean sensorvalue, shooterbutton, shooterAtSpeed, autoShooting, onTarget, armInPosition;

  public NoteHandlerCommand(Shooter shooter, Intake intake, DoubleSupplier intakeSpeedAxis, BooleanSupplier shooterButtonSupplier) {
    this.shooter = shooter;
    this.intake = intake;
    this.intakeSpeedAxis = intakeSpeedAxis;
    this.shooterButtonSupplier = shooterButtonSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;
    portSpeed = NoteHandlerSpeeds.PORT_SHOOTER_SPEED;
    starboardSpeed = NoteHandlerSpeeds.STARBOARD_SHOOTER_SPEED;
    SmartDashboard.putNumber("PortSpeed", portSpeed);
    SmartDashboard.putNumber("StarboardSpeed", starboardSpeed);
    SmartDashboard.putNumber(Auton.ARM_ANGLE_KEY, ShooterConstants.ARM_LOWER_LIMIT.getRadians());
    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, false);
    SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, false);
    
    shooter.setShooterSpeed(portSpeed, starboardSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read inputs
    sensorvalue = shooter.getNoteSensor();
    shooterbutton = shooterButtonSupplier.getAsBoolean();
    commandedIntakeSpeed = intakeSpeedAxis.getAsDouble();
    shooterAtSpeed = (Math.abs(portSpeed - shooter.getPortShooterSpeed()) <= NoteHandlerSpeeds.SHOOTER_SPEED_TOLERANCE)
        &&
        (Math.abs(starboardSpeed - shooter.getStarboardShooterSpeed()) <= NoteHandlerSpeeds.SHOOTER_SPEED_TOLERANCE);
    armInPosition = shooter.armAtGoal();
    portSpeed = SmartDashboard.getNumber("PortSpeed", portSpeed);
    starboardSpeed = SmartDashboard.getNumber("StarboardSpeed", starboardSpeed);
    autoShooting = SmartDashboard.getBoolean(Auton.AUTO_SHOOTING_KEY, false);
    onTarget = SmartDashboard.getBoolean(Auton.ON_TARGET_KEY, false);
    autoArmAngle = Rotation2d.fromRadians(SmartDashboard.getNumber(Auton.ARM_ANGLE_KEY, ShooterConstants.ARM_LOWER_LIMIT.getRadians()));
    
    // Execute statemachine logic
    switch (currentState) {
      case IDLE:
        // Set desired outputs in this state
        intakeSpeed = commandedIntakeSpeed;
        loaderSpeed = (commandedIntakeSpeed > 0) ? NoteHandlerSpeeds.LOADER_INTAKE_SPEED : NoteHandlerSpeeds.LOADER_IDLE_SPEED;
        portShootingSpeed = NoteHandlerSpeeds.PORT_IDLE_SPEED;
        starboardShootingSpeed = NoteHandlerSpeeds.STARBOARD_IDLE_SPEED;
        armAngle = ShooterConstants.ARM_LOWER_LIMIT;

        // Determine if we neeed to change state
        if (sensorvalue) {
          currentState = State.INDEXING_REV;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

        // Don't forget this-- things get real weird when every successive case can execute
        break;

      case SPOOLING:
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE_SPEED : NoteHandlerSpeeds.LOADER_IDLE_SPEED;
        armAngle = ShooterConstants.ARM_MANUAL_SHOT_ANGLE;

        if (!shooterbutton) {
          currentState = sensorvalue ? State.HOLDING : State.IDLE;
        }
        if (shooterbutton && shooterAtSpeed && armInPosition) {
          currentState = State.SHOOTING;
        }

        break;
      
      case AUTO_SPOOLING:
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE_SPEED : NoteHandlerSpeeds.LOADER_IDLE_SPEED;
        armAngle = autoArmAngle;

        if (!autoShooting) {
          currentState = sensorvalue ? State.HOLDING : State.IDLE;
        }
        if (autoShooting && shooterAtSpeed && armInPosition && onTarget) {
          currentState = State.AUTO_SHOOTING;
        }
        
      break;

      case SHOOTING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = NoteHandlerSpeeds.LOADER_FIRING_SPEED;
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        armAngle = ShooterConstants.ARM_MANUAL_SHOT_ANGLE;

        if (!shooterbutton) {
          currentState = sensorvalue ? State.HOLDING : State.IDLE;
        }

      break;
      
      case AUTO_SHOOTING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = NoteHandlerSpeeds.LOADER_FIRING_SPEED;
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        armAngle = autoArmAngle;

        if (!autoShooting) {
          currentState = sensorvalue ? State.HOLDING : State.IDLE;
        }

      break;

      case INDEXING_REV:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = -NoteHandlerSpeeds.LOADER_INDEXING_SPEED;
        portShootingSpeed = NoteHandlerSpeeds.PORT_IDLE_SPEED;
        starboardShootingSpeed = NoteHandlerSpeeds.STARBOARD_IDLE_SPEED;
        armAngle = ShooterConstants.ARM_LOWER_LIMIT;

        if (!sensorvalue) {
          currentState = State.INDEXING_FWD;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      
      break;

      case INDEXING_FWD:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = NoteHandlerSpeeds.LOADER_INDEXING_SPEED;
        portShootingSpeed = NoteHandlerSpeeds.PORT_IDLE_SPEED;
        starboardShootingSpeed = NoteHandlerSpeeds.STARBOARD_IDLE_SPEED;
        armAngle = ShooterConstants.ARM_LOWER_LIMIT;

        if (sensorvalue) {
          currentState = State.HOLDING;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      
      break;

      case HOLDING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE_SPEED;
        loaderSpeed = commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE_SPEED : NoteHandlerSpeeds.LOADER_IDLE_SPEED;
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        armAngle = ShooterConstants.ARM_LOWER_LIMIT;

        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

        break;
    }

    // Apply outputs
    intake.runRollers(intakeSpeed);
    shooter.setShooterSpeed(portShootingSpeed, starboardShootingSpeed);
    shooter.runLoader(loaderSpeed);
    shooter.setArmAngle(armAngle);

    SmartDashboard.putString("currentState", currentState.toString());
    SmartDashboard.putNumber("intakeSpeed", intakeSpeed);
    SmartDashboard.putNumber("shooterSpeed", portShootingSpeed);
    SmartDashboard.putNumber("loaderSpeed", loaderSpeed);
    SmartDashboard.putBoolean("sensor", sensorvalue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
