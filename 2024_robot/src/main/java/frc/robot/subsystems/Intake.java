// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeRoller;
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    intakeRoller = new CANSparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
    intakeRoller.restoreFactoryDefaults();

    intakeRoller.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    intakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, IntakeConstants.STATUS_FRAME_0_PERIOD);
    intakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, IntakeConstants.STATUS_FRAME_1_PERIOD);
    intakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, IntakeConstants.STATUS_FRAME_2_PERIOD);

    intakeRoller.burnFlash();
  }

  public void runRollers(double speed) {
    //intakeRoller.set(speed * IntakeConstants.MAX_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
