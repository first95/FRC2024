// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax topintakeRoller, bottomIntakeRoller;
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    topintakeRoller = new CANSparkMax(IntakeConstants.TOP_ROLLER_ID, MotorType.kBrushless);
    bottomIntakeRoller = new CANSparkMax(IntakeConstants.BOTTOM_ROLLER_ID, MotorType.kBrushless);
    topintakeRoller.restoreFactoryDefaults();
    bottomIntakeRoller.restoreFactoryDefaults();
    topintakeRoller.burnFlash();
    bottomIntakeRoller.burnFlash();
  }

  public void runRollers(double speed) {
    topintakeRoller.set(speed);
    bottomIntakeRoller.set(-speed);
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
