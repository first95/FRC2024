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
  private CANSparkMax topIntakeRoller, bottomIntakeRoller;
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    topIntakeRoller = new CANSparkMax(IntakeConstants.TOP_ROLLER_ID, MotorType.kBrushless);
    bottomIntakeRoller = new CANSparkMax(IntakeConstants.BOTTOM_ROLLER_ID, MotorType.kBrushless);
    topIntakeRoller.restoreFactoryDefaults();
    bottomIntakeRoller.restoreFactoryDefaults();

    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, IntakeConstants.STATUS_FRAME_0_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, IntakeConstants.STATUS_FRAME_1_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, IntakeConstants.STATUS_FRAME_2_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, IntakeConstants.STATUS_FRAME_3_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, IntakeConstants.STATUS_FRAME_4_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, IntakeConstants.STATUS_FRAME_5_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, IntakeConstants.STATUS_FRAME_6_PERIOD);

    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, IntakeConstants.STATUS_FRAME_0_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, IntakeConstants.STATUS_FRAME_1_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, IntakeConstants.STATUS_FRAME_2_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, IntakeConstants.STATUS_FRAME_3_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, IntakeConstants.STATUS_FRAME_4_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, IntakeConstants.STATUS_FRAME_5_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, IntakeConstants.STATUS_FRAME_6_PERIOD);

    topIntakeRoller.burnFlash();
    bottomIntakeRoller.burnFlash();
  }

  public void runRollers(double speed) {
    topIntakeRoller.set(speed);
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
