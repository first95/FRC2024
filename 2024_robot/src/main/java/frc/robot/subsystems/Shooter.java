// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final CANSparkFlex portShooter, starboardShooter, shoulder;
  private final CANSparkMax loader; // shoulder2;
  private final RelativeEncoder portShooterEncoder, starboardShooterEncoder, shoulderEncoder;
  private final SparkPIDController portShooterPID, starboardShooterPID, shoulderPID;
  private final TrapezoidProfile shoulderProfile;
  // private final SparkLimitSwitch bottomLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  private final DigitalInput noteSensor;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private final ArmFeedforward shoulderFeedforward;

  private final SysIdRoutine shoulderCharacterizer, portShootCharacterizer, starboardShootCharacterizer;

  private Rotation2d armGoal;
  private final Timer timer;
  private TrapezoidProfile.State armSetpoint, profileStart;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    portShooter = new CANSparkFlex(ShooterConstants.PORT_SHOOTER_ID, MotorType.kBrushless);
    starboardShooter = new CANSparkFlex(ShooterConstants.STARBOARD_SHOOTER_ID, MotorType.kBrushless);
    loader = new CANSparkMax(ShooterConstants.LOADER_ID, MotorType.kBrushless);
    shoulder = new CANSparkFlex(ShooterConstants.SHOULDER_ID, MotorType.kBrushless);

    portShooter.restoreFactoryDefaults();
    starboardShooter.restoreFactoryDefaults();
    loader.restoreFactoryDefaults();
    shoulder.restoreFactoryDefaults();

    portShooter.setInverted(ShooterConstants.INVERT_PORT_SHOOTER);
    starboardShooter.setInverted(ShooterConstants.INVERT_STARBOARD_SHOOTER);
    loader.setInverted(ShooterConstants.INVERT_LOADER);
    shoulder.setInverted(ShooterConstants.INVERT_SHOULDER);

    portShooter.setIdleMode(IdleMode.kCoast);
    starboardShooter.setIdleMode(IdleMode.kCoast);
    loader.setIdleMode(IdleMode.kBrake);
    shoulder.setIdleMode(IdleMode.kBrake);

    portShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    starboardShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    loader.setSmartCurrentLimit(ShooterConstants.LOADER_CURRENT_LIMIT);
    shoulder.setSmartCurrentLimit(ShooterConstants.SHOULDER_CURRENT_LIMIT);

    portShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    portShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);

    portShooterEncoder = portShooter.getEncoder();
    starboardShooterEncoder = starboardShooter.getEncoder();
    shoulderEncoder = shoulder.getEncoder();

    shoulderEncoder.setPositionConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION);
    shoulderEncoder.setVelocityConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION / 60);

    shoulderEncoder.setPosition(ShooterConstants.ARM_LOWER_LIMIT.getRadians());

    portShooterPID = portShooter.getPIDController();
    starboardShooterPID = starboardShooter.getPIDController();
    shoulderPID = shoulder.getPIDController();

    portShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    portShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    portShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    portShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    starboardShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    starboardShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    starboardShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    starboardShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    shoulderPID.setP(ShooterConstants.SHOULDER_KP);
    shoulderPID.setI(ShooterConstants.SHOULDER_KI);
    shoulderPID.setD(ShooterConstants.SHOULDER_KD);
    shoulderPID.setFF(ShooterConstants.SHOULDER_KF);

    shoulderPID.setOutputRange(ShooterConstants.SHOULDER_MIN_CONTROL_EFFORT,
        ShooterConstants.SHOULDER_MAX_CONTROL_EFFORT);

    bottomLimitSwitch = new DigitalInput(ShooterConstants.LIMIT_SWITCH_ID);

    shoulderProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ShooterConstants.MAX_SPEED,
            ShooterConstants.MAX_ACCELERATION));
    armGoal = ShooterConstants.ARM_LOWER_LIMIT;
    profileStart = new TrapezoidProfile.State(ShooterConstants.ARM_LOWER_LIMIT.getRadians(), 0);

    shoulderFeedforward = new ArmFeedforward(
        ShooterConstants.SHOULDER_KS,
        ShooterConstants.SHOULDER_KG,
        ShooterConstants.SHOULDER_KV,
        ShooterConstants.SHOULDER_KA);

    flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.FLYWHEEL_KS,
        ShooterConstants.FLYWHEEL_KV,
        ShooterConstants.FLYWHEEL_KA);

    noteSensor = new DigitalInput(ShooterConstants.NOTE_SENSOR_ID);

    portShooter.burnFlash();
    starboardShooter.burnFlash();
    loader.burnFlash();
    shoulder.burnFlash();

    timer = new Timer();
    timer.start();

    shoulderCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              shoulder.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("shoulder")
                  .voltage(Volts.of(shoulder.getAppliedOutput() * shoulder.getBusVoltage()))
                  .angularPosition(Radians.of(shoulderEncoder.getPosition()))
                  .angularVelocity(RadiansPerSecond.of(shoulderEncoder.getVelocity()));
            },
            this));

    portShootCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              portShooter.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("portShooter")
                  .voltage(Volts.of(portShooter.getAppliedOutput() * portShooter.getBusVoltage()))
                  .angularPosition(Rotations.of(portShooterEncoder.getPosition()))
                  .angularVelocity(RotationsPerSecond.of(portShooterEncoder.getVelocity()));
            },
            this));

    starboardShootCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              starboardShooter.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("starboardShooter")
                  .voltage(Volts.of(starboardShooter.getAppliedOutput() * starboardShooter.getBusVoltage()))
                  .angularPosition(Rotations.of(starboardShooterEncoder.getPosition()))
                  .angularVelocity(RotationsPerSecond.of(starboardShooterEncoder.getVelocity()));
            },
            this));
  }

  public void runLoader(double speed) {
    loader.set(speed);
  }

  public void setShooterSpeed(double portRPM, double starboardRPM) {
    portShooterPID.setReference(portRPM, ControlType.kVelocity, 0, flywheelFeedforward.calculate(portRPM),
        ArbFFUnits.kVoltage);
    starboardShooterPID.setReference(starboardRPM, ControlType.kVelocity, 0,
        flywheelFeedforward.calculate(starboardRPM), ArbFFUnits.kVoltage);
  }

  public void setPortShooterRaw(double speed) {
    portShooter.set(speed);
  }

  public void setStarboardShooterRaw(double speed) {
    starboardShooter.set(speed);
  }

  public void setArmAngle(Rotation2d angle) {
    armGoal = angle;
    profileStart = new TrapezoidProfile.State(shoulderEncoder.getPosition(), shoulderEncoder.getVelocity());
    timer.stop();
    timer.reset();
    timer.start();
  }

  public void setShoulderVolts(double volts) {
    shoulder.setVoltage(volts);
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromRadians(shoulderEncoder.getPosition());
  }

  public double getPortShooterSpeed() {
    return portShooterEncoder.getVelocity();
  }

  public double getStarboardShooterSpeed() {
    return starboardShooterEncoder.getVelocity();
  }

  public boolean getNoteSensor() {
    return noteSensor.get();
  }

  public boolean armAtGoal() {
    return Math.abs(armGoal.getRadians() - shoulderEncoder.getPosition()) <= ShooterConstants.ARM_TOLERANCE;
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.get()) {
      shoulderEncoder.setPosition(ShooterConstants.ARM_LOWER_LIMIT.getRadians());
    }
    if (armGoal.getRadians() >= ShooterConstants.ARM_UPPER_LIMIT.getRadians()) {
      armGoal = ShooterConstants.ARM_UPPER_LIMIT;
    }
    armSetpoint = shoulderProfile.calculate(
        timer.get(),
        profileStart,
        new TrapezoidProfile.State(armGoal.getRadians(), 0));
    // This really shouldn't be necessary
    armSetpoint = (armSetpoint.position >= ShooterConstants.ARM_UPPER_LIMIT.getRadians())
        ? new TrapezoidProfile.State(ShooterConstants.ARM_UPPER_LIMIT.getRadians(), 0)
        : armSetpoint;

    if (Math
        .abs(armSetpoint.position - ShooterConstants.ARM_LOWER_LIMIT.getRadians()) <= ShooterConstants.ARM_DEADBAND) {
      shoulder.set(0);
    } else {
      shoulderPID.setReference(
          armSetpoint.position,
          ControlType.kPosition,
          0,
          shoulderFeedforward.calculate(armSetpoint.position, armSetpoint.velocity),
          ArbFFUnits.kVoltage);
    }

    SmartDashboard.putNumber("ProportionalTerm",
        ShooterConstants.SHOULDER_KP * (armSetpoint.position - shoulderEncoder.getPosition()));
    SmartDashboard.putNumber("FeedforwardValue",
        shoulderFeedforward.calculate(armSetpoint.position, armSetpoint.velocity));
    SmartDashboard.putBoolean("LimitSwitch", bottomLimitSwitch.get());
    SmartDashboard.putNumber("ShooterShoulderGoal", armGoal.getDegrees());
    SmartDashboard.putNumber("ShooterShoulderSetpoint", Math.toDegrees(armSetpoint.position));
    SmartDashboard.putNumber("ShooterShoulderSetpointVel", Math.toDegrees(armSetpoint.velocity));
    SmartDashboard.putNumber("ShooterShoulderPos", Math.toDegrees(shoulderEncoder.getPosition()));
    SmartDashboard.putNumber("ShoulderControlEffort", shoulder.getAppliedOutput() * shoulder.getBusVoltage());
    SmartDashboard.putNumber("PortVolts", portShooter.getAppliedOutput() * portShooter.getBusVoltage());
    SmartDashboard.putNumber("StarboardVolts", starboardShooter.getAppliedOutput() * starboardShooter.getBusVoltage());
    SmartDashboard.putNumber("PortRPM", portShooterEncoder.getVelocity());
    SmartDashboard.putNumber("StarboardRPM", starboardShooterEncoder.getVelocity());
  }

  public Command sysIdQuasiShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.quasistatic(direction);
  }

  public Command sysIdQuasiPortShooter(SysIdRoutine.Direction direction) {
    return portShootCharacterizer.quasistatic(direction);
  }

  public Command sysIdQuasiStarShooter(SysIdRoutine.Direction direction) {
    return starboardShootCharacterizer.quasistatic(direction);
  }

  public Command sysIdDynShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.dynamic(direction);
  }

  public Command sysIdDynPortShooter(SysIdRoutine.Direction direction) {
    return portShootCharacterizer.dynamic(direction);
  }

  public Command sysIdDynStarShooter(SysIdRoutine.Direction direction) {
    return starboardShootCharacterizer.dynamic(direction);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
