package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.BetterSwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final CANSparkMax angleMotor, driveMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final SparkPIDController angleController, driveController;
    private final Timer time;
    private final SwerveModuleConstants constants;
    private final PIDController currentController;
    private double angle, lastAngle, omega, speed, fakePos, lastTime, dt;
    private boolean currentControl;
    private ArrayList<Double> ampSamples = new ArrayList<Double>();

    private SimpleMotorFeedforward feedforward;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        angle = 0;
        speed = 0;
        fakePos = 0;
        constants = moduleConstants;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        // Config angle encoders
        absoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
        absoluteEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60);
        absoluteEncoder.setZeroOffset(angleOffset);
        absoluteEncoder.setInverted(Drivebase.ABSOLUTE_ENCODER_INVERT);
        absoluteEncoder.setAverageDepth(1);

        // Config angle motor/controller
        angleController = angleMotor.getPIDController();
        angleController.setP(Drivebase.MODULE_KP);
        angleController.setI(Drivebase.MODULE_KI);
        angleController.setD(Drivebase.MODULE_KD);
        angleController.setFF(Drivebase.MODULE_KF);
        angleController.setIZone(Drivebase.MODULE_IZ);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180);
        angleController.setPositionPIDWrappingMinInput(-180);
        angleController.setFeedbackDevice(absoluteEncoder);
        angleMotor.setInverted(Drivebase.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setSmartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);

        // Config drive motor/controller
        driveController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
        driveEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60);
        driveEncoder.setAverageDepth(1);
        driveController.setP(Drivebase.VELOCITY_KP);
        driveController.setI(Drivebase.VELOCITY_KI);
        driveController.setD(Drivebase.VELOCITY_KD);
        driveController.setFF(Drivebase.VELOCITY_KF);
        driveController.setIZone(Drivebase.VELOCITY_IZ);
        driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);

        driveMotor.burnFlash();
        angleMotor.burnFlash();

        feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

        time = new Timer();
        time.start();
        lastTime = time.get();
        
        lastAngle = getState().angle.getDegrees();

        currentController = new PIDController(Drivebase.CURRENT_KP, Drivebase.CURRENT_KI, Drivebase.CURRENT_KD);
        currentControl = false;
    }

    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, true);
    }

    /*public void setGains(double kp, double ki, double kd, double ks, double kv, double ka) {
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        driveController.setP(kp);
        driveController.setI(ki);
        driveController.setD(kd);
    }*/
    
    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop, boolean antijitter) {
        currentControl = false;
        SwerveModuleState simpleState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
        desiredState = new BetterSwerveModuleState(simpleState.speedMetersPerSecond, simpleState.angle, desiredState.omegaRadPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint: ", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Drivebase.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Drivebase.MAX_SPEED * 0.01)) && antijitter ? 
            lastAngle :
            desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition, 0, Math.toDegrees(desiredState.omegaRadPerSecond) * Drivebase.MODULE_KV);
        lastAngle = angle;

        this.angle = desiredState.angle.getDegrees();
        omega = desiredState.omegaRadPerSecond;
        speed = desiredState.speedMetersPerSecond;

        if (!Robot.isReal()) {
            dt = time.get() - lastTime;
            fakePos += (speed * dt);
            lastTime = time.get();
        }
    }

    public BetterSwerveModuleState getState() {
        double velocity;
        Rotation2d azimuth;
        double omega;
        if (Robot.isReal()) {
            velocity = driveEncoder.getVelocity();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
            omega = absoluteEncoder.getVelocity();
        } else {
            velocity = speed;
            azimuth = Rotation2d.fromDegrees(this.angle);
            omega = this.omega;
        }
        return new BetterSwerveModuleState(velocity, azimuth, omega);
    }

    public SwerveModulePosition getPosition() {
        double position;
        Rotation2d azimuth;
        if (Robot.isReal()) {
            position = driveEncoder.getPosition();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        } else {
            position = fakePos;
            azimuth = Rotation2d.fromDegrees(angle + (Math.toDegrees(omega) * dt));
        }
        SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
        return new SwerveModulePosition(position, azimuth);
    }

    public double getAbsoluteEncoder() {
        return absoluteEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake) {
        driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    public void turnModule(double speed) {
        currentControl = false;
        angleMotor.set(speed);
        SmartDashboard.putNumber("AbsoluteEncoder" + moduleNumber, absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("ControlEffort" + moduleNumber, angleMotor.getAppliedOutput());
    }

    public void setDriveCurrent(double amps) {
        currentControl = true;
        currentController.setSetpoint(amps);
    }

    public void setRawAngle(Rotation2d angle) {
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    }

    public void periodic() {
        double currentAmps = driveMotor.getOutputCurrent();
        SmartDashboard.putNumber("Module " + moduleNumber + " Volts:", (driveMotor.getAppliedOutput() * driveMotor.getBusVoltage()));
        SmartDashboard.putNumber("Module " + moduleNumber + " Amps", currentAmps);
        if (ampSamples.size() == Drivebase.CURRENT_MOVING_AVERAGE_SAMPLES) {
            ampSamples.remove(0);
        }
        ampSamples.add(currentAmps);
        double sum = 0;
        for (double value : ampSamples) {
            sum += value;
        }
        double average = sum / Drivebase.CURRENT_MOVING_AVERAGE_SAMPLES;
        SmartDashboard.putNumber("Module " + moduleNumber + " Avg. Amps", average);
        if (currentControl && DriverStation.isEnabled()) {
            double controlEffort = currentController.calculate(average);
            driveMotor.set(controlEffort);
            SmartDashboard.putNumber("Module " + moduleNumber + " ControlEffort", controlEffort);
        }
    }

    public double getAppliedDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public Translation2d getPositionFromCenter() {
        return new Translation2d(
            constants.xPos,
            constants.yPos
        );
    }
}
