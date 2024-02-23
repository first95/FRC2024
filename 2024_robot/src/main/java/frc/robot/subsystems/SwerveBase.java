// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PoseLatency;
import frc.lib.util.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Vision;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveBase extends SubsystemBase {

  private final SwerveModule[] swerveModules;
  private Pigeon2 imu;
  private NetworkTable bowLimelightData, sternLimelightData;
  
  public Field2d field = new Field2d();
  private Field2d sternCam, bowCam;

  private double angle, lasttime;

  private Timer timer;

  private boolean wasGyroReset, wasOdometrySeeded;

  private final SwerveDrivePoseEstimator odometry;

  private Alliance alliance = null;

  private final SysIdRoutine driveCharacterizer, angleCharacterizer;

  /** Creates a new swerve drivebase subsystem.  Robot is controlled via the drive() method,
   * or via the setModuleStates() method.  The drive() method incorporates kinematics— it takes a 
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * setModuleStates() takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsytem also handles odometry.
  */
  public SwerveBase() {

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
    } else {
      imu = new Pigeon2(Drivebase.PIGEON);
      Pigeon2Configurator configurator = imu.getConfigurator();
      Pigeon2Configuration config = new Pigeon2Configuration();
      config.MountPose.MountPosePitch = Drivebase.IMU_MOUNT_PITCH;
      config.MountPose.MountPoseRoll = Drivebase.IMU_MOUNT_ROLL;
      config.MountPose.MountPoseYaw = Drivebase.IMU_MOUNT_YAW;
      configurator.apply(config);
    }

    this.swerveModules = new SwerveModule[] {
      new SwerveModule(Drivebase.Mod0.CONSTANTS),
      new SwerveModule(Drivebase.Mod1.CONSTANTS),
      new SwerveModule(Drivebase.Mod2.CONSTANTS),
      new SwerveModule(Drivebase.Mod3.CONSTANTS),
    };
    
    odometry = new SwerveDrivePoseEstimator(
      Drivebase.KINEMATICS,
      getYaw(),
      getModulePositions(),
      new Pose2d(),
      VecBuilder.fill(Vision.ODOMETRY_TRANSLATIONAL_STD_DEV, Vision.ODOMETRY_TRANSLATIONAL_STD_DEV, Vision.ODOMETRY_ANGULAR_STD_DEV),
      VecBuilder.fill(Vision.VISION_FAR_TRANSLATIONAL_STD_DEV, Vision.VISION_FAR_TRANSLATIONAL_STD_DEV, Vision.VISION_FAR_ANGULAR_STD_DEV));
    wasOdometrySeeded = false;
    wasGyroReset = false;

    sternCam = new Field2d();
    bowCam = new Field2d();
    SmartDashboard.putData("SternCam", sternCam);
    SmartDashboard.putData("BowCam", bowCam);
    SmartDashboard.putData("Field", field);

    driveCharacterizer = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          for (SwerveModule module : this.swerveModules) {
            module.setAzimuth(new Rotation2d());
            module.setDriveVolts(volts.in(Volts));
          }
        },
        log -> {
          log.motor("driveLinear")
          .voltage(Volts.of(avgDriveVolts()))
          .linearPosition(Meters.of(odometry.getEstimatedPosition().getX()))
          .linearVelocity(MetersPerSecond.of(getRobotVelocity().vxMetersPerSecond));
        },
        this));
    
    angleCharacterizer = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          for (SwerveModule module : this.swerveModules) {
            module.setAzimuth(module.getModuleLocation().getAngle().plus(Rotation2d.fromDegrees(90)));
            module.setDriveVolts(volts.in(Volts));
          }
        },
        log -> {
          log.motor("driveAngular")
          .voltage(Volts.of(avgDriveVolts()))
          .angularPosition(Radians.of(getPose().getRotation().getRadians()))
          .angularVelocity(RadiansPerSecond.of(getRobotVelocity().omegaRadiansPerSecond));
        },
        this));
  }

  /**
   * The primary method for controlling the drivebase.  Takes a Translation2d and a rotation rate, and 
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity
   * control for the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation
   *  vector is used.
   * @param translation  Translation2d that is the commanded linear velocity of the robot, in meters per second.
   * In robot-relative mode, positive x is torwards the bow (front) and positive y is torwards port (left).  In field-
   * relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking 
   * through the driver station glass (field West).
   * @param rotation  Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative  Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop  Whether or not to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    SmartDashboard.putString("InputCommands", translation.toString() + ", Omega: " + rotation);
    
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      correctForDynamics(
        new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation),
        Constants.LOOP_CYCLE,
        Drivebase.SKEW_CORRECTION_FACTOR),
      getPose().getRotation()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );

    //velocity = correctForDynamics(velocity, Constants.LOOP_CYCLE, 1);

    // Display commanded speed for testing
    SmartDashboard.putString("RobotVelocity", velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    
    // Desaturate calculated speeds
    //SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Speed Setpoint: ", swerveModuleStates[module.moduleNumber].speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle Setpoint: ", swerveModuleStates[module.moduleNumber].angle.getDegrees());
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  /**
   * Set robot-relative chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Robot-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      Drivebase.KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public Pose2d getTelePose() {
    if (alliance == Alliance.Red) {
      var odomPose = odometry.getEstimatedPosition();
      return new Pose2d(odomPose.getTranslation(), odomPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    } else {
      return odometry.getEstimatedPosition();
    }
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getPose().getRotation());
  }

  public ChassisSpeeds getTeleFieldVelocity() {
    if (alliance == Alliance.Red) {
      var speeds = getFieldVelocity();
      return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
    } else {
      return getFieldVelocity();
    }
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return Drivebase.KINEMATICS.toChassisSpeeds(getStates());
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions cointaing the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Drivebase.NUM_MODULES];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  public void clearGyroReset() {
    wasGyroReset = false;
  }



  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  private Rotation2d getYaw() {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (Robot.isReal()) {
      double yaw = imu.getYaw().getValueAsDouble();
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    } else {
      return new Rotation2d(angle);
    }
  }

  public Rotation2d getPitch() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
    } else {
      return new Rotation2d();
    }
  }

  public Rotation2d getRoll() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
    } else {
      return new Rotation2d();
    }
  }

  public Command sysIdQuasiLinear(SysIdRoutine.Direction direction) {
    return driveCharacterizer.quasistatic(direction);
  }
  public Command sysIdQuasiAngular(SysIdRoutine.Direction direction) {
    return angleCharacterizer.quasistatic(direction);
  }
  public Command sysIdDynLinear(SysIdRoutine.Direction direction) {
    return driveCharacterizer.dynamic(direction);
  }
  public Command sysIdDynAngular(SysIdRoutine.Direction direction) {
    return angleCharacterizer.dynamic(direction);
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new SwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle()),
        true,
        false);
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public void clearOdometrySeed() {
    wasOdometrySeeded = false;
  }

  public void setVelocityModuleGains() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setGains(
        SmartDashboard.getNumber("KP", Drivebase.VELOCITY_KP),
        SmartDashboard.getNumber("KI", Drivebase.VELOCITY_KI),
        SmartDashboard.getNumber("KD", Drivebase.VELOCITY_KD),
        SmartDashboard.getNumber("KS", Drivebase.KS),
        SmartDashboard.getNumber("KV", Drivebase.KV),
        SmartDashboard.getNumber("KA", Drivebase.KA));
    }
  }

  public PoseLatency getVisionPose(String limelightName) {
    NetworkTable visionData = NetworkTableInstance.getDefault().getTable(limelightName);
    if ((visionData.getEntry("tv").getDouble(0) == 0 ||
      visionData.getEntry("getPipe").getDouble(0) != Vision.APRILTAG_PIPELINE_NUMBER)) {
      return null;
    }
    double[] poseComponents;
    if (alliance != null) {
      poseComponents = visionData.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    } else {
      return null;
    }
    return new PoseLatency(poseComponents);
  }
  
  private void addVisionMeasurement(String limelightName, Pose2d estimatedPose) {
    PoseLatency visionMeasurement = getVisionPose(limelightName);
    if (visionMeasurement == null) {
      return;
    }
    double targetArea = LimelightHelpers.getTA(limelightName);
    LimelightResults llResults = LimelightHelpers.getLatestResults(limelightName);
    int numTargets = llResults.targetingResults.targets_Fiducials.length;

    //double poseDifference = estimatedPose.getTranslation().getDistance(visionMeasurement.pose2d.getTranslation());
    double xyStds, degStds;

    if (Math.abs(visionMeasurement.pose3d.getZ()) >= Vision.MAX_ALLOWABLE_Z_ERROR) {
      return;
    }
    
    if (numTargets >= 2) {
      if (targetArea > Vision.MIN_CLOSE_MULTITARGET_AREA) {
        xyStds = Vision.VISION_CLOSE_MULTITARGET_TRANSLATIONAL_STD_DEV;
        degStds = Vision.VISION_CLOSE_MULTITARGET_ANGULAR_STD_DEV;
      } else if (targetArea > Vision.MIN_FAR_MULTITARGET_AREA) {
        xyStds = Vision.VISION_FAR_MULTITARGET_TRANSLATIONAL_STD_DEV;
        degStds = Vision.VISION_FAR_MULTITARGET_ANGULAR_STD_DEV;
      } else {
        return;
      }
    } else {
      if (targetArea > Vision.MIN_CLOSE_TARGET_AREA) {
        xyStds = Vision.VISION_CLOSE_TRANSLATIONAL_STD_DEV;
        degStds = Vision.VISION_CLOSE_ANGULAR_STD_DEV;
      } else if (targetArea > Vision.MIN_FAR_TARGET_AREA) {
        xyStds = Vision.VISION_FAR_TRANSLATIONAL_STD_DEV;
        degStds = Vision.VISION_FAR_ANGULAR_STD_DEV;
      } else {
        return;
      }
    }
    double timestamp = Timer.getFPGATimestamp() - (visionMeasurement.latency / 1000);
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds)));
    odometry.addVisionMeasurement(visionMeasurement.pose2d, timestamp);
  }

  @Override
  public void periodic() {
    odometry.update(getYaw(), getModulePositions());

    SmartDashboard.putString("Gyro", getYaw().toString());
    SmartDashboard.putString("alliance", (alliance != null) ? alliance.toString() : "NULL");
    SmartDashboard.putString("OdometryPos", getPose().toString());
    /*ChassisSpeeds robotVelocity = getRobotVelocity();
    SmartDashboard.putNumber("Robot X Vel", robotVelocity.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Y Vel", robotVelocity.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Ang Vel", robotVelocity.omegaRadiansPerSecond);*/

    SmartDashboard.putBoolean("seeded", wasOdometrySeeded);
    // Seed odometry if this has not been done
    if (!wasOdometrySeeded) { 
      PoseLatency bowSeed = getVisionPose(Vision.BOW_LIMELIGHT_NAME);
      PoseLatency sternSeed = getVisionPose(Vision.STERN_LIMELIGHT_NAME);
      if ((bowSeed != null) && (sternSeed != null)) {
        // Average position, pick a camera for rotation
        Translation2d translation = bowSeed.pose2d.getTranslation().plus(sternSeed.pose2d.getTranslation()).div(2);
        //Rotation2d rotation = starboardSeed.getRotation();
        // Rotation average:
        Rotation2d rotation = 
          new Translation2d(
            (bowSeed.pose2d.getRotation().getCos() + sternSeed.pose2d.getRotation().getCos()) / 2,
            (bowSeed.pose2d.getRotation().getSin() + sternSeed.pose2d.getRotation().getSin()) / 2)
          .getAngle();
        resetOdometry(new Pose2d(translation, rotation));
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (sternSeed != null) {
        resetOdometry(sternSeed.pose2d);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (bowSeed != null) {
        resetOdometry(bowSeed.pose2d);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      } else {
        DriverStation.reportWarning("Alliance not set or tag not visible", false);
      }
    }

    Pose2d estimatedPose = getPose();
    
    addVisionMeasurement(Vision.BOW_LIMELIGHT_NAME, estimatedPose);
    addVisionMeasurement(Vision.STERN_LIMELIGHT_NAME, estimatedPose);

    field.setRobotPose(estimatedPose);
    PoseLatency bowCamPose = getVisionPose(Vision.BOW_LIMELIGHT_NAME);
    if (bowCamPose != null) {
      bowCam.setRobotPose(bowCamPose.pose2d);
      SmartDashboard.putNumber("BowCamZ", bowCamPose.pose3d.getZ());
    }
    PoseLatency sternCamPose = getVisionPose(Vision.STERN_LIMELIGHT_NAME);
    if (sternCamPose != null) {
      sternCam.setRobotPose(sternCamPose.pose2d);
      SmartDashboard.putNumber("SternCamZ", sternCamPose.pose3d.getZ());
    }

    /*ChassisSpeeds robotVelocity = getRobotVelocity();
    SmartDashboard.putNumber("Robot X Vel", robotVelocity.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Y Vel", robotVelocity.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Ang Vel", robotVelocity.omegaRadiansPerSecond);*/

    // Update angle accumulator if the robot is simulated
    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();
      
    }

    double[] moduleStates = new double[8];
    double[] moduleSetpoints = new double[8];
    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber("Module" + module.moduleNumber + "CANCoder", module.getAbsoluteEncoder());
      var state = module.getState();
      var desState = module.getDesiredState();
      moduleStates[module.moduleNumber] = state.angle.getRadians();
      moduleSetpoints[module.moduleNumber] = desState.angle.getRadians();
      moduleStates[module.moduleNumber + 1] = state.speedMetersPerSecond;
      moduleSetpoints[module.moduleNumber + 1] = desState.speedMetersPerSecond;
    }
    SmartDashboard.putNumberArray("moduleStates", moduleStates);
    SmartDashboard.putNumberArray("moduleSetpoints", moduleSetpoints);
    SmartDashboard.putNumber("OdomHeading", estimatedPose.getRotation().getRadians());
  }

  @Override
  public void simulationPeriodic() {
  }


  private ChassisSpeeds correctForDynamics(ChassisSpeeds initial, double dt, double magicFactor) {
    final double oneMinusCos = 1 - Math.cos(initial.omegaRadiansPerSecond * dt);
    if (Math.abs(oneMinusCos) < 1E-9) {
      return initial;
    } else {
      final var linearVel = new Translation2d(initial.vxMetersPerSecond, initial.vyMetersPerSecond);
      final double tangentVel = linearVel.getNorm();
      final double radius;
      radius = tangentVel / initial.omegaRadiansPerSecond;
      final double skewVelocity = (radius * oneMinusCos) / dt;
      var direction = linearVel.getAngle().minus(Rotation2d.fromDegrees(90));
      var velocityCorrection = new Translation2d(skewVelocity, direction).times(magicFactor);
      var translationVel = linearVel.plus(velocityCorrection);
      return new ChassisSpeeds(
        translationVel.getX(),
        translationVel.getY(),
        initial.omegaRadiansPerSecond
      );
    }
  }
  
  public void turnModules(double speed) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.turnModule(speed);
    }
  }

  private double avgDriveVolts() {
    double sum = 0;
    for (SwerveModule module : swerveModules) {
      sum += module.getDriveVolts();
    }
    return (sum / Drivebase.NUM_MODULES);
  }
}
