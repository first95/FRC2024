// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drivebase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Autos;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.NoteHandlerCommand;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveBase;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Intake intake = new Intake();
    private final SwerveBase drivebase = new SwerveBase();
    private final Shooter shooter = new Shooter();
    private final TeleopDrive openRobotRel, closedRobotRel, openFieldRel, closedFieldRel;
    private final AbsoluteDrive absoluteDrive;

    private final CommandJoystick driveController = new CommandJoystick(OperatorConstants.driveControllerPort);
    private final CommandJoystick headingController = new CommandJoystick(OperatorConstants.headingControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OperatorConstants.operatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        openRobotRel = new TeleopDrive(
                drivebase,
                () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getY()
                        : 0,
                () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getX()
                        : 0,
                () -> -headingController.getTwist(), () -> false, true);

        closedRobotRel = new TeleopDrive(
                drivebase,
                () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getY()
                        : 0,
                () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getX()
                        : 0,
                () -> -headingController.getTwist(), () -> false, false);

        openFieldRel = new TeleopDrive(
                drivebase,
                () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
                        ? driveController.getY()
                        : 0,
                () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
                        ? driveController.getX()
                        : 0,
                () -> -headingController.getTwist(), () -> true, true);

        closedFieldRel = new TeleopDrive(
                drivebase,
                () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getY()
                        : 0,
                () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getX()
                        : 0,
                () -> -headingController.getTwist(), () -> true, false);

        absoluteDrive = new AbsoluteDrive(
                drivebase,
                () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getY()
                        : 0,
                () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
                        ? -driveController.getX()
                        : 0,
                () -> -headingController.getX(),
                () -> -headingController.getY(), false);

        NoteHandlerCommand noteManager = new NoteHandlerCommand(
                shooter,
                intake,
                () -> (operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis()) * 0.7,
                () -> operatorController.y().getAsBoolean());

        shooter.setDefaultCommand(noteManager);

        drivebase.setDefaultCommand(absoluteDrive);
        // Configure the trigger bindings
        configureBindings();
        SmartDashboard.putData("setGains", new InstantCommand(drivebase::setVelocityModuleGains));

        SmartDashboard.putNumber("KV", Drivebase.KV);
        SmartDashboard.putNumber("KA", Drivebase.KA);
        SmartDashboard.putNumber("KP", 0);
        SmartDashboard.putNumber("KI", 0);
        SmartDashboard.putNumber("KD", 0);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        /*
         * operatorController.y().whileTrue(shooter.sysIdQuasiShoulder(SysIdRoutine.
         * Direction.kForward));
         * operatorController.b().whileTrue(shooter.sysIdQuasiShoulder(SysIdRoutine.
         * Direction.kReverse));
         * operatorController.a().whileTrue(shooter.sysIdDynShoulder(SysIdRoutine.
         * Direction.kForward));
         * operatorController.x().whileTrue(shooter.sysIdDynShoulder(SysIdRoutine.
         * Direction.kReverse));
         */
        driveController.button(1).whileTrue(new AutoShoot(drivebase));
        driveController.button(8).onTrue(new InstantCommand(drivebase::clearOdometrySeed).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;// Autos.exampleAuto(m_exampleSubsystem);
    }

    public void sendAlliance() {
        drivebase.setAlliance(DriverStation.getAlliance().get());
    }

    private Map<String, ChoreoTrajectory> loadTrajectories() {
        Set<String> trajNames;
        try {
            trajNames = listFilesUsingFilesList("/deploy/choreo");
        } catch (IOException e) {
            DriverStation.reportError("Invalid Directory! Trajectories failed to load!", true);
            return null;
        }
        return trajNames.stream().collect(Collectors.toMap(
            entry -> entry,
            entry -> Choreo.getTrajectory(entry.split("\\.")[0])
        ));
    }

    public Set<String> listFilesUsingFilesList(String dir) throws IOException {
        try (Stream<Path> stream = Files.list(Paths.get(dir))) {
            return stream
                    .filter(file -> !Files.isDirectory(file))
                    .map(Path::getFileName)
                    .map(Path::toString)
                    .collect(Collectors.toSet());
        }
    }
}
