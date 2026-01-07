// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;
import static frc.robot.RobotConstants.Drivetrain.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotConstants.Drivetrain;
import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandXboxController m_programmerController = new CommandXboxController(5);

    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final DrivetrainSubsystem m_drivetrain;
    private final VisionSubsystem m_vision;

    public RobotContainer() {
        switch (RobotConstants.BEHAVIOR) {
            case REAL -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(1),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                // TODO: Make these in the robot constants;
                // subsystem constants should be configurations / constants that
                // work on any robot, robot configuration ones should be robot-specific,
                // sub-classes for each subsystem
                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOReal("FrontCAM", new Transform3d(
                        new Translation3d(0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, 0.0)
                    )),

                    new VisionIOReal("BackCAM", new Transform3d(
                        new Translation3d(-0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, Math.PI)
                    ))
                );
            }

            case SIMULATION -> {
                var drivetrainSimulation = new SwerveDriveSimulation(
                    DRIVETRAIN_SIMULATION_CONFIG, DRIVETRAIN_SIMULATION_START_POSE
                );

                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOSim(drivetrainSimulation.getGyroSimulation(), 1),
                    new SwerveModuleIOSim(kModuleConfigurations[0], drivetrainSimulation.getModules()[0]),
                    new SwerveModuleIOSim(kModuleConfigurations[1], drivetrainSimulation.getModules()[1]),
                    new SwerveModuleIOSim(kModuleConfigurations[2], drivetrainSimulation.getModules()[2]),
                    new SwerveModuleIOSim(kModuleConfigurations[3], drivetrainSimulation.getModules()[3]),
                    drivetrainSimulation
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOSim(
                        "PERFECTCAM",
                        new Transform3d(),
                        new SimCameraProperties(),
                        m_drivetrain::getSimulationPose
                    )
                );

                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(m_drivetrain, m_vision);

        configureBindings();
        configureAutoChooser();
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));
        m_driverController.a().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
        m_driverController.b().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
    }

    private final void configureAutoChooser() {
        m_autoChooser.addDefaultOption("None", new InstantCommand());

        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(key -> {
            m_autoChooserAlert.set(key.equals("None"));
        });
    }

    public final Command getAutonomousCommand() {
        return m_autoChooser.get();
    }
}
