// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.*;
// TODO: only static import from the files corresponding constants
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.DriveToPoseCommandV5;
import frc.robot.subsystems.blinkin.BlinkinSubsystem;
import frc.robot.subsystems.blinkin.io.BlinkinIOSim;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.io.ElevatorIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.FieldUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final VisionSubsystem m_vision;
    private final DrivetrainSubsystem m_drivetrain;
    private final ElevatorSubsystem m_elevator;
    private final BlinkinSubsystem m_blinkin;

    public RobotContainer() {
        switch (kRobotBehavior) {
            case REAL -> {
                throw new UnsupportedOperationException();
            }

            case SIMULATION -> {
                var drivetrainSimulation = new SwerveDriveSimulation(
                    kDrivetrainSimulationConfiguration, new Pose2d(7.0, 2.75, Rotation2d.fromDegrees(120.0))
                );

                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOSim(drivetrainSimulation.getGyroSimulation(), 1),
                    new SwerveModuleIOSim(drivetrainSimulation.getModules()[0], kFLSwerveModuleConfiguration),
                    new SwerveModuleIOSim(drivetrainSimulation.getModules()[1], kFRSwerveModuleConfiguration),
                    new SwerveModuleIOSim(drivetrainSimulation.getModules()[2], kBLSwerveModuleConfiguration),
                    new SwerveModuleIOSim(drivetrainSimulation.getModules()[3], kBRSwerveModuleConfiguration),
                    Optional.of(drivetrainSimulation)
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator()
                );

                m_elevator = new ElevatorSubsystem(new ElevatorIOSim(33));
                m_blinkin = new BlinkinSubsystem(new BlinkinIOSim());

                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            case REPLAY -> {
                throw new UnsupportedOperationException();
            }

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(m_vision, m_drivetrain, m_elevator, m_blinkin);

        configureBindings();
        configureAutoChooser();
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));

        m_driverController.leftTrigger().whileTrue(getAlignLeftCommand());
        m_driverController.rightTrigger().whileTrue(getAlignRightCommand());
        m_driverController.a().whileTrue(getAlignStationCommand());
        m_driverController.b().whileTrue(getWheelRadiusCommand());
    }

    private final void configureAutoChooser() {
        m_autoChooser.addDefaultOption("None", new InstantCommand());
        
        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(key -> {
            m_autoChooserAlert.set(key.equals("None"));
        });
    }

    private final Command getWheelRadiusCommand() {
        return Commands.sequence(
            Commands.runOnce(
                () -> m_drivetrain.drive(new ChassisSpeeds(0.0, 0.0, Math.PI), false, false),
                m_drivetrain
            ),
            
            Commands.waitSeconds(1),

            Commands.run(
                () -> {
                    var speeds = m_drivetrain.getFieldChassisSpeeds();
                    var wheels = m_drivetrain.getSwerveModuleStates();
                    double velocity = 0.0;
    
                    for (var state : wheels) {
                        velocity += Math.abs(state.speedMetersPerSecond);
                    }
    
                    velocity /= wheels.length;
    
                    Logger.recordOutput("WheelDiameterMeters", (speeds.omegaRadiansPerSecond * Math.hypot(0.25, 0.25)) / (velocity / kDriveMotorToWheelFactor));
                    System.out.println((speeds.omegaRadiansPerSecond * Math.hypot(0.25, 0.25)) / (velocity / kDriveMotorToWheelFactor));
                },
    
                m_drivetrain
            )
        );
    }

    private final Command getAlignLeftCommand() {
        return new DriveToPoseCommandV5(
            () -> m_drivetrain.getEstimatedPose(),
            () -> FieldUtils.getNearestReefAprilTagPose(m_drivetrain.getEstimatedPose())
                .transformBy(new Transform2d(0.4825, -0.15, Rotation2d.kPi)),
            () -> m_drivetrain.getFieldChassisSpeeds(),
            (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
            m_drivetrain
        );
    }

    private final Command getAlignRightCommand() {
        return new DriveToPoseCommandV5(
            () -> m_drivetrain.getEstimatedPose(),
            () -> FieldUtils.getNearestReefAprilTagPose(m_drivetrain.getEstimatedPose())
                .transformBy(new Transform2d(0.4825, 0.15, Rotation2d.kPi)),
            () -> m_drivetrain.getFieldChassisSpeeds(),
            (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
            m_drivetrain
        );
    }

    private final Command getAlignStationCommand() {
        return new DriveToPoseCommandV5(
            () -> m_drivetrain.getEstimatedPose(),
            () -> FieldUtils.getNearestStationAprilTagPose(m_drivetrain.getEstimatedPose())
                .transformBy(new Transform2d(0.4825, 0.0, Rotation2d.kPi)),
            () -> m_drivetrain.getFieldChassisSpeeds(),
            (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
            m_drivetrain
        );
    }

    // public final Command getAutonomousCommand() {
    //     return m_autoChooser.get();
    // }
    public final Command getAutonomousCommand() {
        Time[] times = new Time[2];
        times[0] = RobotController.getMeasureFPGATime();

        return Commands.sequence(
            new DriveToPoseCommandV5(
                m_drivetrain::getEstimatedPose,
                () -> new Pose2d(4.93, 2.75, Rotation2d.fromDegrees(120)),
                m_drivetrain::getFieldChassisSpeeds,
                (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
                m_drivetrain
            ),
            m_elevator.goToSetpoint(ElevatorSetpoints.L4, true),
            Commands.waitSeconds(0.25),
            m_elevator.goToSetpoint(ElevatorSetpoints.INTAKE, false),
            new DriveToPoseCommandV5(
                m_drivetrain::getEstimatedPose,
                () -> {
                    Pose2d goal = new Pose2d(1.60, 0.66, Rotation2d.fromDegrees(53.0));
                    double xDistance = m_drivetrain.getEstimatedPose().getY() - goal.getY();

                    return goal.transformBy(
                        new Transform2d(
                            0.0,
                            -MathUtil.clamp(Math.pow(xDistance, 2.0), 0.0, Double.MAX_VALUE) * 0.625,
                            Rotation2d.kZero
                        )
                    );
                },
                m_drivetrain::getFieldChassisSpeeds,
                (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
                m_drivetrain
            ),
            m_elevator.goToSetpoint(ElevatorSetpoints.INTAKE, true),
            Commands.waitSeconds(0.125),
            new DriveToPoseCommandV5(
                m_drivetrain::getEstimatedPose,
                () -> new Pose2d(4.02, 2.78, Rotation2d.fromDegrees(60)),
                m_drivetrain::getFieldChassisSpeeds,
                (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
                m_drivetrain
            ),
            m_elevator.goToSetpoint(ElevatorSetpoints.L4, true),
            Commands.waitSeconds(0.25),
            m_elevator.goToSetpoint(ElevatorSetpoints.INTAKE, false),
            new DriveToPoseCommandV5(
                m_drivetrain::getEstimatedPose,
                () -> new Pose2d(1.60, 0.66, Rotation2d.fromDegrees(53.0)),
                m_drivetrain::getFieldChassisSpeeds,
                (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
                m_drivetrain
            ),
            m_elevator.goToSetpoint(ElevatorSetpoints.INTAKE, true),
            Commands.waitSeconds(0.125),
            new DriveToPoseCommandV5(
                m_drivetrain::getEstimatedPose,
                () -> new Pose2d(3.71, 2.95, Rotation2d.fromDegrees(60)),
                m_drivetrain::getFieldChassisSpeeds,
                (desiredSpeeds) -> m_drivetrain.drive(desiredSpeeds, true, false),
                m_drivetrain
            ),
            m_elevator.goToSetpoint(ElevatorSetpoints.L4, true),
            Commands.waitSeconds(0.25),
            m_elevator.goToSetpoint(ElevatorSetpoints.INTAKE, false),
            Commands.runOnce(() -> {
                times[1] = RobotController.getMeasureFPGATime();
                System.out.println("Auto time: " + times[1].minus(times[0]).in(Seconds));
            })
        );
    }
}
