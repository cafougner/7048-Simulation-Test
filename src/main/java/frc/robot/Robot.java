// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.RobotConstants.RobotBehavior;
import frc.robot.utils.RobotVisualizer;

public class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;

    private Command m_autonomousCommand;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changed commited.";
            case 1 -> "Uncommited changes.";
            default -> "Unknown.";
        });

        switch (RobotConstants.ROBOT_BEHAVIOR) {
            case REAL -> {
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter());
            }

            case SIMULATION -> {
                Logger.addDataReceiver(new NT4Publisher());
            }
        }

        Logger.start();
        SignalLogger.enableAutoLogging(false);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        RobotVisualizer.updateComponents();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) m_autonomousCommand.schedule();

        if (RobotConstants.ROBOT_BEHAVIOR == RobotBehavior.SIMULATION) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public final void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public final void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput(
            "SimulatedArena/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        );

        Logger.recordOutput(
            "SimulatedArena/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        );
    }
}
