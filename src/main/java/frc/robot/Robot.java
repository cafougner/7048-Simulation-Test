// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.RobotConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.utils.RobotVisualizer;

public final class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;

    private Command m_autonomousCommand;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes commited.";
            case 1 -> "Uncommited changes.";
            default -> "Unknown.";
        });

        switch (kRobotBehavior) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case SIMULATION -> Logger.addDataReceiver(new NT4Publisher());

            case REPLAY -> {
                String logPath = LogFileUtil.findReplayLog();

                setUseTiming(false);
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
            }
        }

        Logger.start();
        SignalLogger.enableAutoLogging(false);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

        m_robotContainer = new RobotContainer();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        RobotVisualizer.update();
    }

    @Override
    public final void disabledInit() {}

    @Override
    public final void disabledPeriodic() {}

    @Override
    public final void disabledExit() {}

    @Override
    public final void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public final void autonomousPeriodic() {}

    @Override
    public final void autonomousExit() {}

    @Override
    public final void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public final void teleopPeriodic() {}

    @Override
    public final void teleopExit() {}

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public final void testPeriodic() {}

    @Override
    public final void testExit() {}

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
