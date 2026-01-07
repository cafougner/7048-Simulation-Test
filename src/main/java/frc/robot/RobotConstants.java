package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleConfig;

public final class RobotConstants {
    public static final RobotBehavior BEHAVIOR =
        Robot.isReal() ? RobotBehavior.REAL : RobotBehavior.SIMULATION;

    public static enum RobotBehavior {
        REAL,
        SIMULATION
    }

    public static final class Drivetrain {
        @SuppressWarnings("unchecked") // TODO: Fix warning (type safety).
        public static final DriveTrainSimulationConfig DRIVETRAIN_SIMULATION_CONFIG = new DriveTrainSimulationConfig(
            Pounds.of(135.0), // Robot Mass
            Meters.of(0.965), // Bumper X
            Meters.of(0.855), // Bumper Y
            Meters.of(kModuleConfigurations[0].moduleTranslation().getX() * 2.0), // Drivebase X
            Meters.of(kModuleConfigurations[0].moduleTranslation().getY() * 2.0), // Drivebase Y

            COTS.ofPigeon2(),
            COTS.ofMark4(
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                3 // Gear Ratio Level
            )
        );

        public static final Pose2d DRIVETRAIN_SIMULATION_START_POSE = new Pose2d(7.0, 2.75, Rotation2d.fromDegrees(120.0));
    }
}
