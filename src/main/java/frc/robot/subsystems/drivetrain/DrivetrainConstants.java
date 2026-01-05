package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleConfig;

public final class DrivetrainConstants {
    public static final double kDriveMotorGearReduction = 6.12;
    public static final double kAzimuthMotorGearReduction = 12.6;
    public static final Distance kDriveWheelRadius = Inches.of(2.0);
    
    /** A conversion factor to convert from drive motor rotations per second to wheel meters per second. */
    public static final double kDriveMotorToWheelFactor =
        (2.0 * Math.PI * kDriveWheelRadius.in(Meters)) / kDriveMotorGearReduction;

    public static final Distance kDriveMotorConversion = kDriveWheelRadius
        .times(2.0 * Math.PI).div(kDriveMotorGearReduction);

    public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(4.0);
    public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(0.5);
    public static final LinearVelocity kMaxAttainableLinearSpeed = MetersPerSecond.of(4.0);
    public static final AngularVelocity kMaxWheelAzimuthSpeed = RotationsPerSecond.of(10.0);
    public static final LinearAcceleration kMaxLinearAcceleration = MetersPerSecondPerSecond.of(4.0);

    public static final SwerveModuleConfig kFLSwerveModuleConfiguration =
        new SwerveModuleConfig(1, 2, 3, new Translation2d(0.276, 0.276));

    public static final SwerveModuleConfig kFRSwerveModuleConfiguration =
        new SwerveModuleConfig(4, 5, 6, new Translation2d(0.276, -0.276));

    public static final SwerveModuleConfig kBLSwerveModuleConfiguration =
        new SwerveModuleConfig(7, 8, 9, new Translation2d(-0.276, 0.276));

    public static final SwerveModuleConfig kBRSwerveModuleConfiguration =
        new SwerveModuleConfig(10, 11, 12, new Translation2d(-0.276, -0.276));

    public static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        ).withSlot0(new Slot0Configs()
            .withKS(0.020288)
            .withKV(0.13505)
            .withKA(0.013456)
            .withKP(0.034721)
        ).withMotionMagic(new MotionMagicConfigs()  
            .withMotionMagicAcceleration(9999.0)
        );

    public static final TalonFXConfiguration kAzimuthMotorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withContinuousWrap(true)
        ).withSlot0(new Slot0Configs()
            .withKS(0.018053)
            //.withKP(9.6245)
            .withKP(20.0)
            .withKD(0.14147)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(9999.0)
            .withMotionMagicCruiseVelocity(9999.0)
        );

    /**************************************************
     * Drivetrain Simulation
    **************************************************/
    @SuppressWarnings("unchecked")
    public static final DriveTrainSimulationConfig kDrivetrainSimulationConfiguration = new DriveTrainSimulationConfig(
        Pounds.of(135.0), // Robot Mass
        Meters.of(0.965), // Bumper X Length
        Meters.of(0.855), // Bumper Y Length

        Meters.of(0.5), // Drivebase X Distance
        Meters.of(0.5), // Drivebase Y Distance

        COTS.ofPigeon2(),
        COTS.ofMark4(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
            3 // MK4 Gear Ratio Level
        )
    );
}
