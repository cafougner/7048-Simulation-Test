package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModule;

public final class TuningCommands {
    public static final Command getWheelRadiusCommand(DrivetrainSubsystem drivetrain) {
        SwerveModule[] swerveModules = drivetrain.getSwerveModules();

        return Commands.sequence(
            Commands.runOnce(
                // this does not rotate 180deg/s (~120), could be an issue with the drivetrain?
                () -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, Math.PI), false, false),
                drivetrain
            ),

            Commands.run(
                () -> {
                    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();
                    double driveMotorVelocities = 0.0;

                    for (SwerveModule module : swerveModules) {
                        driveMotorVelocities += module.getInputs().driveMotorVelocity.abs(RadiansPerSecond);
                    }

                    // temp
                    Logger.recordOutput("WheelDiameterMeters", (currentSpeeds.omegaRadiansPerSecond * 12.6 * Math.hypot(0.276, 0.276) / (driveMotorVelocities / 4.0) / 2.0));
                },
                drivetrain
            )
        );
    }
}
