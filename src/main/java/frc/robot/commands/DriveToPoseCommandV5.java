package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class DriveToPoseCommandV5 extends Command {
    private final Supplier<Pose2d> m_currentPosition;
    private final Supplier<Pose2d> m_targetPosition;
    private final Supplier<ChassisSpeeds> m_currentSpeeds;
    private final Consumer<ChassisSpeeds> m_driveFieldRelative;

    private final LinearVelocity kMinimumCompensationVelocity = MetersPerSecond.of(0.125);

    private final PIDController m_translationPID = new PIDController(3.5, 0.0, 0.0);
    private final PIDController m_thetaPID = new PIDController(3.5, 0.0, 0.0);

    public DriveToPoseCommandV5(
        Supplier<Pose2d> currentPosition,
        Supplier<Pose2d> targetPosition,
        Supplier<ChassisSpeeds> currentSpeeds,
        Consumer<ChassisSpeeds> driveFieldRelative, 
        Subsystem... requirements
    ) {
        m_currentPosition = currentPosition;
        m_targetPosition = targetPosition;
        m_currentSpeeds = currentSpeeds;
        m_driveFieldRelative = driveFieldRelative;

        m_translationPID.setTolerance(0.035);
        m_thetaPID.setTolerance(Units.degreesToRadians(2.5));

        setName("DriveToPoseCommandV5");
        addRequirements(requirements);
    }

    @Override
    public final void initialize() {
        m_translationPID.reset();
        m_thetaPID.reset();

        m_translationPID.setSetpoint(0.0);
    }

    @Override
    public final void execute() {
        Pose2d currentPosition = m_currentPosition.get();
        Pose2d targetPosition = m_targetPosition.get();
        ChassisSpeeds currentSpeeds = m_currentSpeeds.get();

        double desiredLinearVelocity = m_translationPID.calculate(
            getDistance(currentPosition, targetPosition)
        );

        Rotation2d desiredAngle = getAdjustedAngle(currentPosition, targetPosition, currentSpeeds);

        m_driveFieldRelative.accept(new ChassisSpeeds(
            desiredLinearVelocity * desiredAngle.getCos(),
            desiredLinearVelocity * desiredAngle.getSin(),
            m_thetaPID.calculate(currentPosition.getRotation().getRadians(), targetPosition.getRotation().getRadians())
        ));
    }

    @Override
    public final void end(boolean isFinished) {
        m_driveFieldRelative.accept(new ChassisSpeeds());
    }

    @Override
    public final boolean isFinished() {
        return m_translationPID.atSetpoint() && m_thetaPID.atSetpoint();
    }

    public final DriveToPoseCommandV5 withContinuousRotation(double min, double max) {
        m_thetaPID.enableContinuousInput(min, max);
        return this;
    }

    private final double getDistance(Pose2d poseA, Pose2d poseB) {
        return poseA.getTranslation().getDistance(poseB.getTranslation());
    }

    private final Rotation2d getDirectAngle(Pose2d currentPose, Pose2d targetPose) {
        return targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    }

    private final Rotation2d getAdjustedAngle(
        Pose2d currentPose,
        Pose2d targetpose,
        ChassisSpeeds currentSpeeds
    ) {
        Rotation2d directAngle = getDirectAngle(currentPose, targetpose);
        Rotation2d velocityAngle = new Rotation2d(
            currentSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond
        );

        Rotation2d adjustedAngle = new Rotation2d(
            directAngle.getRadians() - Math.sin(velocityAngle.minus(directAngle).getRadians())
        );

        double adjustFactor = Math.hypot(
            currentSpeeds.vxMetersPerSecond,
            currentSpeeds.vyMetersPerSecond
        ) - kMinimumCompensationVelocity.in(MetersPerSecond);

        return directAngle.interpolate(adjustedAngle, Math.max(0.0, adjustFactor));
    }
}
