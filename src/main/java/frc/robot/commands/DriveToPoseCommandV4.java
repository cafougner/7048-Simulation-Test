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

public final class DriveToPoseCommandV4 extends Command {
    private final Supplier<Pose2d> m_currentPosition;
    private final Supplier<Pose2d> m_targetPosition;
    private final Supplier<ChassisSpeeds> m_currentSpeeds;
    private final Consumer<ChassisSpeeds> m_driveFieldRelative;

    private final LinearVelocity kMinimumCompensationVelocity = MetersPerSecond.of(0.5);

    private final PIDController m_translationPID = new PIDController(3.5, 0.0, 0.0);
    private final PIDController m_thetaPID = new PIDController(4.0, 0.0, 0.0);

    public DriveToPoseCommandV4(
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

        m_translationPID.setTolerance(0.025);

        m_thetaPID.setTolerance(Units.degreesToRadians(1.25));
        m_thetaPID.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);

        setName("DriveToPoseCommandV4");
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

        m_thetaPID.setSetpoint(targetPosition.getRotation().getRadians());

        double desiredLinearVelocity = m_translationPID.calculate(
            getDistance(currentPosition, targetPosition)
        );

        Rotation2d desiredAngle = getAdjustedAngle(currentPosition, targetPosition, currentSpeeds);

        // TODO: Is there a way to fix rotation getting "ignored" when the translation velocity is high?
        // Is there a way to make it so the rotation target is reached say tolerance meters before the end?
        // How do i split up different requirements like tolerance and continuous input range?
        m_driveFieldRelative.accept(new ChassisSpeeds(
            desiredLinearVelocity * desiredAngle.getCos(),
            desiredLinearVelocity * desiredAngle.getSin(),
            m_thetaPID.calculate(currentPosition.getRotation().getRadians()) * Math.max(Math.sqrt(Math.abs(desiredLinearVelocity * 0.25)), 1)
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
        ) / kMinimumCompensationVelocity.in(MetersPerSecond);

        return directAngle.interpolate(adjustedAngle, Math.pow(adjustFactor, 3.0));
    }
}
