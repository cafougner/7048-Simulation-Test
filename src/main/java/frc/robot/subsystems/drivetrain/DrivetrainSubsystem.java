package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.gyro.Gyro;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModule;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;

public final class DrivetrainSubsystem extends SubsystemBase {
    private final Gyro m_gyro;
    private final SwerveModule[] m_swerveModules = new SwerveModule[4];

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        kFLSwerveModuleConfiguration.getModuleTranslation(),
        kFRSwerveModuleConfiguration.getModuleTranslation(),
        kBLSwerveModuleConfiguration.getModuleTranslation(),
        kBRSwerveModuleConfiguration.getModuleTranslation()
    );

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveSimulation m_simulation;

    public DrivetrainSubsystem(
        GyroIO gyroIO,
        SwerveModuleIO flSwerveModuleIO,
        SwerveModuleIO frSwerveModuleIO,
        SwerveModuleIO blSwerveModuleIO,
        SwerveModuleIO brSwerveModuleIO,
        Optional<SwerveDriveSimulation> simulation
    ) {
        m_gyro = new Gyro(gyroIO);
        m_swerveModules[0] = new SwerveModule(flSwerveModuleIO, "FL");
        m_swerveModules[1] = new SwerveModule(frSwerveModuleIO, "FR");
        m_swerveModules[2] = new SwerveModule(blSwerveModuleIO, "BL");
        m_swerveModules[3] = new SwerveModule(brSwerveModuleIO, "BR");

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_gyro.getRotation(),
            getSwerveModulePositions(),
            Pose2d.kZero
        );

        if (simulation.isPresent()) {
            m_simulation = simulation.get();
        } else {
            m_simulation = null;
        }
    }

    @Override
    public final void periodic() {
        m_gyro.periodic();

        for (SwerveModule swerveModule : m_swerveModules) {
            swerveModule.periodic();
        }

        m_poseEstimator.update(m_gyro.getRotation(), getSwerveModulePositions());

        Logger.recordOutput("DrivetrainSubsystem/ChassisSpeeds", getRobotChassisSpeeds());
        Logger.recordOutput("DrivetrainSubsystem/CurrentStates", getSwerveModuleStates());
        Logger.recordOutput("DrivetrainSubsystem/EstimatedPose", getEstimatedPose());
    }

    @Override
    public final void simulationPeriodic() {
        Logger.recordOutput("DrivetrainSubsystem/SimulationPose", getSimulationPose());
    }

    public final void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean blueRelative) {
        Rotation2d estimatedRotation = getEstimatedPose().getRotation();

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                blueRelative ? estimatedRotation : estimatedRotation.plus(Rotation2d.kPi)
            );
        }

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Robot.defaultPeriodSecs);
        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxAttainableLinearSpeed);

        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public final Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public final Pose2d getSimulationPose() {
        return m_simulation.getSimulatedDriveTrainPose();
    }

    public final SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            m_swerveModules[0].getSwerveModulePosition(),
            m_swerveModules[1].getSwerveModulePosition(),
            m_swerveModules[2].getSwerveModulePosition(),
            m_swerveModules[3].getSwerveModulePosition()
        };
    }

    public final SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            m_swerveModules[0].getSwerveModuleState(),
            m_swerveModules[1].getSwerveModuleState(),
            m_swerveModules[2].getSwerveModuleState(),
            m_swerveModules[3].getSwerveModuleState()
        };
    }

    public final ChassisSpeeds getRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public final ChassisSpeeds getFieldChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getRobotChassisSpeeds(), m_poseEstimator.getEstimatedPosition().getRotation()
        );
    }

    public final SwerveDrivePoseEstimator getPoseEstimator() {
        return m_poseEstimator;
    }
}
