package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public final class ShooterUtils {
    private static final double[][] m_angleCoefficients = {
        {
            -71.266188447569917
        },

        {
            -161.684430508642976,
            92.239379642263188
        },

        {
            -23.929486123536780,
            60.667295795086858,
            -20.485516376989938
        },

        {
            -2.364418630234339,
            7.360878175921579,
            -8.891620351789737,
            2.209791809586092
        },

        {
            -0.091830256743059,
            0.452911516474226,
            -0.737863730283898,
            0.581145838107222,
            -0.115910911316219
        },

        {
            -0.001521081562816,
            0.008526541591285,
            -0.021204463188639,
            0.023983217832612,
            -0.014099175681324,
            0.002368592334477
        }
    };

    public static final Angle getLaunchAngle(Distance distance, LinearVelocity velocity) {
        double[] distancePowers = new double[m_angleCoefficients.length + 1];
        double[] velocityPowers = new double[m_angleCoefficients.length + 1];
        distancePowers[0] = 1.0;
        velocityPowers[0] = 1.0;

        for (int i = 1; i < distancePowers.length; i++) {
            distancePowers[i] = distancePowers[i - 1] * distance.in(Meters);
            velocityPowers[i] = velocityPowers[i - 1] * velocity.in(MetersPerSecond);
        }

        double angle = 0.0;

        for (int i = 0; i < m_angleCoefficients.length; i++) {
            double[] degreeCoefficients = m_angleCoefficients[i];

            for (int j = 0; j < degreeCoefficients.length; j++) {
                angle += degreeCoefficients[j] * distancePowers[i - j] * velocityPowers[j];
            }
        }

        return Degrees.of(angle);
    }

    public static final void getLeadPose(DrivetrainSubsystem drivetrain) {
        Pose2d robotPose = drivetrain.getEstimatedPose();
        ChassisSpeeds robotSpeeds = drivetrain.getChassisSpeeds();
        Translation2d allianceHubTranslation = FieldUtils.getAllianceHub();

        ChassisSpeeds hubSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, robotPose.getRotation().minus(allianceHubTranslation.minus(robotPose.getTranslation()).getAngle())
        );
    }

    public static final Pair<Angle, Angle> getLaunchAngles(
        LinearVelocity exitVelocity,
        Distance horizontalDistance,
        Distance verticalDistance
    ) {
        double v = exitVelocity.in(MetersPerSecond);
        double x = horizontalDistance.in(Meters);
        double y = verticalDistance.in(Meters);

        double principal = Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x, 2) + 2.0 * y * Math.pow(v, 2)));
        Angle principalAngle = Degrees.of(Math.toDegrees(Math.atan((Math.pow(v, 2) + principal) / (9.8 * x))));
        Angle secondaryAngle = Degrees.of(Math.toDegrees(Math.atan((Math.pow(v, 2) - principal) / (9.8 * x))));

        return Pair.of(
            principalAngle.lt(secondaryAngle) ? principalAngle : secondaryAngle,
            principalAngle.gt(secondaryAngle) ? principalAngle : secondaryAngle
        );
    }
}
