package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class ShooterUtils {
    // Coefficients must use rectangular mapping (necessitated by the MathUtils implementations of polynomials).
    private static final double[][] m_angleCoefficients = {
        {  -71.266188447569920,   92.239379642263190,  -20.485516376989940,    2.209791809586092,   -0.115910911316219,    0.002368592334477},
        { -161.684430508642980,   60.667295795086860,   -8.891620351789737,    0.581145838107222,   -0.014099175681324,    0.000000000000000},
        {  -23.929486123536780,    7.360878175921579,   -0.737863730283898,    0.023983217832612,    0.000000000000000,    0.000000000000000},
        {   -2.364418630234339,    0.452911516474226,   -0.021204463188639,    0.000000000000000,    0.000000000000000,    0.000000000000000},
        {   -0.091830256743059,    0.008526541591285,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000},
        {   -0.001521081562816,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000,    0.000000000000000}
    };

    public static final Angle getPolynomialAngle(Distance distance, LinearVelocity velocity) {
        return Degrees.of(
            MathUtils.evaluateBivariate(
                m_angleCoefficients,
                distance.in(Meters),
                velocity.in(MetersPerSecond)
            )
        );
    }

    public static final Pair<Angle, Angle> getQuadraticAngles(
        Distance horizontalDistance,
        Distance verticalDistance,
        LinearVelocity velocity
    ) {
        double v = velocity.in(MetersPerSecond);
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
