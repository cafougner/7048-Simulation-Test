package frc.robot.utils;

public final class MathUtils {
    private MathUtils() {}

    public static final double evaluateUnivariate(double[] coeffs, double x) {
        double result = 0.0;

        for (int i = coeffs.length - 1; i >= 0; i--) {
            result = Math.fma(result, x, coeffs[i]);
        }

        return result;
    }

    public static final double evaluateBivariate(double[][] coeffs, double x, double y) {
        double result = 0.0;

        for (int i = coeffs.length - 1; i >= 0; i--) {
            result = Math.fma(result, x, evaluateUnivariate(coeffs[i], y));
        }

        return result;
    }
}
