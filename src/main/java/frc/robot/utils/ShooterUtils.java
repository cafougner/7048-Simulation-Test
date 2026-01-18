package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class ShooterUtils {
    private static final double[][] m_angleCoefficients = {
        {
            1695.041783925697927
        },

        {
            602.314552640619467,
            -1124.639476832428272
        },

        {
            72.978520956807457,
            -326.518646504040304,
            310.541389971889600
        },

        {
            0.9046076242583777,
            -19.391425938486499,
            60.949652776587200,
            -42.237838238371296
        },

        {
            -2.067273497845160,
            0.794911650349824,
            1.960512345921958,
            -5.020213333043194,
            2.831365213474911
        },

        {
            0.153553556024739,
            0.086592155539089,
            -0.076385055218013,
            -0.060539898363824,
            0.153301670128456,
            -0.074882671132399
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
}
