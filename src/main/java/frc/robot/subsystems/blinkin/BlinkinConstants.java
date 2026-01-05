package frc.robot.subsystems.blinkin;

public final class BlinkinConstants {
    public static final int kBlinkinPWM = 0;

    public static enum BlinkinPattern {
        SOLID_DARK_RED      (0.59),
        SOLID_DARK_BLUE     (0.85),
        SOLID_DARK_GREEN    (0.75);

        private final double m_dutyCycle;

        BlinkinPattern(double dutyCycle) {
            m_dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return m_dutyCycle;
        }
    }
}
