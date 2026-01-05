package frc.robot;

public final class RobotConstants {
    public static final RobotBehavior kRobotBehavior =
        Robot.isReal() ? RobotBehavior.REAL : RobotBehavior.SIMULATION;

    public static enum RobotBehavior {
        REAL,
        SIMULATION,
        REPLAY
    }
}
