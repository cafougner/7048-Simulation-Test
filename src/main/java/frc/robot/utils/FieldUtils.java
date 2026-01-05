package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public final class FieldUtils {
    // This can be modified to use a custom field, like a calibrated one from WPIcal. Replace
    // `AprilTagFields.xyz` with a custom JSON as described in the example.txt in the deploy folder.
    private static final AprilTagFieldLayout m_aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private static final List<Pose2d> m_reefAprilTagPoses = new ArrayList<>(12);
    private static final List<Pose2d> m_stationAprilTagPoses = new ArrayList<>(4);

    static {
        for (var aprilTag : m_aprilTagLayout.getTags()) {
            switch (aprilTag.ID) {
                case 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 -> {
                    m_reefAprilTagPoses.add(aprilTag.pose.toPose2d());
                }

                case 1, 2, 12, 13 -> {
                    m_stationAprilTagPoses.add(aprilTag.pose.toPose2d());
                }
            }
        }
    }

    public static final AprilTagFieldLayout getAprilTagLayout() {
        return m_aprilTagLayout;
    }

    public static final Pose2d getNearestReefAprilTagPose(Pose2d currentPose) {
        return currentPose.nearest(m_reefAprilTagPoses);
    }

    public static final Pose2d getNearestStationAprilTagPose(Pose2d currentPose) {
        return currentPose.nearest(m_stationAprilTagPoses);
    }
}
