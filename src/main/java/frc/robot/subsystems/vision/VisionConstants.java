package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class VisionConstants {
    public static final PoseStrategy kSingleTagPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    public static final PoseStrategy kMultiTagPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final Distance kMaxSingleTagDistance = Meters.of(0.75);
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(15.0));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0));
    
    public static final Matrix<N3, N1> kIgnoredStdDevs = VecBuilder.fill(
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
    );
}
