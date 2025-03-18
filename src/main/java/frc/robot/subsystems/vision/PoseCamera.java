package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: consider vSLAMming every pose camera for more data and fusing in a kalman filter setup
/**
 * Class to manage all cameras designated to robot pose estimation. Each camera
 * has its own pose estimator and returns its own pose along with a std devs to
 * use.
 */
public class PoseCamera extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Optional<PhotonPipelineResult> latestResult = Optional.empty();
    
    public PoseCamera(String cameraName, Transform3d cameraPosition) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition); // Setting to use all available tags and do
                                                                            // the math on coprocessor
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // Switching to lowest ambiguity pose
                                                                                  // strategy when only one tag is
                                                                                  // visible
    }
    

    /**
     * Returns an {@link EstimatedRobotPose} if camera pipeline has a new result
     * with apriltag targets
     * 
     * @return An {@link Optional}
     */
    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (this.latestResult.isEmpty()) {
            return Optional.empty();
        } else {
            return poseEstimator.update(this.latestResult.get());
        }
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) { // TODO: tune this - this function was pulled from photon
        if (this.latestResult.isEmpty()) {
            return VecBuilder.fill(100, 100, 100);
        }

        Matrix<N3, N1> estStdDevs = VecBuilder.fill(1, 1, 100);
        List<PhotonTrackedTarget> targets = this.latestResult.get().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VecBuilder.fill(0.4, 0.4, 10000);

        // Increase std devs based on (average) distance
        else if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * Returns the apriltag id of the primary target in the camera view
     * 
     * @return The apriltag id of the primary target in the camera view
     */
    public int getPrimaryTagId() {
        if (latestResult.isEmpty()) {
            return -1;
        } else {
            return latestResult.get().getBestTarget().getFiducialId();
        }
    }

    public Optional<PhotonTrackedTarget> getPrimaryTrackedTarget() {
        if (latestResult.isEmpty()) {
            return Optional.empty();
        } 
        if (!hasTargets())
        {
            return Optional.empty();
        }
        else {
            return Optional.of(latestResult.get().getBestTarget());
        }
    }

    /**
     * Returns the yaw value of the primary tag in the camera view
     * 
     * @return The yaw value of the primary tag in the camera view
     */
    public double getPrimaryTagYaw() {
        if (latestResult.isEmpty()) {
            return -1;
        } else {
            return latestResult.get().getBestTarget().getYaw();
        }
    }

    public Optional<PhotonTrackedTarget> getTagById(Optional<Integer> tagId) {
        if (tagId.isEmpty() || latestResult.isEmpty()) {
            return Optional.empty();
        }
        return latestResult.get()
                .targets
                .stream()
                .filter(target -> target.getFiducialId() == tagId.get())
                .findFirst();
    }

    public boolean hasTargets() {
        if (latestResult.isEmpty()) {
            return false;
        } else {
            return latestResult.get().hasTargets();
        }
    }

    /**
     * Calculates and stores the appropiate std devs for the newest pose estimation
     * reading
     */
    @Override
    public void periodic() {
        var results = this.camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            this.latestResult = Optional.of(results.get(results.size() - 1));
        }
    }
}
