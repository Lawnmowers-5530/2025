package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages all pose cameras. Designed to declutter
 * {@link frc.robot.subsystems.Swerve Swerve}.
 */
public class PoseCameraManager extends SubsystemBase {
    private ArrayList<PoseCamera> camList = new ArrayList<>();

    public PoseCameraManager() {
        camList.add(new PoseCamera("Front", new Transform3d()));
        //camList.add(new PoseCamera("Front2", new Transform3d()));
    }

    /**
     * Fetch estimated pose from all registered pose cameras paired with respective
     * std devs
     * 
     * @return List of Optional estimated robot poses from cameras paired with
     *         respective std devs
     */
    public ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getEstimatedPoses() {
        ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> poseList = new ArrayList<>();
        Optional<EstimatedRobotPose> estimate;
        for (PoseCamera camera : camList) {
            estimate = camera.getPoseEstimate();
            estimate.ifPresent(estimatedRobotPose -> poseList.add(Pair.of(estimatedRobotPose, camera.getEstimationStdDevs(estimatedRobotPose.estimatedPose.toPose2d()))));
        }
        return poseList;
    }

    public ArrayList<PhotonTrackedTarget> getTagsById(int fiducialTagId) {
        ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();
        for (PoseCamera camera : camList) {
            camera.getTagById(fiducialTagId).ifPresent(targets::add);
        }
        return targets;
    }

    /**
     *
     * @param fiducialTagId Tag id to find yaw of
     * @return Double of yaw if tag id is primary tag in view of a registered camera
     */
    public Optional<Double> getFiducialIdYaw(int fiducialTagId) {
        for (PoseCamera camera : camList) {
            if(camera.getPrimaryTagId() == fiducialTagId) {
                return Optional.of(camera.getPrimaryTagYaw());
            }
        }
        return Optional.empty();
    }
}
