package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import io.github.oblarg.oblog.Loggable;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages all pose cameras. Designed to declutter
 * {@link Swerve Swerve}.
 */
public class PoseCameraManager implements Loggable {
    private ArrayList<PoseCamera> camList = new ArrayList<>();
    Field2d field = new Field2d();

    public PoseCameraManager() {
        camList.add(new PoseCamera("Front", new Transform3d()));
        camList.add(new PoseCamera("Front2", new Transform3d()));
        Shuffleboard.getTab("Vision").add("Field", field);
        // change?
        // camList.add(new PoseCamera("cam2", new Transform3d()));
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
            estimate.ifPresent(estimatedRobotPose -> poseList.add(Pair.of(estimatedRobotPose,
                    camera.getEstimationStdDevs(estimatedRobotPose.estimatedPose.toPose2d()))));
        }
        return poseList;
    }

    public ArrayList<PhotonTrackedTarget> getTagsById(Optional<Integer> fiducialTagId) {
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
            if (camera.getPrimaryTagId() == fiducialTagId) {
                return Optional.of(camera.getPrimaryTagYaw());
            }
        }
        return Optional.empty();
    }

    public Optional<Integer> getPrimaryIdLeft() {
        if (camList.get(0).hasTargets() == true) {
            return Optional.of(camList.get(0).getPrimaryTagId());
        }
        return Optional.empty();
    }

    public Optional<Integer> getPrimaryIdRight() {
        if (camList.get(1).hasTargets() == true) {
            return Optional.of(camList.get(1).getPrimaryTagId());
        }
        return Optional.empty();
    }

    public void periodic() {
        Pose3d pose = new Pose3d();

        int added_estimates = 0;
        for (PoseCamera camera : camList) {
            if (camera.hasTargets()) {
                var estimate = camera.getPoseEstimate();
                if (estimate.isEmpty()) {
                    continue;
                }
                var est = estimate.get().estimatedPose;

                pose.plus(new Transform3d(est.getX(), est.getY(), est.getZ(), est.getRotation()));
                added_estimates++;
            }
        }

        pose.div(added_estimates);
        field.setRobotPose(pose.toPose2d());
    }
}
