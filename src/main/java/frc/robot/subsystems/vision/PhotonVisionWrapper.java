package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionWrapper extends SubsystemBase{
    PhotonCamera cam;
    AprilTagFieldLayout layout;
    PhotonPoseEstimator est;
    Field2d field = new Field2d();

    public PhotonVisionWrapper() {
        //Forward Camera
        cam = new PhotonCamera("photonvision");
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        
        Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        est = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return est.update(cam.getLatestResult());
    }

    @Override
    public void periodic() {
        var estimation = getEstimatedGlobalPose();
        estimation.ifPresent(estimatedRobotPose -> {
            field.setRobotPose(estimatedRobotPose.estimatedPose.toPose2d());
        });
        SmartDashboard.putData("Field", field);
    }
}
