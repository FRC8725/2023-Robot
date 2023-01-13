package frc.robot.subsystems;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;

import java.util.List;
import java.util.Optional;

public class VisionManager extends SubsystemBase {

    static PhotonCamera camera = new PhotonCamera("");
    static PhotonPipelineResult result;
    // Pair<PhotonCamera, Transform3d> campair = new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot);
    static RobotPoseEstimator estimator = new RobotPoseEstimator(FieldConstants.atfl, RobotPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, List.of(new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot)));

    public VisionManager() {
        camera.setDriverMode(false);
    }

    static public Transform3d getAprilTagRelative() {
        camera.setPipelineIndex(0);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    static public Transform3d getReflectiveTapeRelative() {
        camera.setPipelineIndex(1);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    static public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        camera.setPipelineIndex(0);
        estimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> estresult = estimator.update();
        if (estresult.isPresent()) {
            return new Pair<Pose2d, Double>(
                    estresult.get().getFirst().toPose2d(), currentTime - estresult.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    static public boolean hasTarget() {
        return result.hasTargets();
    }

    static public void setLED(boolean isOn) {
        camera.setLED(isOn? VisionLEDMode.kOn: VisionLEDMode.kOff);
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }
}

