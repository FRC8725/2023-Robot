package frc.robot.subsystems;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    PhotonCamera camera = new PhotonCamera("OV5647");
    PhotonPipelineResult result = new PhotonPipelineResult();
    // Pair<PhotonCamera, Transform3d> campair = new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot);
    RobotPoseEstimator estimator = new RobotPoseEstimator(FieldConstants.atfl, RobotPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, List.of(new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot)));

    public VisionManager() {
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOff);
    }

    public Transform3d getAprilTagRelative() {
        camera.setPipelineIndex(1);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    public Transform2d getReflectiveTapeRelative() {
        camera.setPipelineIndex(0);
        PhotonTrackedTarget target;
        Transform2d bestCameraToTarget = new Transform2d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            double distance = (FieldConstants.kReflectiveTrapeTargetHeight -VisionConstants.kPhotonLensHeightMeters)/Math.tan(target.getPitch());
            bestCameraToTarget = new Transform2d(new Translation2d(distance, Rotation2d.fromDegrees(target.getYaw())), new Rotation2d());
        }
        return bestCameraToTarget;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
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

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public void setLED(boolean isOn) {
        camera.setLED(isOn? VisionLEDMode.kOn: VisionLEDMode.kOff);
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }
}

