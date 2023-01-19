package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(FieldConstants.atfl, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.Robot2Photon);

    public VisionManager() {
        camera.setDriverMode(false);
        camera.setLED(VisionLEDMode.kOff);
    }

    public Transform3d getAprilTagRelative() {
        camera.setLED(VisionLEDMode.kOff);
        camera.setPipelineIndex(1);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    public Transform3d getReflectiveTapeRelative() {
        camera.setLED(VisionLEDMode.kOn);
        camera.setPipelineIndex(0);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            double distance = (FieldConstants.kReflectiveTrapeTargetHeight -VisionConstants.kPhotonLensHeightMeters)/Math.tan(Units.degreesToRadians(target.getPitch()));
            bestCameraToTarget = new Transform3d(new Translation3d(distance, new Rotation3d()), new Rotation3d());
            SmartDashboard.putNumber("Distance", distance);
        }
        return bestCameraToTarget;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        camera.setLED(VisionLEDMode.kOff);
        camera.setPipelineIndex(1);
        estimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> estresult = estimator.update();
        if (estresult.isPresent()) {
            return new Pair<Pose2d, Double>(
                    estresult.get().estimatedPose.toPose2d(), currentTime - estresult.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }
}

