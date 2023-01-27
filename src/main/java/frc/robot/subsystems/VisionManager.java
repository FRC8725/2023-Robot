package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;

import java.util.Optional;

public class VisionManager extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("OV5647");
    UsbCamera usbCamera;
    CvSink cvSink;
    PhotonPipelineResult result = new PhotonPipelineResult();
    // Pair<PhotonCamera, Transform3d> campair = new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot);
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(FieldConstants.aprilTagField, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.Robot2Photon);

    public VisionManager() {
        usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(VisionConstants.UsbCameraResolution[0], VisionConstants.UsbCameraResolution[1]);
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

    public double getReflectiveTapeRelativeYawRads() {
        camera.setPipelineIndex(0);
        PhotonTrackedTarget target;
        double angle = 0;
        if (result.hasTargets()) {
            target = result.getBestTarget();
            angle = -Units.degreesToRadians(target.getYaw());
        }
        return angle;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        camera.setPipelineIndex(1);
        estimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> estimatorResult = estimator.update();
        if (estimatorResult.isPresent()) {
            return new Pair<Pose2d, Double>(
                    estimatorResult.get().estimatedPose.toPose2d(), currentTime - estimatorResult.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public void setDriverMode(boolean driverMode) {
        camera.setDriverMode(driverMode);
    }

    public boolean isCone() {
        Mat img = new Mat();
        cvSink.grabFrame(img);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        double[] centerColor = img.get(VisionConstants.UsbCameraResolution[0]/2, VisionConstants.UsbCameraResolution[1]/2);
        // inRange(HSV, hThreshold, lThreshold)
        return !(centerColor[0] < VisionConstants.kYellowLowThreshold[0] || centerColor[0] > VisionConstants.kYellowHighThreshold[0] ||
                centerColor[1] < VisionConstants.kYellowLowThreshold[1] || centerColor[1] > VisionConstants.kYellowHighThreshold[1] ||
                centerColor[2] < VisionConstants.kYellowLowThreshold[2] || centerColor[2] > VisionConstants.kYellowHighThreshold[2]
        );
    }

    boolean isFirstConnected = false;
    @Override
    public void periodic() {
        if (usbCamera.isConnected()) cvSink = CameraServer.getVideo(usbCamera);
        if (!camera.isConnected()) return;
        if (isFirstConnected) {
            camera.setDriverMode(true);
            isFirstConnected = true;
        }
        result = camera.getLatestResult();
    }
}

