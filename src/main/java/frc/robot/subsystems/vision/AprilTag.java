package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTag extends SubsystemBase {

    PhotonCamera camera;
    PhotonPipelineResult result;

    public AprilTag(PhotonCamera camera) {
        this.camera = camera;
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }

    public Transform3d getAprilTagRelative() {
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        double yaw;
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }
}

