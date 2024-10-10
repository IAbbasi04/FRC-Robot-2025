package frc.robot.hardware;

import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import frc.robot.common.Constants;

public class PhotonCam {
    private PhotonCamera camera;

    public PhotonCam(String name) {
        this.camera = new PhotonCamera(name);
    }

    /**
     * Grabs all visible targets
     */
    public List<PhotonTrackedTarget> getTargets() {
        return this.camera.getLatestResult().targets;
    }

    /**
     * Grabs the target most matching the current pipeline
     */
    public PhotonTrackedTarget getBestTarget() {
        return this.camera.getLatestResult().getBestTarget();
    }

    /**
     * Sets the current pipeline index
     */
    public void setPipeline(int pipeline) {
        this.camera.setPipelineIndex(pipeline);
    }

    /**
     * Whether there is any target visible in the camera
     */
    public boolean isAnyTargetVisible() {
        return this.camera.getLatestResult().getBestTarget() != null;
    }

    /**
     * Whether a particular AprilTag is visible in the camera
     */
    public boolean isAprilTagVisible(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToBestTarget() {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(this.camera.getLatestResult().getBestTarget().getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToTarget(PhotonTrackedTarget target) {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(target.getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToAprilTag(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return Rotation2d.fromDegrees(target.getYaw());
        }
        return null; // Return null if the particular april tag is not seen by the camera
    }

    /**
     * The calculated 2d field position of the robot based on april tag vision data
     */
    public Pose2d getEstimatedFieldPose() {
        return PhotonUtils.estimateFieldToRobotAprilTag(
            getBestTarget().getBestCameraToTarget(), 
            Constants.VISION.APRIL_TAG_LAYOUT.getTagPose(
                getBestTarget().getFiducialId()
            ).get(), 
            Constants.VISION.CAMERA_POSE_TO_ROBOT_POSE
        ).toPose2d();
    }
}