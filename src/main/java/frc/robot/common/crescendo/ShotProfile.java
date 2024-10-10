package frc.robot.common.crescendo;

public class ShotProfile {
    public double leftShotRPM, rightShotRPM, pivotDegrees, extensionMeters;
    public boolean shouldShoot;

    public ShotProfile(double leftShotRPM, double rightShotRPM, double pivotDegrees, double extensionMeters) {
        this.leftShotRPM = leftShotRPM;
        this.rightShotRPM = rightShotRPM;
        this.pivotDegrees = pivotDegrees;
        this.extensionMeters = extensionMeters;
    }

    public ShotProfile() {
        this(0, 0, 0, 0);
    }

    public ShotProfile flywheel(double left, double right) {
        this.leftShotRPM = left;
        this.rightShotRPM = right;
        return this;
    }

    public ShotProfile pivot(double angle) {
        this.pivotDegrees = angle;
        return this;
    }

    public ShotProfile extension(double extension) {
        this.extensionMeters = extension;
        return this;
    }

    public ShotProfile shouldShoot(boolean shouldShoot) {
        this.shouldShoot = shouldShoot;
        return this;
    }
}