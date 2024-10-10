package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable limelightTable; // Network table associated with the given limelight name
    private LimelightProfile profile;

    /**
     * Physical properties of the limelight
     */
    public class LimelightProfile {
        public double mountHeight = 0.0; // meters
        public double mountAngle = 0.0; // radians
        
        public LimelightProfile(double mountHeight, double mountAngle) {
            this.mountHeight = mountHeight;
            this.mountAngle = mountAngle;
        }

        /**
         * Height above the ground the limelight is mounted
         */
        public LimelightProfile setMountHeight(double mountHeight) {
            this.mountHeight = mountHeight;
            return this;
        }

        /**
         * Angle at which the limelight is mounted
         */
        public LimelightProfile setMountAngle(double mountAngle) {
            this.mountAngle = mountAngle;
            return this;
        }
    }

    public Limelight(String limelightName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /**
     * Physical properties of the limelight
     */
    public Limelight withProfile(LimelightProfile profile) {
        this.profile = profile;
        return this;
    }

    /**
     * The limelight's horizontal angle to the target
     */
    public double getX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * The limelight's vertical angle to the target
     */
    public double getY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Limelight distance in meters from the target
     */
    public double getDistanceFromTarget(double targetHeight) {
        if (isTargetValid()) {
            double angleFromHorizon = this.getY() + profile.mountAngle;
            double deltaHeight = targetHeight - profile.mountHeight;
            return deltaHeight / Math.tan(angleFromHorizon);
        }
        return -1.0;
    }

    /**
     * If there is a target visible in the limelight
     */
    public boolean isTargetValid() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Sets the visible pipeline of the limelight
     */
    public void setPipeline(double pipeline) {
        limelightTable.getEntry("pipeline").setDouble(pipeline);
    }

    /**
     * Returns the current visible pipeline number of the limelight
     */
    public int getPipeline() {
        return (int)limelightTable.getEntry("pipeline").getDouble(0);
    }
}