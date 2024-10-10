package frc.robot.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class OrangePy {
    private String name;

    private double tag_x;
    private double tag_y;
    private double tag_z;
    private double tag_yaw; 
    private int curr_tag_id;

    private double tag2_x;
    private double tag2_y;
    private double tag2_z;
    private double tag2_yaw; 
    private int curr_tag2_id;

    public String[] VISUAL_SERVO_TARGETS = {
        "RELATIVE_X", // left-right
        "RELATIVE_Y", // up-down (will probably never need this)
        "RELATIVE_Z", // forward distance to apriltag projected out from the tag or camera
        "RELATIVE_YAW" // e.g. even if mis-aligned, robot is pointing in same direction as apriltag
    };

    public OrangePy(String name) {
        this.name = name;
    }

    /* -- CURRENT TAG -- */
    public double getCurrTagX() {
        tag_x = SmartDashboard.getNumber(name + "_apriltag_x", 0.0);
        return tag_x;
    }

    public double getCurrTagY() {
        tag_y = SmartDashboard.getNumber(name + "_apriltag_y", 0.0);
        return tag_y;
    }

    public double getCurrTagZ() {
        tag_z = SmartDashboard.getNumber(name + "_apriltag_z", 0.0);
        return tag_z;
    }

    public int getCurrTagID() {
        curr_tag_id = (int)SmartDashboard.getNumber(name + "_apriltag_id", 0.0);
        return curr_tag_id;
    }

    public double getCurrTagYaw() {
        // negative so that clockwise positive
        tag_yaw = -SmartDashboard.getNumber(name + "_apriltag_pitch", 0); // coordinate systems are weird
        return tag_yaw;
    }

    public boolean getTagInView() {
        boolean tagInView = SmartDashboard.getBoolean(name + "_tag_visible", false);
        return tagInView;
    }

    public boolean aprilTagValid(int[] tags) {
        return getTagInView() && tags.toString().contains("" + getCurrTagID()) ||
            getTag2InView() && tags.toString().contains("" + getCurrTag2ID());
    }

    /* -- CURRENT TAG 2 (CHECK IF IN VIEW) -- */
    public double getCurrTag2X() {
        tag2_x = SmartDashboard.getNumber(name + "2_apriltag_x", 0.0);
        return tag2_x;
    }

    public double getCurrTag2Y() {
        tag2_y = SmartDashboard.getNumber(name + "2_apriltag_y", 0.0);
        return tag2_y;
    }

    public double getCurrTag2Z() {
        tag2_z = SmartDashboard.getNumber(name + "2_apriltag_z", 0.0);
        return tag2_z;
    }

    public int getCurrTag2ID() {
        curr_tag2_id = (int)SmartDashboard.getNumber(name + "2_apriltag_id", 0.0);
        return curr_tag2_id;
    }

    public double getCurrTag2Yaw() {
        tag2_yaw = SmartDashboard.getNumber(name + "2_apriltag_pitch", 0); // coordinate systems are weird
        return tag2_yaw;
    }

    public boolean getTag2InView() {
        boolean tagInView = SmartDashboard.getBoolean(name + "2_tag_visible", false);
        return tagInView;
    }

    /* -- ABSOLUTE POSITIONING */
    public double getX() {
        return SmartDashboard.getNumber("vision_x", 0.0);
    }

    public double getY() {
        return SmartDashboard.getNumber("vision_y", 0.0);
    }

    public double getZ() {
        return SmartDashboard.getNumber("vision_z", 0.0);
    }

    public double getYaw() {
        return SmartDashboard.getNumber("vision_yaw", 0.0);
    }

    public Pose2d getPose2d() {
        return new Pose2d(getX(), getY(), new Rotation2d(getYaw()));
    }

    /**
     * Return "tx" equivalent from limelight
     * @return tx calculated from Oak-D data
     */
    public double getTagTx() {
        // atan2(z, x) - yaw  
        // verify math is correct

        // SOP log
        System.out.println("atan2(z,x) = " + Math.atan2(getCurrTagZ(), getCurrTagX()));
        System.out.println("yaw = " + getCurrTagYaw());

        return Math.atan2(getCurrTagX(), getCurrTagZ()) - getCurrTagYaw();
    }

    /**
     * Horizontal offset from specified april tag
     */
    public double getXOffsetFromTag(int tag) {        
        if (tag == getCurrTagID()) {
            return getCurrTagX();
        } else if (tag == getCurrTag2ID()) {
            return getCurrTag2X();
        } else {
            return Double.NaN;
        }
    }

    //     return out;
    // }
    /**
     * Returns the distance to the apriltag in ground plane
     * Returns -1 if tag ID not in view
     * @param id
     * @return distance to apriltag (meters)
     */
    public double distanceToAprilTag(int id) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && getCurrTagID() == id) {
            // it's tag 1
            return getCurrTagZ();
        }    
        else if (getTag2InView() && getCurrTag2ID() == id) {
            // it's tag 2
            return getCurrTag2Z();
        }
        else {
            return -1.0; // tag not in view
        }
    }

    /**
     * Returns the distance to the apriltag in ground plane
     * Returns -1 if tag ID not in view
     * @param ids list of tag IDs
     * @return distance to apriltag (meters)
     */
    public double distanceToAprilTag(List<Integer> ids) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && ids.contains(getCurrTagID())) {
            // it's tag 1
            return getCurrTagZ();
        }    
        else if (getTag2InView() && ids.contains(getCurrTag2ID())) {
            // it's tag 2
            return getCurrTag2Z();
        }
        else {
            return -1.0; // tag not in view
        }
    }

    public double offsetFromAprilTag(List<Integer> ids) {
        // check if it's tag 1 or tag 2, first check if it's in view
        // return directly because we are more confidient in tag 1
        if (getTagInView() && ids.contains(getCurrTagID())) {
            // it's tag 1
            return getCurrTagX();
        }    
        else if (getTag2InView() && ids.contains(getCurrTag2ID())) {
            // it's tag 2
            return getCurrTag2X();
        }
        else {
            return -1.0; // tag not in view
        }
    }
}