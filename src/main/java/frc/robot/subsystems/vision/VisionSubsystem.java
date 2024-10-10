package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.common.Constants;
import frc.robot.common.MatchMode;
import frc.robot.common.crescendo.Field2024;
import frc.robot.common.crescendo.ShotProfile;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.OrangePy;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.common.crescendo.AprilTags.*;

public class VisionSubsystem extends NewtonSubsystem {
    private static VisionSubsystem INSTANCE = null;
    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }
        return INSTANCE;
    }

    private OrangePy rearOakD;
    private Limelight frontLimelight;

    private VisionSubsystem() {
        rearOakD = new OrangePy(Constants.VISION.REAR_ORANGE_PY_NAME);
        frontLimelight = new Limelight(Constants.VISION.FRONT_LIMELIGHT_NAME);
    }

    /**
     * Distance to the centered april tag on the speaker
     */
    public double getDistanceToSpeaker() {
        double distance = rearOakD.distanceToAprilTag(
            List.of(BLUE_SPEAKER_CENTER.id(), 
                    RED_SPEAKER_CENTER.id()
                )
            );

        if (distance == -1) {
            Pose2d currentPose = SwerveSubsystem.getInstance().getCurrentPose();
            Pose2d targetPose = Field2024.SPEAKER_OPENING.getPose(true);
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {  
                    targetPose = new Pose2d(
                        new Translation2d(
                            Constants.FIELD.RED_WALL_X - targetPose.getX(),
                            targetPose.getY()
                        ),
                        targetPose.getRotation()
                    );
                }
            }
            Transform2d deltas = targetPose.minus(currentPose);
            distance = Math.sqrt(Math.pow(deltas.getX(), 2) + Math.pow(deltas.getY(), 2));
        }
        return distance;
    }

    /**
     * Yaw offset from speaker
     */
    public Rotation2d getAngleToSpeaker() {
        double offset = rearOakD.getXOffsetFromTag(BLUE_SPEAKER_CENTER.id());
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {  
                offset = rearOakD.getXOffsetFromTag(RED_SPEAKER_CENTER.id());
            }
        }
        
        if (Double.isNaN(offset)) {
            Pose2d currentPose = SwerveSubsystem.getInstance().getCurrentPose();
            Pose2d targetPose = Field2024.SPEAKER_OPENING.getPose(true);
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {  
                    targetPose = new Pose2d(
                        new Translation2d(
                            Constants.FIELD.RED_WALL_X - targetPose.getX(),
                            targetPose.getY()
                        ),
                        targetPose.getRotation()
                    );
                }
            }
            Transform2d deltas = targetPose.minus(currentPose);
            offset = Units.radiansToDegrees(Math.atan(deltas.getY()/deltas.getX()));
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                  offset += 180;
                }
            }
        }
        return Rotation2d.fromDegrees(offset);
    }

    /**
     * Whether the april tag on the speaker is visible or not
     */
    public boolean isSpeakerTargetVisible() {
        return rearOakD.getCurrTagID() == RED_SPEAKER_CENTER.id() || 
            rearOakD.getCurrTagID() == BLUE_SPEAKER_CENTER.id() || 
            rearOakD.getCurrTag2ID() == RED_SPEAKER_CENTER.id() || 
            rearOakD.getCurrTag2ID() == BLUE_SPEAKER_CENTER.id();
    }

    /**
     * Whether a note is in view of the front camera
     */
    public boolean isNoteInView() {
        return frontLimelight.isTargetValid();
    }

    /**
     * Whether the visible note is in suitable targettable
     */
    public boolean isNoteTargettable() {
        return isNoteInView() && frontLimelight.getY() <= -15.0;
    }

    /**
     * The calculated 2d pose of the robot based on April Tag data
     */
    public Pose2d getEstimatedPose() {
        if (rearOakD.getTagInView()) {
            // int tagID = rearOakD.getCurrTagID();
            // double dx = rearOakD.getCurrTagX();
            // double dy = rearOakD.getCurrTagY();
            // double dz = rearOakD.getCurrTagZ();
            // Pose3d tagPose = Constants.FIELD.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get();
            // Pose2d robotPose = new Pose2d(
            //     new Translation2d(
            //         tagPose.getX() - dx,
            //         tagPose.getY() - dy
            //     ),
            //     new Rotation2d()
            // );
            return new Pose2d();
        }
        return null; // Return null if there is no tag visible
    }
    
    @Override
    public void onInit(MatchMode mode) {

    }

    @Override
    public void periodicLogs() {
        m_logger.logDouble("Distance to Target (m)", getDistanceToSpeaker());

        ShotProfile rangedShotFromHere = Robot.SHOT_TABLE.getShotFromDistance(getDistanceToSpeaker());
        m_logger.logDouble("Distance Based Desired Pivot Angle", rangedShotFromHere.pivotDegrees);
        m_logger.logDouble("Distance Based Desired Left RPM", rangedShotFromHere.leftShotRPM);
        m_logger.logDouble("Distance Based Desired Right RPM", rangedShotFromHere.rightShotRPM);
    }

    @Override
    public void periodicOutputs() {

    }
}