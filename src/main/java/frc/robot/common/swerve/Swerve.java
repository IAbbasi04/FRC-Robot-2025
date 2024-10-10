package frc.robot.common.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class Swerve {

    public abstract void setThrottleCurrentLimit(double currentlimit);

    public abstract SwerveDriveKinematics getKinematics();

    public abstract ChassisSpeeds getCurrentSpeeds();
    
    public abstract Rotation2d getGyroscopeRotation();

    public abstract void resetPose(Pose2d pose); 

    public abstract void resetGyro(double degrees); 

    public abstract Pose2d getCurrentPose();
    
    public abstract void driveFieldCentric(ChassisSpeeds speeds);

    public abstract void driveRobotCentric(ChassisSpeeds speeds);

}