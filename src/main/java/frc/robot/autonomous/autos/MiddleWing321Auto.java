package frc.robot.autonomous.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.BaseAuto;

public class MiddleWing321Auto extends BaseAuto {
    @Override
    public Command createAuto() {
        return new PathPlannerAuto("MiddleWing321Auto");
    }

    @Override
    public Pose2d getStartPose() {
        return PathPlannerAuto.getStaringPoseFromAutoFile("MiddleWing321Auto");
    }
}