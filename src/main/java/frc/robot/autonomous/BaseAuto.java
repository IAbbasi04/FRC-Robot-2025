package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class BaseAuto extends Command {
    public abstract Command createAuto();

    public abstract Pose2d getStartPose();
}