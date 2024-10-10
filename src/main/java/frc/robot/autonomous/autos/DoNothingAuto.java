package frc.robot.autonomous.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.BaseAuto;

public class DoNothingAuto extends BaseAuto {
    @Override
    public Command createAuto() {
        return new WaitCommand(0.0);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d();
    }
}