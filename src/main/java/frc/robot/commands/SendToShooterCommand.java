package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class SendToShooterCommand extends NewtonCommand {
    public SendToShooterCommand() {
        super(
            // If we have a set shot already then use that
            // Otherwise use basic subwoofer shot
            new ConditionalCommand(
                m_feeder.getCommands().setFeederState(FeederState.kShoot), 
                new ShootCommand(Robot.SHOT_TABLE.getSubwooferShot()), 
                () -> m_shooter.getDesiredLeftVelocity() > 0d
            )
        );
    }
}
