package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.common.crescendo.ShotProfile;

public class PrimeCommand extends NewtonCommand {
    public PrimeCommand(ShotProfile desiredShot) {
        super(
            new ConditionalCommand(
                m_shooter.getCommands().setShooting(desiredShot)
                .alongWith(
                    m_elevator.getCommands()
                        .setOverridePosition(
                            desiredShot.pivotDegrees, 
                            desiredShot.extensionMeters
                        )
                )
                .until(() -> false), // Never return true
                new IntakeCommand() // Finish intaking
                .andThen(
                    m_shooter.getCommands().setShooting(desiredShot)
                    .alongWith(
                        m_elevator.getCommands()
                            .setOverridePosition(
                                desiredShot.pivotDegrees, 
                                desiredShot.extensionMeters
                            )
                    )
                    .until(() -> false) // Never return true
                ), 
                () -> (m_feeder.hasNote() && m_feeder.noteStaged()) || Robot.isSimulation()
            )
        );
    }
}