package frc.robot.commands;

import frc.robot.common.crescendo.ShotProfile;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class ShootCommand extends NewtonCommand {
    public ShootCommand(ShotProfile desiredShot) {
        super(
            m_shooter.getCommands().setShooting(desiredShot)
                .alongWith(
                    m_elevator.getCommands()
                        .setOverridePosition(desiredShot.pivotDegrees, desiredShot.extensionMeters)
                )
                .until(() -> (m_shooter.isAtTargetSpeed() && m_elevator.atTargetPosition()))
                .andThen(m_feeder.getCommands().setFeederState(FeederState.kShoot))
            );
    }
}