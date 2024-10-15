package frc.robot.commands;

import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class StowCommand extends NewtonCommand {
    public StowCommand() {
        super(
            m_elevator.getCommands().stowElevator()
            .alongWith(m_intake.getCommands().setOff())
            .alongWith(m_feeder.getCommands().setFeederState(FeederState.kOff))
            .alongWith(m_shooter.getCommands().setOff())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    }
}