package frc.robot.commands;

import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class StowCommand extends NewtonCommand {
    public StowCommand() {
        super(
            new SetElevatorCommand(ElevatorState.kStow)
            .alongWith(m_intake.getCommands().setOff())
            .alongWith(m_feeder.getCommands().setFeederState(FeederState.kOff))
            .alongWith(m_shooter.getCommands().setOff())
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        addRequirements(m_intake, m_feeder, m_shooter, m_elevator);
    }
}