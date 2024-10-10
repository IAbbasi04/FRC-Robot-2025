package frc.robot.commands;

import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

public class SetElevatorCommand extends NewtonCommand {
    public SetElevatorCommand(ElevatorState state) {
        super(
            m_elevator.getCommands().setState(state)
            .until(() -> m_elevator.atTargetPosition())
        );
    }
}