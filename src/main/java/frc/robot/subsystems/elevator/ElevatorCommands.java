package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

public class ElevatorCommands {
    private ElevatorSubsystem m_elevator;

    public ElevatorCommands(ElevatorSubsystem elevator) {
        this.m_elevator = elevator;
    }

    public Command setState(ElevatorState state) {
        return this.m_elevator.runOnce(() -> {
            m_elevator.setElevatorState(state);
        });
    }

    public Command setOverridePosition(double pivotAngle, double elevatorExtension) {
        return this.m_elevator.run(() -> {
            m_elevator.setExtension(elevatorExtension);
            m_elevator.setPivot(pivotAngle);
        });
    }
}