package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public Command stowElevator() {
        CommandScheduler.getInstance().cancelAll();
        return this.setState(ElevatorState.kStow);
    }
}