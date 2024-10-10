package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Robot;

public class SimulatedCommand extends WrapperCommand {
    public SimulatedCommand(Command command, double simulatedTime) {
        super(
            Robot.isReal() ? command : command.withTimeout(simulatedTime)
        );
    }
}
