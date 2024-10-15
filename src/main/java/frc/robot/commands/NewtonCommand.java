package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NewtonCommand extends WrapperCommand {
    protected static SwerveSubsystem m_swerve = SwerveSubsystem.getInstance();
    protected static ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
    protected static IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
    protected static ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
    protected static FeederSubsystem m_feeder = FeederSubsystem.getInstance();
    protected static LEDSubsystem m_led = LEDSubsystem.getInstance();

    protected NewtonCommand(Command command) {
        super(command);
    }
}