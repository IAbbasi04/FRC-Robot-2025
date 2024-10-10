package frc.robot.commands;

import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class IntakeCommand extends NewtonCommand {
    public IntakeCommand() {
        super(
            // Run the intake rollers until the feeder has the note
            m_intake.getCommands().setIntaking()
                .alongWith(m_feeder.getCommands().setFeederState(FeederState.kIntake))
                .until(() -> m_feeder.hasNote())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        addRequirements(m_intake, m_feeder); // Do not do anything else with feeder and intake
    }
}