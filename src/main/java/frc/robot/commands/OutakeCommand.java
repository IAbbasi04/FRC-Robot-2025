package frc.robot.commands;

import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class OutakeCommand extends NewtonCommand {
    public OutakeCommand() {
        super(
            // Run the intake rollers reversed while reversing the feeder
            m_intake.getCommands().setOutaking()
                .alongWith(m_feeder.getCommands().setFeederState(FeederState.kOutake))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .finallyDo(() -> {
                    m_intake.setDesiredVelocity(0.0);
                    m_feeder.setFeederState(FeederState.kOff);
                })
        );
    }
}