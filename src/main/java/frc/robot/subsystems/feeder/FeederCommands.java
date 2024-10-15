package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;

public class FeederCommands {
    private FeederSubsystem m_feeder;
    public FeederCommands(FeederSubsystem feeder) {
        this.m_feeder = feeder;
    }

    public Command setFeederState(FeederState state) {
        return m_feeder.run(() -> {
            m_feeder.setFeederState(state);
        });
    }
}