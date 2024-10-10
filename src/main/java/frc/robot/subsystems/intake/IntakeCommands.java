package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.Constants;

public class IntakeCommands {
    private IntakeSubsystem m_intake;

    public IntakeCommands(IntakeSubsystem intake) {
        m_intake = intake;
    }

    /**
     * Sets the rollers to the desired velocity in RPM
     */
    public Command setIntakeVelocity(DoubleSupplier velocityRPM) {
        return m_intake.run(() -> {
            m_intake.setDesiredVelocity(velocityRPM.getAsDouble());
        });
    }

    /**
     * Sets the rollers to the intaking velocity in RPM; turns off after interrupted
     */
    public Command setIntaking() {
        return m_intake.runEnd(
            () -> m_intake.setDesiredVelocity(Constants.INTAKE.ROLLER_INTAKE_RPM), 
            () -> m_intake.setDesiredVelocity(0.0)
        );
    }

    /**
     * Sets the rollers to the outaking velocity in RPM; turns off after interrupted
     */
    public Command setOutaking() {
        return m_intake.runEnd(
            () -> m_intake.setDesiredVelocity(Constants.INTAKE.ROLLER_OUTAKE_RPM), 
            () -> m_intake.setDesiredVelocity(0.0)
        );
    }

    /**
     * Turns off the roller motor (Sets to 0.0 RPM)
     */
    public Command setOff() {
        return this.setIntakeVelocity(() -> 0.0);
    }
}