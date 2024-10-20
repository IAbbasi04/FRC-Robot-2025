package frc.robot.subsystems.test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.motors.VortexMotor;
import frc.robot.subsystems.SimpleRollerSubsystem;

public class TestSubsystem extends SimpleRollerSubsystem<VortexMotor> {
    private static TestSubsystem INSTANCE = null;
    public static TestSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new TestSubsystem();
        return INSTANCE;
    }

    private TestSubsystem() {
        super.m_motor = new VortexMotor(0);
    }

    public Command setRollerVelocity(DoubleSupplier desiredRPM) {
        return this.runOnce(() -> {
            this.setDesiredVelocity(desiredRPM.getAsDouble());
        });
    }

    @Override
    public void periodicLogs() {
        m_logger.logDouble("Desired Velocity RPM", m_desiredVelocityRPM);
    }
}