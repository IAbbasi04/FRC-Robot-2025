package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.common.crescendo.ShotProfile;

public class ShooterCommands {
    private ShooterSubsystem m_shooter;

    public ShooterCommands(ShooterSubsystem shooter) {
        m_shooter = shooter;
    }

    public Command setShooting(DoubleSupplier leftRPM, DoubleSupplier rightRPM) {
        return m_shooter.runEnd(() -> {
            m_shooter.setDesiredVelocity(leftRPM.getAsDouble(), rightRPM.getAsDouble());
        }, () -> {
            this.setOff();
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command setShooting(ShotProfile desiredShot) {
        return this.setShooting(
            () -> desiredShot.leftShotRPM, 
            () -> desiredShot.rightShotRPM
        );
    }

    public Command setOff() {
        return m_shooter.runOnce(() -> {
            m_shooter.setDesiredVelocity(0.0, 0.0);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}