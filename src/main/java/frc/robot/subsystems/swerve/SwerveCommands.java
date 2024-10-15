package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveCommands {
    private SwerveSubsystem m_swerve;

    public SwerveCommands(SwerveSubsystem swerve) {
        m_swerve = swerve;
    }

    /**
     * Drives the robot based on driver joystick control
     */
    public Command teleopDriveCommand(DoubleSupplier suppliedX, DoubleSupplier suppliedY, DoubleSupplier suppliedRot) {
        return m_swerve.run(() -> {
            m_swerve.driveRobotCentric(m_swerve.processInputs(
                suppliedX.getAsDouble(),
                suppliedY.getAsDouble(),
                suppliedRot.getAsDouble()
            ));
        });
    }

    public Command turnToPose(Pose2d pose) {
        return m_swerve.run(() -> {
            ChassisSpeeds currentSpeeds = m_swerve.getCurrentSpeeds();
            m_swerve.driveFieldCentric(new ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond,
                m_swerve.getRotationSpeedToTarget(pose)
            ));
        });
    }

    public Command turnToAngle(Rotation2d desiredRotation) {
        return m_swerve.run(() -> {
            ChassisSpeeds currentSpeeds = m_swerve.getCurrentSpeeds();
            m_swerve.driveFieldCentric(new ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond,
                currentSpeeds.vyMetersPerSecond,
                m_swerve.getRotationSpeedToRotation(desiredRotation)
            ));
        });
    }

    public Command setSnail(boolean snail) {
        return m_swerve.runOnce(() -> {
            m_swerve.setSnailMode(snail);
        });
    }
}