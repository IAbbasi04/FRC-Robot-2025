package frc.robot.common.swerve.ctre;

import com.ctre.phoenix6.configs.Slot0Configs;

public class SwerveConstantsCreator {
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double SteerMotorGearRatio;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius;
    /** The maximum amount of current the drive motors can apply without slippage */
    public double SlipCurrent = 400;

    /** The steer motor gains */
    public Slot0Configs SteerMotorGains;
    /** The drive motor gains */
    public Slot0Configs DriveMotorGains;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorReversed;

    public SwerveConstantsCreator(
            double swerveModuleDriveRatio,
            double swerveModuleSteerRatio,
            double swerveModuleWheelRadius,
            double swerveModuleSlipCurrent,
            Slot0Configs swerveModuleSteerGains,
            Slot0Configs swerveModuleDriveGains,
            boolean SteerMotorReversed) {
        this.DriveMotorGearRatio = swerveModuleDriveRatio;
        this.SteerMotorGearRatio = swerveModuleSteerRatio;
        this.WheelRadius = swerveModuleWheelRadius;
        this.SlipCurrent = swerveModuleSlipCurrent;

        this.SteerMotorGains = swerveModuleSteerGains;
        this.DriveMotorGains = swerveModuleDriveGains;
        this.SteerMotorReversed = SteerMotorReversed;
    }

    public SwerveModuleConstants createModuleConstants(
            int steerId,
            int driveId,
            int cancoderId,
            double cancoderOffset,
            double locationX,
            double locationY) {
        return new SwerveModuleConstants()
                .withSteerMotorId(steerId)
                .withDriveMotorId(driveId)
                .withCANcoderId(cancoderId)
                .withCANcoderOffset(cancoderOffset)
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorGearRatio(DriveMotorGearRatio)
                .withSteerMotorGearRatio(SteerMotorGearRatio)
                .withWheelRadius(WheelRadius)
                .withSlipCurrent(SlipCurrent)
                .withSteerMotorGains(SteerMotorGains)
                .withDriveMotorGains(DriveMotorGains)
                .withSteerMotorReversed(SteerMotorReversed);
    }
}