package frc.robot.common.swerve.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class CTRESwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;

    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    private BaseStatusSignal[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    private PositionVoltage m_angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    private SwerveModuleConstants m_constants;

    public CTRESwerveModule(SwerveModuleConstants constants, String canbusName) {
        m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
        m_steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
        m_cancoder = new CANcoder(constants.CANcoderId, canbusName);

        this.m_constants = constants;

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        m_driveMotor.getConfigurator().apply(talonConfigs);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true; // Enable continuous wrap for swerve modules

        talonConfigs.MotorOutput.Inverted =
                constants.SteerMotorReversed
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        m_steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        m_cancoder.getConfigurator().apply(cancoderConfigs);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_cancoder.getPosition();
        m_steerVelocity = m_cancoder.getVelocity();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public CTRESwerveModule(SwerveModuleConstants constants) {
        m_driveMotor = new TalonFX(constants.DriveMotorId);
        m_steerMotor = new TalonFX(constants.SteerMotorId);
        m_cancoder = new CANcoder(constants.CANcoderId);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        m_driveMotor.getConfigurator().apply(talonConfigs);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true; // Enable continuous wrap for swerve modules

        talonConfigs.MotorOutput.Inverted =
                constants.SteerMotorReversed
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        m_steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        m_cancoder.getConfigurator().apply(cancoderConfigs);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_cancoder.getPosition();
        m_steerVelocity = m_cancoder.getVelocity();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot =
            BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            m_driveMotor.getVelocity().getValueAsDouble() * this.m_constants.WheelRadius,
            m_internalState.angle
        );
    }

    public void setThrottleCurrentLimit(double currentLimit) {

    }

    public void apply(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    BaseStatusSignal[] getSignals() {
        return m_signals;
    }
}