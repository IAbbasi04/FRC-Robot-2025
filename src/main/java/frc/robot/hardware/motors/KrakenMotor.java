package frc.robot.hardware.motors;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.hardware.ProfileGains;

public class KrakenMotor extends Motor {
    private TalonFX falconMotor;
    private TalonFXConfiguration configuration;

    private PositionVoltage positionOutput;
    private VelocityVoltage velocityOutput;

    public KrakenMotor(int id) {
        this(id, false);
    }

    public KrakenMotor(int id, boolean reversed) {
        this.falconMotor = new TalonFX(id);
        this.falconMotor.setInverted(reversed);

        this.configuration = new TalonFXConfiguration();
        this.configuration.MotorOutput.Inverted = reversed ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.falconMotor.getConfigurator().apply(configuration);

        this.positionOutput = new PositionVoltage(0.0);
        this.velocityOutput = new VelocityVoltage(0.0);
    }

    @Override
    public int getMotorID() {
        return this.falconMotor.getDeviceID();
    }

    @Override
    public void follow(Motor other, boolean reversed) {
        this.falconMotor.setControl(new Follower(other.getMotorID(), reversed));
    }

    @Override
    public void withGains(ProfileGains gains, int index) {
        switch (index) {
            case 0:
                Slot0Configs slot0Config = new Slot0Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD());

                this.falconMotor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = new Slot1Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD());

                this.falconMotor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = new Slot2Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD());

                this.falconMotor.getConfigurator().apply(slot2Config);
                break;
            default:
                SlotConfigs slotConfig = new SlotConfigs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD());

                this.falconMotor.getConfigurator().apply(slotConfig);
                break;
        }
    }

    @Override
    public double getVelocity() {
        return this.falconMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getPosition() {
        return this.falconMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setCurrentLimit(int amps) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = amps;
        currentConfigs.StatorCurrentLimitEnable = true;

        configuration.CurrentLimits = currentConfigs;
        falconMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond, int pidSlot) {
        falconMotor.setControl(velocityOutput.withVelocity(velocityMetersPerSecond));
    }

    @Override
    public void setProfiledVelocity(double velocityMetersPerSecond, int pidSlot) {
        setVelocity(velocityMetersPerSecond, pidSlot);
    }

    @Override
    public void setPosition(double positionRotations, int pidSlot) {
        falconMotor.setControl(positionOutput.withPosition(positionRotations));
    }
}