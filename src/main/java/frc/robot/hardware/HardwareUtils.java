package frc.robot.hardware;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class HardwareUtils {
    public static void applyPIDGains(CANSparkMax motor, ProfileGains gains) {
        motor.getEncoder().setPositionConversionFactor(gains.getScalingFactor());
        motor.getEncoder().setVelocityConversionFactor(gains.getScalingFactor());
        
        SparkPIDController neoCtrl = motor.getPIDController();
        neoCtrl.setP(gains.getP());
        neoCtrl.setI(gains.getI());
        neoCtrl.setD(gains.getD());
        neoCtrl.setFF(gains.getFF());
        neoCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, gains.getSlot());
        neoCtrl.setSmartMotionMaxAccel(gains.getMaxAccel(), gains.getSlot());
        neoCtrl.setSmartMotionMaxVelocity(gains.getMaxVelocity(), gains.getSlot());

        if (gains.hasSoftLimits()) {
            motor.setSoftLimit(SoftLimitDirection.kForward, gains.getSoftLimits()[1]);
            motor.setSoftLimit(SoftLimitDirection.kForward, gains.getSoftLimits()[0]);
        }

        if (gains.getCurrentLimit() > 0) {
            motor.setSmartCurrentLimit(gains.getCurrentLimit());
        }
    }

    public static void applyPIDGains(CANSparkFlex motor, ProfileGains gains) {
        motor.getEncoder().setPositionConversionFactor(gains.getScalingFactor());
        motor.getEncoder().setVelocityConversionFactor(gains.getScalingFactor());
        
        SparkPIDController vortexCtrl = motor.getPIDController();
        vortexCtrl.setP(gains.getP());
        vortexCtrl.setI(gains.getI());
        vortexCtrl.setD(gains.getD());
        vortexCtrl.setFF(gains.getFF());
        vortexCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, gains.getSlot());
        vortexCtrl.setSmartMotionMaxAccel(gains.getMaxAccel(), gains.getSlot());
        vortexCtrl.setSmartMotionMaxVelocity(gains.getMaxVelocity(), gains.getSlot());

        if (gains.hasSoftLimits()) {
            motor.setSoftLimit(SoftLimitDirection.kForward, gains.getSoftLimits()[1]);
            motor.setSoftLimit(SoftLimitDirection.kForward, gains.getSoftLimits()[0]);
        }

        if (gains.getCurrentLimit() > 0) {
            motor.setSmartCurrentLimit(gains.getCurrentLimit(), gains.getCurrentLimit());
        }
    }

    public static Slot0Configs applyPIDGains(ProfileGains gains) {
        return new Slot0Configs()
            .withKP(gains.getP())
            .withKI(gains.getI())
            .withKD(gains.getD())
            .withKG(gains.getG())
            .withKS(gains.getG())
            .withKA(gains.getG())
            .withKV(gains.getG())
        ;
    }
}