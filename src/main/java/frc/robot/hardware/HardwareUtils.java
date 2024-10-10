package frc.robot.hardware;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class HardwareUtils {
    public static void setPIDGains(CANSparkMax motor, ProfileGains gains) {
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

    public static void setPIDGains(CANSparkFlex motor, ProfileGains gains) {
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
}