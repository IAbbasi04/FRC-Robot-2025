package frc.robot.common.suppliers;

import java.util.function.DoubleSupplier;

public class ChassisSpeedsSupplier {
    public DoubleSupplier vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond;

    public ChassisSpeedsSupplier(DoubleSupplier p_vx, DoubleSupplier p_vy, DoubleSupplier p_omega) {
        vxMetersPerSecond = p_vx;
        vyMetersPerSecond = p_vy;
        omegaRadiansPerSecond = p_omega;
    }
}