package frc.robot.common.suppliers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dSupplier {
    private DoubleSupplier m_rotAsDegrees;

    public Rotation2dSupplier(DoubleSupplier p_rotAsDegrees) {
        this.m_rotAsDegrees = p_rotAsDegrees;
    }

    public Rotation2dSupplier fromDegrees(DoubleSupplier m_degrees) {
        this.m_rotAsDegrees = m_degrees;
        return this;
    }

    public Rotation2dSupplier fromRadians(DoubleSupplier m_radians) {
        this.m_rotAsDegrees = () -> (m_radians.getAsDouble() * 180  / Math.PI);
        return this;
    }

    public Rotation2d getAsRotation2d() {
        return Rotation2d.fromDegrees(m_rotAsDegrees.getAsDouble());
    }
}