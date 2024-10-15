package frc.robot.common.suppliers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Suppliers {
    public static BooleanSupplier isRed = () -> DriverStation.getAlliance().isPresent() && 
                                                DriverStation.getAlliance().get() == Alliance.Red;

    public static DoubleSupplier distanceToSpeaker = () -> VisionSubsystem.getInstance().getDistanceToSpeaker();
}