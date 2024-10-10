package frc.robot.common.crescendo.tables;

import frc.robot.common.crescendo.ShotProfile;
import frc.robot.common.regression.PiecewiseRegression;
import frc.robot.common.regression.Regression;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static frc.robot.common.crescendo.Global.*;

public class ShotTable {
    protected Regression leftShooterRPMRegression;
    protected Regression rightShooterRPMRegression;
    protected Regression elevatorPivotRegression;
    protected Regression elevatorExtensionRegression;

    public ShotTable() {
        this.leftShooterRPMRegression = new PiecewiseRegression(FLYHWEEL_RANGES[1], 0.2);
        this.rightShooterRPMRegression = new PiecewiseRegression(FLYHWEEL_RANGES[2], 0.2);
        this.elevatorPivotRegression = new PiecewiseRegression(UNDEFENDED_ELEVATOR_RANGES[1], 0.2);
        this.elevatorExtensionRegression = new PiecewiseRegression(UNDEFENDED_ELEVATOR_RANGES[2], 0.2);
    }

    /**
     * Uses interpolation tables to get the right shot from desired distances
     */
    public ShotProfile getShotFromDistance(double distance) {
        if (distance == -1) { // Speaker tag not visible
            return new ShotProfile().shouldShoot(false);
        }

        double leftRPM = leftShooterRPMRegression.grabInterpolation(distance);
        double rightRPM = rightShooterRPMRegression.grabInterpolation(distance);
        double pivot = elevatorPivotRegression.grabInterpolation(distance);
        double extension = elevatorExtensionRegression.grabInterpolation(distance);

        return new ShotProfile()
            .flywheel(leftRPM, rightRPM)
            .pivot(pivot)
            .extension(extension)
            .shouldShoot(true)
        ;
    }

    /**
     * Shoots the note at whatever angle it currently is
     */
    public ShotProfile getStaticShot() {
        double shooterRPM = 4000;
        double extension = ElevatorSubsystem.getInstance().getElevatorExtensionMeters();
        double pivot = ElevatorSubsystem.getInstance().getPivotAngleDegrees();
        return new ShotProfile()
            .flywheel(shooterRPM, shooterRPM)
            .pivot(pivot)
            .extension(extension)
            .shouldShoot(true)
        ;
    }

    /**
     * Statically set shot from subwoofer range
     */
    public ShotProfile getSubwooferShot() {
        return getShotFromDistance(1.4);
    }

    /**
     * Statically set shot from podium range
     */
    public ShotProfile getPodiumShot() {
        return getShotFromDistance(2.83);
    }

    /**
     * Static Shot profile for passes
     */
    public ShotProfile getPassShot() {
        return new ShotProfile()
            .flywheel(4500, 5000)
            .pivot(45)
            .shouldShoot(true)
        ;
    }
}