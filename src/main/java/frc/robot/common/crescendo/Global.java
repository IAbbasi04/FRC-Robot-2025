package frc.robot.common.crescendo;

public class Global {
    public static final double[][] FLYHWEEL_RANGES = new double[][]{
        new double[] {0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0}, // distances
        new double[] {3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 4000, 4000, 4000, 4500, 4500, 4500, 4500, 5000, 5500, 5500}, // left flywheel
        new double[] {2700, 2700, 2700, 2700, 2700, 2700, 2700, 2700, 2700, 2700, 2700, 3000, 3000, 3000, 3500, 3500, 3500, 3500, 3500, 3500, 3500} // right flywheel
    };

    public static final double[][] UNDEFENDED_ELEVATOR_RANGES = new double[][]{
        new double[] {0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0}, // distances
        new double[] {0, 0, 0, 0, 0, 0, 0, 0, 6, 9, 13, 18, 19, 22, 27.5, 27.5, 27.5, 29, 31, 32, 33}, // pivot
        new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // extension
    };
}