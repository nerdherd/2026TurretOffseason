package frc.robot.util.nerd_constants;

import org.wpilib.math.controller.PIDController;

public record NerdPIDConstants(
    double kP, double kI, double kD, 
    double kV, double kS, double kA, double kG, 
    boolean continuousInputEnabled, double minimumInput, double maximumInput) {

    public NerdPIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0);
    }

    public NerdPIDConstants enableContinuousInput(double minimumInput, double maximumInput) {
        return new NerdPIDConstants(kP, kI, kD, kV, kS, kA, kG, true, minimumInput, maximumInput);
    }

    public PIDController getController() {
        PIDController controller = new PIDController(kP, kI, kD);
        if (continuousInputEnabled) controller.enableContinuousInput(minimumInput, maximumInput);
        return controller;
    }
}