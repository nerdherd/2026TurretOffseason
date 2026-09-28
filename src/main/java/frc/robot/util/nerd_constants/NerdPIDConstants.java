package frc.robot.util.nerd_constants;

import org.wpilib.math.controller.PIDController;

public record NerdPIDConstants(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
    public NerdPIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
    }

    public PIDController getController() {
        return new PIDController(kP, kI, kD);
    }
}