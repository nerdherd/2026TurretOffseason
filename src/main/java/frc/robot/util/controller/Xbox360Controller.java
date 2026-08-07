// VERSION 1.0

package frc.robot.util.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Xbox360Controller {
    private final Joystick controller;
    private final JoystickButton cross, circle, square, triangle,
                                 bumper1L, bumper1R,
                                 share, options,
                                 leftJoy, rightJoy;

    /**
     * Creates an instance of an Xbox360 controller.
     * @param port The port of the controller on FRC Driver Station
     */
    public Xbox360Controller(int port) {
        controller = new Joystick(port);

        cross       = new JoystickButton(controller, 1);
        circle      = new JoystickButton(controller, 2);
        square      = new JoystickButton(controller, 3);
        triangle    = new JoystickButton(controller, 4);
        bumper1L    = new JoystickButton(controller, 5);
        bumper1R    = new JoystickButton(controller, 6);
        share       = new JoystickButton(controller, 7);
        options     = new JoystickButton(controller, 8);
        leftJoy     = new JoystickButton(controller, 9);
        rightJoy    = new JoystickButton(controller, 10);
    }

    // VALUE METHODS \\
    public double getLeftX()  { return controller.getRawAxis(0); }
    public double getLeftY()  { return controller.getRawAxis(1); }
    public double getRightX() { return controller.getRawAxis(4); }
    public double getRightY() { return controller.getRawAxis(5); }

    public boolean getCross()    { return cross.getAsBoolean(); }
    public boolean getCircle()   { return circle.getAsBoolean(); }
    public boolean getTriangle() { return triangle.getAsBoolean(); } 
    public boolean getSquare()   { return square.getAsBoolean(); }
    public boolean get1L() { return bumper1L.getAsBoolean(); }
    public boolean get1R() { return bumper1R.getAsBoolean(); }
    public boolean get2L() { return controller.getRawAxis(2) > 0.65; }
    public boolean get2R() { return controller.getRawAxis(3) > 0.65; }
    public double get2Laxis() { return controller.getRawAxis(2); }
    public double get2Raxis() { return controller.getRawAxis(3); }

    public boolean getShare()    { return share.getAsBoolean(); }
    public boolean getOptions()  { return options.getAsBoolean(); }
    public boolean getLeftJoy()  { return leftJoy.getAsBoolean(); }
    public boolean getRightJoy() { return rightJoy.getAsBoolean(); }

    public boolean getDpadUp()    { return (controller.getPOV() >= 300 || controller.getPOV() <= 60) && controller.getPOV() != -1; }
    public boolean getDpadRight() { return controller.getPOV() >= 30 && controller.getPOV() <= 150; }
    public boolean getDpadDown()  { return controller.getPOV() >= 120 && controller.getPOV() <= 240; }
    public boolean getDpadLeft()  { return controller.getPOV() >= 210 && controller.getPOV() <= 330; }

    // OBJECT METHODS \\
    public JoystickButton cross()    { return cross; }
    public JoystickButton circle()   { return circle; }
    public JoystickButton square()   { return square; } 
    public JoystickButton triangle() { return triangle; }
    public JoystickButton bumper1L() { return bumper1L; }
    public JoystickButton bumper1R() { return bumper1R; }
    public Trigger trigger2L() { return new Trigger(this::get2L); }
    public Trigger trigger2R() { return new Trigger(this::get2R); }

    public JoystickButton share()   { return share; }
    public JoystickButton options() { return options; }
    public JoystickButton leftJoy()  { return leftJoy; }
    public JoystickButton rightJoy() { return rightJoy; }

    public Trigger dpadUp()    { return new Trigger(this::getDpadUp); }
    public Trigger dpadRight() { return new Trigger(this::getDpadRight); }
    public Trigger dpadDown()  { return new Trigger(this::getDpadDown); }
    public Trigger dpadLeft()  { return new Trigger(this::getDpadLeft); }
}
