// VERSION 3.1
// Uses the generic CommandGamepad class

package frc.robot.util.controller;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

import frc.robot.util.logging.NerdLog;

// thank you william
// you're welcome mason

// im wheler and i love duenas
// im also here hello
public class Controller {
    private final CommandGamepad gamepad;
    private static final double triggerDeadband = 0.65;


    /**
     * Constructor for controller.<p>
     * @param port in Driver Station
     */
    public Controller(int port) {
        gamepad = new CommandGamepad(port);
    }


    // ***** BUTTON METHODS ***** //

    public Trigger triggerLeft()        { return gamepad.leftTrigger();  }
    public Trigger triggerRight()       { return gamepad.rightTrigger(); }
    public Trigger bumperLeft()         { return gamepad.leftBumper();   }
    public Trigger bumperRight()        { return gamepad.rightBumper();  }
    public Trigger buttonUp()           { return gamepad.northFace();    }
    public Trigger buttonRight()        { return gamepad.eastFace();     }
    public Trigger buttonDown()         { return gamepad.southFace();    }
    public Trigger buttonLeft()         { return gamepad.westFace();     }
    public Trigger dpadUp()             { return gamepad.dpadUp();       }
    public Trigger dpadRight()          { return gamepad.dpadRight();    }
    public Trigger dpadDown()           { return gamepad.dpadDown();     }
    public Trigger dpadLeft()           { return gamepad.dpadLeft();     }
    public Trigger joystickLeft()       { return gamepad.leftStick();    }
    public Trigger joystickRight()      { return gamepad.rightStick();   }
    public Trigger controllerLeft()     { return gamepad.back();        } // TODO: this is probably wrong
    public Trigger controllerRight()    { return gamepad.start();        } // TODO: this is probably wrong

    public double getLeftX()            { return gamepad.getLeftX();  }
    public double getLeftY()            { return gamepad.getLeftY();  }
    public double getRightX()           { return gamepad.getRightX(); }
    public double getRightY()           { return gamepad.getRightY(); }

    public boolean getTriggerLeft()     { return gamepad.getLeftTriggerAxis() >= triggerDeadband;}
    public boolean getTriggerRight()    { return gamepad.getRightTriggerAxis() >= triggerDeadband;}  

    public boolean getBumperLeft()      { return gamepad.getHID().getLeftBumperButton();  }
    public boolean getBumperRight()     { return gamepad.getHID().getRightBumperButton(); }    
    public boolean getButtonUp()        { return gamepad.getHID().getNorthFaceButton();   }
    public boolean getButtonRight()     { return gamepad.getHID().getEastFaceButton();    }
    public boolean getButtonDown()      { return gamepad.getHID().getSouthFaceButton();   }
    public boolean getButtonLeft()      { return gamepad.getHID().getWestFaceButton();    }
    public boolean getDpadUp()          { return gamepad.getHID().getDpadUpButton();      }
    public boolean getDpadRight()       { return gamepad.getHID().getDpadRightButton();   }
    public boolean getDpadDown()        { return gamepad.getHID().getDpadDownButton();    }
    public boolean getDpadLeft()        { return gamepad.getHID().getDpadLeftButton();    }
    public boolean getJoystickLeft()    { return gamepad.getHID().getLeftStickButton();   }
    public boolean getJoystickRight()   { return gamepad.getHID().getRightStickButton();  }
    public boolean getControllerLeft()  { return gamepad.getHID().getBackButton();       } // TODO: this is probably wrong
    public boolean getControllerRight() { return gamepad.getHID().getStartButton();       } // TODO: this is probably wrong
    

    // ***** STATE METHODS ***** //

    /**
     * returns analog value from left trigger
     * @return value on [0, 1]
     */
    public double getTriggerLeftAxis() {
        return gamepad.getLeftTriggerAxis();
    }

    /**
     * returns analog value from right trigger
     * @return value on [0, 1]
     */
    public double getTriggerRightAxis() {
        return gamepad.getRightTriggerAxis();
    }

    public static void configureDebugBindings(Controller testController) {
        testController.buttonRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Right Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Right Test")));
        testController.buttonDown()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Down Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Down Test")));
        testController.buttonUp()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Up Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Up Test")));
        testController.buttonLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Left Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Left Test")));

        testController.bumperLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Bumper L Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Bumper L Test")));
        testController.bumperRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Bumper R Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Bumper R Test")));
        
        testController.triggerLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Trigger L Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Trigger L Test")));
        testController.triggerRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Trigger R Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Trigger R Test")));

        testController.dpadUp()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Up Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Up Test")));
        testController.dpadRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Right Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Right Test")));
        testController.dpadDown()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Down Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Down Test")));
        testController.dpadLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Left Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Dpad Left Test")));

        testController.controllerLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Controller Left Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Controller Left Test")));
        testController.controllerRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Controller Right Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Controller Right Test")));
        
        testController.joystickLeft()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Left Joy Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Left Joy Test")));
        testController.joystickRight()
            .onTrue(Commands.runOnce(() -> NerdLog.reportInfo("Button Right Joy Test")))
            .onFalse(Commands.runOnce(() -> NerdLog.reportInfo("Button Right Joy Test")));
    }

    public void logAnalogValues() {
        // NerdLog.logNumber("Controller Joy Left X", getLeftX());
        // NerdLog.logNumber("Controller Joy Left Y", getLeftY());
        // NerdLog.logNumber("Controller Joy Right X", getRightX());
        // NerdLog.logNumber("Controller Joy Right Y", getRightY());

        // SmartDashboard.putNumber("Controller Left Trigger", getTriggerLeftAxis());
        // SmartDashboard.putNumber("Controller Right Trigger", getTriggerRightAxis());

        // SmartDashboard.putBoolean("Right Button", getControllerRight());
    }
}
