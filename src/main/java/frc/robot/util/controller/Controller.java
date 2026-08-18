// VERSION 3.0
// Uses the generic CommandGamepad class

package frc.robot.util.controller;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

// thank you william
// you're welcome mason
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
    public Trigger controllerLeft()     { return gamepad.guide();        } // TODO: this is probably wrong
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
    public boolean getControllerLeft()  { return gamepad.getHID().getGuideButton();       } // TODO: this is probably wrong
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
}
