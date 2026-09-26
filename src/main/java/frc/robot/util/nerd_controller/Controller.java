// VERSION 3.1
// Uses the generic CommandGamepad class

package frc.robot.util.nerd_controller;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

import dev.doglog.DogLog;

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
    public Trigger buttonUp()           { return gamepad.faceUp();    }
    public Trigger buttonRight()        { return gamepad.faceRight();     }
    public Trigger buttonDown()         { return gamepad.faceDown();    }
    public Trigger buttonLeft()         { return gamepad.faceLeft();     }
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

    public boolean getTriggerLeft()     { return gamepad.getLeftTrigger() >= triggerDeadband;}
    public boolean getTriggerRight()    { return gamepad.getRightTrigger() >= triggerDeadband;}  

    public boolean getBumperLeft()      { return gamepad.getGamepad().getLeftBumperButton();  }
    public boolean getBumperRight()     { return gamepad.getGamepad().getRightBumperButton(); }    
    public boolean getButtonUp()        { return gamepad.getGamepad().getFaceUpButton();   }
    public boolean getButtonRight()     { return gamepad.getGamepad().getFaceRightButton();    }
    public boolean getButtonDown()      { return gamepad.getGamepad().getFaceDownButton();   }
    public boolean getButtonLeft()      { return gamepad.getGamepad().getFaceLeftButton();    }
    public boolean getDpadUp()          { return gamepad.getGamepad().getDpadUpButton();      }
    public boolean getDpadRight()       { return gamepad.getGamepad().getDpadRightButton();   }
    public boolean getDpadDown()        { return gamepad.getGamepad().getDpadDownButton();    }
    public boolean getDpadLeft()        { return gamepad.getGamepad().getDpadLeftButton();    }
    public boolean getJoystickLeft()    { return gamepad.getGamepad().getLeftStickButton();   }
    public boolean getJoystickRight()   { return gamepad.getGamepad().getRightStickButton();  }
    public boolean getControllerLeft()  { return gamepad.getGamepad().getBackButton();       } 
    public boolean getControllerRight() { return gamepad.getGamepad().getStartButton();       } 
    

    // ***** STATE METHODS ***** //

    /**
     * returns analog value from left trigger
     * @return value on [0, 1]
     */
    public double getTriggerLeftAxis() {
        return gamepad.getLeftTrigger();
    }

    /**
     * returns analog value from right trigger
     * @return value on [0, 1]
     */
    public double getTriggerRightAxis() {
        return gamepad.getRightTrigger();
    }

    // call once
    public static void configureDebugBindings(Controller testController) {
        testController.buttonRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Right Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Right Test", "bye")));
        testController.buttonDown()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Down Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Down Test", "bye")));
        testController.buttonUp()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Up Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Up Test", "bye")));
        testController.buttonLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Left Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Left Test", "bye")));

        testController.bumperLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Bumper L Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Bumper L Test", "bye")));
        testController.bumperRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Bumper R Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Bumper R Test", "bye")));
        
        testController.triggerLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Trigger L Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Trigger L Test", "bye")));
        testController.triggerRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Trigger R Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Trigger R Test", "bye")));

        testController.dpadUp()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Up Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Up Test", "bye")));
        testController.dpadRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Right Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Right Test", "bye")));
        testController.dpadDown()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Down Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Down Test", "bye")));
        testController.dpadLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Left Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Dpad Left Test", "bye")));

        testController.controllerLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Controller Left Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Controller Left Test", "bye")));
        testController.controllerRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Controller Right Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Controller Right Test", "bye")));
        
        testController.joystickLeft()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Left Joy Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Left Joy Test", "bye")));
        testController.joystickRight()
            .onTrue(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Right Joy Test", "hi")))
            .onFalse(Commands.runOnce(() -> DogLog.log("ControllerTest/Button Right Joy Test", "bye")));
    }

    // call repeatedly
    public void logAnalogValues() {
        DogLog.log("ControllerTest/Controller Joy Left X", getLeftX());
        DogLog.log("ControllerTest/Controller Joy Left Y", getLeftY());
        DogLog.log("ControllerTest/Controller Joy Right X", getRightX());
        DogLog.log("ControllerTest/Controller Joy Right Y", getRightY());

        DogLog.log("ControllerTest/Controller Left Trigger", getTriggerLeftAxis());
        DogLog.log("ControllerTest/Controller Right Trigger", getTriggerRightAxis());

        DogLog.log("ControllerTest/Right Button", getControllerRight());
    }
}
