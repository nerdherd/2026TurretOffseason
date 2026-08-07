package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class BannerSensor {
    private final DigitalInput bannerSensorBlack;
    private final DigitalInput bannerSensorWhite;

    private boolean detected;
    private boolean lastBlackValue;
    private boolean lastWhiteValue;
    private boolean illegalInput = false;

    public BannerSensor(int blackPort, int whitePort) {
        bannerSensorBlack = new DigitalInput(blackPort);
        bannerSensorWhite = new DigitalInput(whitePort);
    }

    public boolean pollSensor() {
        lastBlackValue = bannerSensorBlack.get();
        lastWhiteValue = bannerSensorWhite.get();
        if ((lastBlackValue && lastWhiteValue) || (!lastBlackValue && !lastWhiteValue)) {
            illegalInput = true;
            detected = false;
        }

        if(!lastBlackValue && lastWhiteValue){
            detected = true;
        }
        else if(lastBlackValue && !lastWhiteValue){
            detected = false;
        }
        else{
            DriverStation.reportError("Fault in banner sensor, error code: ", true);
            detected = false;
        }
        return detected;
    }

    public boolean pieceDetectedWithoutPolling() {
        return detected;
    }

    public boolean isSensorConnected() {
        return !illegalInput;
    }

    public boolean getLastBlack() {
        return lastBlackValue;
    }

    public boolean getLastWhite() {
        return lastWhiteValue;
    }
}