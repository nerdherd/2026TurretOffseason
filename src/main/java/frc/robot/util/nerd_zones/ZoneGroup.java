package frc.robot.util.nerd_zones;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;

public class ZoneGroup implements NerdZone {    
    private final List<NerdZone> zones = new ArrayList<NerdZone>();
    private final List<NerdZone> exclusion = new ArrayList<NerdZone>();

    public ZoneGroup() {}

    public ZoneGroup(NerdZone... zones) {
        this.zones.addAll(Arrays.asList(zones));
    }

    public ZoneGroup addZone(NerdZone zone) {
        this.zones.add(zone);
        return this;
    }

    public ZoneGroup excludeZone(NerdZone zone) {
        this.exclusion.add(zone);
        return this;
    }

    @Override
    public boolean check(Pose2d robotPose) {
        for (NerdZone zone : exclusion) {
            if (zone.check(robotPose)) return false;
        }

        for (NerdZone zone : zones) {
            if (zone.check(robotPose)) {
                return true;
            }
        }

        return false;
        
    }
}
