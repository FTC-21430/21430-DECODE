package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import java.util.Dictionary;
import java.util.Hashtable;

public class GlobalPositions {
    public static enum ALLIANCE{
        RED,
        BLUE
    }
    private ALLIANCE targetAlliance = ALLIANCE.RED;
    public static enum POS{
        CLOSE_3,
        CLOSE_2,
        CLOSE_1,
        END,
        INTAKE_START_1,
        INTAKE_END_1,
        INTAKE_START_2,
        INTAKE_END_2,
        INTAKE_START_3,
        INTAKE_END_3,
        GATE_PREP,
        GATE_OPEN,
        INTAKE_START_CORNER,
        INTAKE_END_CORNER,
        FAR_1,
        FAR_2,
        FAR_3,
        CLOSE_START,
        FAR_START,
        FAR_END
    }

    private final Dictionary<POS,Waypoint> positions;

    public GlobalPositions(ALLIANCE alliance){
        targetAlliance = alliance;
        positions = new Hashtable<POS, Waypoint>();
        //TODO: These coordinates are approximate, please tune in - Tobin - (hiding in the tech booth while the show starts in 10 minutes
        positions.put(POS.CLOSE_1, new Waypoint(-0,0,150,1,true));
        positions.put(POS.CLOSE_2,new Waypoint(-12,12,155,1,true));
        positions.put(POS.CLOSE_3,new Waypoint(-30,30,155,1,true));
        positions.put(POS.END,new Waypoint(24,34,175,1,true));
        positions.put(POS.INTAKE_START_1,new Waypoint(-13.3, 20, 270,1,true));
        positions.put(POS.INTAKE_END_1,new Waypoint(-13.3, 49.5, 270,1,true));
        positions.put(POS.INTAKE_START_2,new Waypoint(12.5, 20, 270,1,true));
        positions.put(POS.INTAKE_END_2,new Waypoint(12.5, 58.5, 270,1,true));
        positions.put(POS.INTAKE_START_3,new Waypoint(39, 20, 270,1,true));
        positions.put(POS.INTAKE_END_3,new Waypoint(39, 58.5, 270,1,true));
        positions.put(POS.GATE_PREP,new Waypoint(-6, 36.5, 180,1,true));
        positions.put(POS.GATE_OPEN,new Waypoint(-5.5, 52, 180,1,true));
        positions.put(POS.INTAKE_START_CORNER,new Waypoint(56,36,270,1,true));
        positions.put(POS.INTAKE_END_CORNER,new Waypoint(56,52,270,1,true));
        positions.put(POS.FAR_1,new Waypoint(48,0,175,1,true));
        positions.put(POS.FAR_2,new Waypoint(52,8,178,1,true));
        positions.put(POS.FAR_3,new Waypoint(60,18,173,1,true));
        positions.put(POS.CLOSE_START, new Waypoint(-64.22,34.88,180,1,true));
        positions.put(POS.FAR_START, new Waypoint(63.5, 20.5, 0, 1, true));
        positions.put(POS.FAR_END,new Waypoint(68, 33.5, 0,1,true));
    }
    public void setAlliance(ALLIANCE alliance){
        this.targetAlliance = alliance;
    }
    private Waypoint MirrorWaypoint(Waypoint waypoint){
        return new Waypoint(waypoint.getX(),waypoint.getY() * -1, waypoint.getAngle() * -1 , waypoint.getSpeed(), true);
    }

    public Waypoint get(POS position){
        Waypoint point = positions.get(position);
        switch (targetAlliance){
            case RED:
                // Points are default
                break;
            case BLUE:
                point = MirrorWaypoint(point);
                break;
        }
        return point;
    }
}
