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
        positions.put(POS.CLOSE_1, new Waypoint(2,1,150,1,true));
        positions.put(POS.CLOSE_2,new Waypoint(-10,18,155,1,true));
        positions.put(POS.CLOSE_3,new Waypoint(-28,31,155,1,true));
        positions.put(POS.INTAKE_START_1,new Waypoint(-11.3, 18, 270,1,true));
        positions.put(POS.INTAKE_END_1,new Waypoint(-11.3, 50.6, 270,1,true));
        positions.put(POS.INTAKE_START_2,new Waypoint(16.5, 18, 270,1,true));
        positions.put(POS.INTAKE_END_2,new Waypoint(16.5, 58, 270,1,true));
        positions.put(POS.INTAKE_START_3,new Waypoint(39, 18, 270,1,true));
        positions.put(POS.INTAKE_END_3,new Waypoint(39, 58, 270,1,true));
        positions.put(POS.GATE_PREP,new Waypoint(-6, 34.5, 180,1,true));
        positions.put(POS.GATE_OPEN,new Waypoint(-4.5, 53.3, 180,1,true));
        positions.put(POS.INTAKE_START_CORNER,new Waypoint(58,36,270,1,true));
        positions.put(POS.INTAKE_END_CORNER,new Waypoint(64.5,64,270,1,true));
        positions.put(POS.FAR_1,new Waypoint(48,0,175,1,true));
        positions.put(POS.FAR_2,new Waypoint(52,8,178,1,true));
        positions.put(POS.FAR_3,new Waypoint(58,18,173,1,true));
        positions.put(POS.CLOSE_START, new Waypoint(-62.22,38.5,180,1,true));
        positions.put(POS.FAR_START, new Waypoint(59.466, 13.50, 180, 1, true));
        positions.put(POS.FAR_END, new Waypoint(60.5, 40, 180,1,true));
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
