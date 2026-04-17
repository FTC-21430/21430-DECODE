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
        FAR_3
    }

    private final Dictionary<POS,Waypoint> positions;

    public GlobalPositions(ALLIANCE alliance){
        targetAlliance = alliance;
        positions = new Hashtable<POS, Waypoint>();
        //TODO: These coordinates are approximate, please tune in - Tobin - (hiding in the tech booth while the show starts in 10 minutes
        positions.put(POS.CLOSE_1, new Waypoint(-0,0,150,1,true));
        positions.put(POS.CLOSE_2,new Waypoint(-12,12,155,1,true));
        positions.put(POS.CLOSE_3,new Waypoint(-24,24,155,1,true));
        positions.put(POS.END,new Waypoint(24,24,175,1,true));
        positions.put(POS.INTAKE_START_1,new Waypoint(-12,24,270,1,true));
        positions.put(POS.INTAKE_END_1,new Waypoint(-12,48,270,1,true));
        positions.put(POS.INTAKE_START_2,new Waypoint(12,  24,270,1,true));
        positions.put(POS.INTAKE_END_2,new Waypoint(-18,52,270,1,true));
        positions.put(POS.INTAKE_START_3,new Waypoint(36,18,270,1,true));
        positions.put(POS.INTAKE_END_3,new Waypoint(36,52,270,1,true));
        positions.put(POS.GATE_PREP,new Waypoint(-6,30,180,1,true));
        positions.put(POS.GATE_OPEN,new Waypoint(-6,46,180,1,true));
        positions.put(POS.INTAKE_START_CORNER,new Waypoint(56,36,270,1,true));
        positions.put(POS.INTAKE_END_CORNER,new Waypoint(56,52,270,1,true));
        positions.put(POS.FAR_1,new Waypoint(48,0,175,1,true));
        positions.put(POS.FAR_2,new Waypoint(52,8,178,1,true));
        positions.put(POS.FAR_3,new Waypoint(60,18,173,1,true));
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

            

        }
        return point;
    }
}
