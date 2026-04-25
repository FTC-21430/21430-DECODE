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
        FAR_END,
        GATE_COLLECT
    }

    private final Dictionary<POS,Waypoint> positions;
    private final Dictionary<POS,Waypoint> bluePositions;

    public GlobalPositions(ALLIANCE alliance){
        targetAlliance = alliance;
        positions = new Hashtable<POS, Waypoint>();
        bluePositions = new Hashtable<POS, Waypoint>();

        positions.put(POS.CLOSE_1, new Waypoint(2,1,150,1,true));
        positions.put(POS.CLOSE_2,new Waypoint(-10,18,155,1,true));
        positions.put(POS.CLOSE_3,new Waypoint(-28,25,155,1,true));
        positions.put(POS.INTAKE_START_1,new Waypoint(-11.3, 18, 270,1,true));
        positions.put(POS.INTAKE_END_1,new Waypoint(-11.3, 51.1, 270,1,true));
        positions.put(POS.INTAKE_START_2,new Waypoint(16.5, 18, 270,1,true));
        positions.put(POS.INTAKE_END_2,new Waypoint(16.5, 60, 270,1,true));
        positions.put(POS.INTAKE_START_3,new Waypoint(39, 18, 270,1,true));
        positions.put(POS.INTAKE_END_3,new Waypoint(39, 60, 270,1,true));
        positions.put(POS.GATE_PREP,new Waypoint(-6, 39.5, 180,1,true));
        positions.put(POS.GATE_OPEN,new Waypoint(-4.5, 55.3, 180,1,true));
        positions.put(POS.INTAKE_START_CORNER,new Waypoint(50,18,270,1,true));
        positions.put(POS.INTAKE_END_CORNER,new Waypoint(64.3,62.5,270,1,true));
        positions.put(POS.FAR_1,new Waypoint(48,-2,175,1,true));
        positions.put(POS.FAR_2,new Waypoint(52,6,178,1,true));
        positions.put(POS.FAR_3,new Waypoint(58,25,173,1,true));
        positions.put(POS.CLOSE_START, new Waypoint(-62.22,38.5,180,1,true));
        positions.put(POS.FAR_START, new Waypoint(63.5, 18.5, 180, 1, true));
        positions.put(POS.FAR_END, new Waypoint(60.5, 40, 180,1,true));
        positions.put(POS.GATE_COLLECT, new Waypoint(15, 59, -90, 1,true));


        bluePositions.put(POS.CLOSE_1, new Waypoint(2,-1,-150,1,true));
        bluePositions.put(POS.CLOSE_2,new Waypoint(-10,-18,-155,1,true));
        bluePositions.put(POS.CLOSE_3,new Waypoint(-28,-25,-155,1,true));
        bluePositions.put(POS.INTAKE_START_1,new Waypoint(-11.3, -18, -270,1,true));
        bluePositions.put(POS.INTAKE_END_1,new Waypoint(-11.3, -50.6, -270,1,true));
        bluePositions.put(POS.INTAKE_START_2,new Waypoint(16.5, -18, -270,1,true));
        bluePositions.put(POS.INTAKE_END_2,new Waypoint(16.5, -59, -270,1,true));
        bluePositions.put(POS.INTAKE_START_3,new Waypoint(35, -18, -270,1,true));
        bluePositions.put(POS.INTAKE_END_3,new Waypoint(35, -59, -270,1,true));
        bluePositions.put(POS.GATE_PREP,new Waypoint(-6, -34.5, -180,1,true));
        bluePositions.put(POS.GATE_OPEN,new Waypoint(-4.5, -57.5, -180,1,true));
        bluePositions.put(POS.INTAKE_START_CORNER,new Waypoint(47,-23,-270,1,true));
        bluePositions.put(POS.INTAKE_END_CORNER,new Waypoint(61.3,-64.5,-270,1,true));
        bluePositions.put(POS.FAR_1,new Waypoint(48,3,-175,1,true));
        bluePositions.put(POS.FAR_2,new Waypoint(52,-5,-178,1,true));
        bluePositions.put(POS.FAR_3,new Waypoint(54,-26,-173,1,true));
        bluePositions.put(POS.CLOSE_START, new Waypoint(-62.22,-38.5,-180,1,true));
        bluePositions.put(POS.FAR_START, new Waypoint(60.5, -21.5, -180, 1, true));
        bluePositions.put(POS.FAR_END, new Waypoint(60.5, -42, -180,1,true));
        bluePositions.put(POS.GATE_COLLECT, new Waypoint(15, -59, -90, 1,true));
    }
    public void setAlliance(ALLIANCE alliance){
        this.targetAlliance = alliance;
    }
    private Waypoint MirrorWaypoint(Waypoint waypoint){
        return new Waypoint(waypoint.getX(),waypoint.getY() * -1, waypoint.getAngle() * -1 , waypoint.getSpeed(), true);
    }

    public Waypoint get(POS position){
        Waypoint point;
        switch (targetAlliance){
            case RED:
                point = positions.get(position);
                break;
            case BLUE:
                point = bluePositions.get(position);
                break;
            default:
                point = positions.get(position);
        }
        return point;
    }
}
