package org.firstinspires.ftc.teamcode.Resources.SWEEP;

public class Waypoint {
    private final double x,y,angle,velocity,duration;
    private final boolean shouldHoldAngle,isWaitPoint;
    public Waypoint(double x, double y, double angle, double velocity, boolean shouldHoldAngle){
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velocity = velocity;
        this.duration = 0;
        this.shouldHoldAngle = shouldHoldAngle;
        isWaitPoint = false;
    }
    public Waypoint(double x, double y, double angle, double duration){
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velocity = 0;
        this.duration = duration;
        this.shouldHoldAngle = true;
        isWaitPoint = true;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getAngle(){
        return angle;
    }
    public double getSpeed(){
        return velocity;
    }
    public double getDuration(){return duration;}
    public boolean shouldHoldAngle(){
        return shouldHoldAngle;
    }
    public boolean isWaitPoint(){return isWaitPoint;}
}
