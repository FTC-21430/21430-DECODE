package org.firstinspires.ftc.teamcode.Resources;

/**
 * Immutable data packet representing a single odometry sample.
 *
 * <p>Holds the robot pose (x, y, yaw) in field coordinates and the
 * corresponding velocity components in the field frame (velX, velY).
 * Instances are immutable and intended to be a lightweight transport
 * object for telemetry, logging, or motion calculations.</p>
 *
 * <p>Note: this class does not assume any particular units for position
 * or velocity — they should be consistent across producers and consumers
 * (for example, meters and meters/second). Yaw is typically expressed in
 * radians, but callers should treat it consistently.</p>
 */
@SuppressWarnings("unused") // prevent method unused warnings from cramping my style :)
public class OdometryPacket {
    private final double x,y,yaw,velX,velY;

    /**
     * Construct a new OdometryPacket.
     *
     * @param x field X position (units are caller-defined, e.g. meters)
     * @param y field Y position (same units as x)
     * @param yaw robot heading (typically in radians; must be consistent across code)
     * @param velocityX velocity along the field X axis (units/time, e.g. meters/second)
     * @param velocityY velocity along the field Y axis (units/time, e.g. meters/second)
     */
    public OdometryPacket(double x, double y, double yaw, double velocityX, double velocityY){
       this.x = x;
       this.y = y;
       this.yaw = yaw;
       this.velX = velocityX;
       this.velY = velocityY;
   
    }

    /**
     * Returns the X position in field coordinates.
     * @return the field X position
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the Y position in field coordinates.
     * @return the field Y position
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the robot heading (yaw).
     * @return the yaw/heading (typically in radians)
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * Returns the velocity along the field X axis.
     * @return velocity in the X direction
     */
    public double getVelX() {
        return velX;
    }

    /**
     * Returns the velocity along the field Y axis.
     * @return velocity in the Y direction
     */
    public double getVelY() {
        return velY;
    }
}