package org.firstinspires.ftc.teamcode.Resources;

import java.util.ArrayList;

// The goal of this class is to input in the distance from the april tag and return the needed launch angle and speed to hit the goal
public class TrajectoryKinematics {
    // A list of values that the angles have been measured at. The order of these values need to be related to the angles and magnitudes list.
    // TODO: these values need to be tuned in a sample size of every 5 inches. Current values were from early season prototype testing
    // NOTE: these values are not actually used, but are the values that the regression functions were made. These values were really calculated in Desmos Graphing Calculator
    private final double[] distances = {
            24,
            81,
            94,
            120,

    };

    private final double[] angles = {
            20,
            35,
            35,
            35
    };
    private final double[] magnitudes = {
            1100,
            1400,
            1550,
            1750,
    };

    // The return value for the angle of the ramp. - in degrees
    private double initialAngle = 0;
    // the return value for how fast the flywheel should go to achieve a launch. In degrees per second
    private double launchMagnitude = 0;
    public TrajectoryKinematics(){
    }

    /**
     * Updates the return variables based on the two regression functions
     * @param distanceInches Inches, the distance from the front (camera side) of the robot and the april tag.
     */
    public void calculateTrajectory(double distanceInches) {
   // All regression functions are calculated using the stored values above and put into Desmos graphing calculator to create a fourth degree regression function!
        initialAngle = angleRegression(distanceInches);
        launchMagnitude = magnitudeRegression(distanceInches);
    }

    /**
     * the regression function tuned by Desmos calculator and based on real world testing data.
     * @param distance Inches, the distance from the front (camera side) of the robot and the april tag.
     * @return returns the angle the ramp needs to be to hit the goal from the given distance
     */
    private double angleRegression(double distance){
        // values a-e represent the tuning values of this 4th degree polynomial calculated by Desmos using the distance and angle values above
        double a =-3.8327541 * Math.pow(10,-13);
        double b = 2.88733484 * Math.pow(10,-10);
        double c = -9.1384146 * Math.pow(10,-8);
        double d = 1.5804336 * Math.pow(10,-5);
        double e = -1.6266162 * Math.pow(10,-3);
        double f = 1.0159161 * Math.pow(10, -1);
        double g = -3.7433493 * Math.pow(10,0);
        double h = 7.3853696 * Math.pow(10,1);
        double i = -5.3187568 * Math.pow(10,2);

        // Math.pow is the exponent function, this is a fourth degree polynomial that is tuning based on real world testing
        return a * Math.pow(distance,8)+ b * Math.pow(distance,7) + c * Math.pow(distance,6) + d * Math.pow(distance,5) + e * Math.pow(distance,4) + f * Math.pow(distance,3)
                + g * Math.pow(distance,2) + h * Math.pow(distance,1) + i;
    }

    /**
     * The regression function for flywheel speed tuned by Desmos calculator and based on real world testing data.
     * @param distance Inches, the distance from the front (camera side) of the robot and the april tag.
     * @return degrees per second, the speed the flywheel needs to be to hit the goal from the given distance
     */
    private double magnitudeRegression(double distance){
        //quadratic tuning values
        double a = 0.000461017;
        double b = -0.096802;
        double c = 11.34428;
        double d = 909.04077;

        // Math.pow is the exponent function, this is a second degree polynomial that is tuning based on real world testing
        return a * Math.pow(distance,3) + b * Math.pow(distance,2) + c * Math.pow(distance,1) + d;
    }

    /**
     * Getter for initial angle
     * @return degrees
     */
    public double getInitialAngle(){
        return initialAngle;
    }

    /**
     * getter for the flywheel speed
     * @return degrees per second
     */
    public double getLaunchMagnitude(){
        return launchMagnitude;
    }
}
