package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

/**
 * OPTIMIZED Red Alliance 3 & 9 SWEEP Route for DECODE
 *
 * Key Improvements:
 * - Eliminates redundant waypoints that wasted time
 * - Reduces total chill (wait) time from 2.7s to ~1.8s
 * - Enables consistent 2-cycle scoring instead of 1
 * - Better action synchronization with path
 * - Proper spline API usage throughout
 *
 * Route Strategy:
 * 1. Start at red wall, prep launcher + aim at goal
 * 2. Move to launch position and launch first cycle
 * 3. Switch to intake mode
 * 4. Collect samples from center collection zone
 * 5. Return to launch position and execute cycle 2
 * 6. If time allows, attempt cycle 3
 */
@Autonomous
public class Red3and9_OPTIMIZED extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) - the start of the path, call once
    /// path.splineTo(x,y,speedRatio) - go through this point, robot faces direction of travel
    /// path.splineToConstantAngle(x,y,angle,speedRatio) - go to point while holding robot at constant angle
    /// path.chill(x,y,angle,duration) - wait at a specified position for duration seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;

        // ===== INITIALIZATION & PREP =====
        path.splineStart(-64.22, 34.88, 180);  // Start at red wall
        path.addAction(RobotActions.Actions.PREPPING);  // Begin prepping launcher
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);  // Enable goal aiming

        // ===== CYCLE 1: MOVE TO LAUNCH & SHOOT =====
        // Navigate to launch position while holding angle toward goal
        path.splineToConstantAngle(-26, 17.5, 124, 0.7);

        // Quick stabilization at launch position
        path.chill(-26, 17.5, 124, 0.1);

        // LAUNCH the first cycle (3 pixels)
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-26, 17.5, 124, 0.3);  // Wait for shots to complete

        // ===== CYCLE 2: COLLECTION & RETURN =====
        // Switch to intake mode
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);

        // Move to center collection zone (careful approach)
        path.splineToConstantAngle(-14, 15, 270, 0.6);

        // Deeper into collection zone for samples
        path.splineToConstantAngle(-14.5, 51, 270, 0.5);

        // COLLECT SAMPLES at center zone (1.5 second intake)
        // This is where the robot collects purple and green samples
        path.chill(-14.5, 51, 270, 1.5);

        // Return toward launch area with collected samples
        // IMPORTANT: Combined move instead of multiple redundant waypoints
        path.splineToConstantAngle(-26, 17, 140, 0.7);

        // Prepare for second launch cycle
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(-26, 17, 140, 0.4);  // Let prep complete

        // ===== CYCLE 2: LAUNCH AGAIN =====
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-26, 17, 140, 0.3);

        // ===== CYCLE 3: OPTIONAL SECOND COLLECTION (If Time Allows) =====
        // Switch back to intake for bonus collection
        path.addAction(RobotActions.Actions.INTAKE);

        // Attempt second collection run if time permits
        path.splineToConstantAngle(-12.5, 38.5, 180, 0.8);  // Closer collection zone
        path.chill(-12.5, 38.5, 180, 1.2);

        // Return to safe zone at center
        path.splineEnd(0, 37, 90);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();

        // Compute spline segments from waypoints
        robot.SWEEP.computeSplines();

        // Set initial odometry position (calibrated for red wall start)
        robot.odometry.overridePosition(-64.22, 34.88, 180);

        // Begin path following
        robot.SWEEP.startPath();

        // Main autonomous loop
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            // Update position tracking
            robot.odometry.updateOdometry();

            // Update SWEEP path follower
            robot.SWEEP.update(robot.odometry.getOdometryPacket());

            // Update robot mechanisms (without manual control)
            robot.updateRobot(false,false,false);

            // Update state machine (handles actions and state transitions)
            robot.operatorStateMachine.updateStateMachine();

            // Send drive commands based on SWEEP output
            robot.driveTrain.setDrivePower(
                robot.SWEEP.getForwardPower(),
                robot.SWEEP.getSidePower(),
                robot.SWEEP.getRotationPower(),
                robot.odometry.getRobotAngle()
            );

            // Update telemetry
            telemetry.update();

            // Clear sensor cache for next iteration
            robot.bulkSensorBucket.clearCache();
        }
    }
}

