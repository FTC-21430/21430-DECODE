package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.Waypoint;

@Autonomous(name = "RedFar3+15Corner", group = "RedAutonomous", preselectTeleOp = "RedTeleopPostAuto")
public class RedFarCollectFromCorner extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        Waypoint corner = new Waypoint(66.3,62.5,270,1,true);
        path.splineStart(GlobalPositions.POS.FAR_START);
        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_FAR);

        path.chill(0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

        path.splineToConstantAngle(GlobalPositions.POS.FAR_3, 1);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.4);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        for (int i = 0; i < 7; i++){
            path.addAction(RobotActions.Actions.INTAKE);
            path.splineToConstantAngle(64.3,56.5,270,1);
            path.splineToConstantAngle(corner, 1);
            path.chill(0.35);
            path.splineToConstantAngle(64.3,62.5,270, 1);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
            path.splineToConstantAngle(GlobalPositions.POS.FAR_3, 1);
            path.addAction(RobotActions.Actions.PREPPING);

            path.chill(0.5);
            path.addAction(RobotActions.Actions.LAUNCH);
            path.chill(0.9);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        }
        path.addAction(RobotActions.Actions.IDLE);
        path.splineEnd(GlobalPositions.POS.FAR_END);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true,"red");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(63.5, 18.5, 180);
        robot.setAlliance("red");
//        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        robot.rotationControl.setPIDController(0.0201,0.0005,0.0007);
        robot.operatorStateMachine.SUPER_FAR_MODE = true;
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            robot.updateRobot(false,false,false);
            robot.operatorStateMachine.updateStateMachine();
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}
