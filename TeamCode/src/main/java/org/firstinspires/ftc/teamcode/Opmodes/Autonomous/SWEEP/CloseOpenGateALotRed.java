package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous(name = "RedCloseGateCycle", group = "RedAutonomous", preselectTeleOp = "RedTeleopPostAuto")
public class CloseOpenGateALotRed extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(POS.CLOSE_START);
        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_CLOSE);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.CLOSE_3,0.7);
        // get to launch position
        path.chill(0.2); // stablize rotation
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);

        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_GOAL);
        // go for the second set
        path.splineToConstantAngle(11.5, 18, 270, 1);
        path.splineToConstantAngle(11.5, 57.5, 270, 0.7);
        path.chill(0.1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.INTAKE_START_2, 1);

        path.splineToConstantAngle(-11,18,155, 0.7);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);


        for (int i = 0; i < 3 ; i ++){
            path.addAction(RobotActions.Actions.INTAKE);
            path.splineToConstantAngle(9, 38, 300,0.9);
            path.splineToConstantAngle(9.4, 58.1, 310,0.42);// Opening from gate?
            path.splineToConstantAngle(14.4, 58, 310,0.42);
            path.chill(0.9);
            path.splineTo(9.3, 46,0.7);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
            path.splineToConstantAngle(-11,18,155, 0.6);
            path.addAction(RobotActions.Actions.PREPPING);
            path.chill(0.2);
            path.addAction(RobotActions.Actions.LAUNCH);
            path.chill(0.8);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        }
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the first set

        path.splineToConstantAngle(POS.INTAKE_START_1, 1);
        path.chill(0.1);
        path.splineToConstantAngle(POS.INTAKE_END_1, 0.63);

        path.chill(0.1);

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-11,18,155, 0.7);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineToConstantAngle(-40,18,155, 0.7);

        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

        path.addAction(RobotActions.Actions.IDLE);
        path.splineEnd(-43.5, 21, 90);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,38.5,180);
        robot.setAlliance("red");
//        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            robot.updateRobot(false,false,false);
            robot.operatorStateMachine.updateStateMachine();
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
        robot.operatorStateMachine.updateStateMachine();
    }
}
