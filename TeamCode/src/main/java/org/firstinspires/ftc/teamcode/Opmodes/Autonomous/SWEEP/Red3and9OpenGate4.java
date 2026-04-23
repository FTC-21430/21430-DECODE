package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous(name="Red3and9OpenGate3", group="RedAutonomous", preselectTeleOp = "RedTeleopPostAuto")
public class Red3and9OpenGate4 extends BaseAuto {

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
        path.chill(0.3); // stablize rotation
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the first set
        path.splineToConstantAngle(POS.INTAKE_START_1, 0.8);
        path.chill(0.5);
        path.splineToConstantAngle(POS.INTAKE_END_1, 0.63);
        path.chill(0.35);
        path.splineToConstantAngle(POS.GATE_PREP, 0.8);
        path.chill(0.2);
        path.splineToConstantAngle(POS.GATE_OPEN,0.68); // stops intaking
        path.chill(0.6); // holding gate


        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.CLOSE_3, 0.7);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the second set
        path.splineToConstantAngle(POS.INTAKE_START_2, 0.8);
        path.splineToConstantAngle(POS.INTAKE_END_2, 0.63);
        path.chill(0.6);

        path.splineToConstantAngle(POS.GATE_PREP, 0.8);

        path.splineToConstantAngle(POS.GATE_OPEN,0.54); // stops intaking
        path.chill(0.6); // holding gate

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.CLOSE_3,0.7);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.3);

        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the third set
        path.splineToConstantAngle(20, 12, 270, 1);
        path.splineToConstantAngle(POS.INTAKE_START_3, 0.85);
        path.splineToConstantAngle(POS.INTAKE_END_3, 0.63);
        path.chill(0.7);

        path.splineToConstantAngle(POS.GATE_PREP, 0.8);
        path.splineToConstantAngle(POS.GATE_OPEN,0.55); // stops intaking
        path.chill(0.6); // holding gate

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(POS.CLOSE_3, 0.7);

        path.chill(0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.splineToConstantAngle(-43, 18, 90,0.8);
        path.addAction(RobotActions.Actions.IDLE);
        path.splineEnd(-46, 21, 90);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true, "red");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-62.22,38.5,180);
        robot.setAlliance("red");
//        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            robot.operatorStateMachine.updateStateMachine();
            robot.updateRobot(false,false,true);
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}