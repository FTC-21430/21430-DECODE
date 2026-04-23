package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import androidx.core.content.res.FontResourcesParserCompat;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;

@Autonomous(name = "BlueClose3", group = "BlueAutonomous", preselectTeleOp = "BlueTeleopPostAuto")
public class BlueClose3nPark extends BaseAuto {

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
        path.splineEnd(-56, -18, -90);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true,"blue");
        robot.setAlliance("blue");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,-34.88,180);
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
    }
}