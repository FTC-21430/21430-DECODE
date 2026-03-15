package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous
public class InitialSWEEPImplementation extends BaseAuto {

    private void defineRoute(){
        // Use splineStart here instead of chill so we create an actual spline from (0,0) -> (24,0).
        // Also set the planner's robotSpeed to a very small value for visible slow following during testing.

        robot.SWEEP.pathPlanner.splineStart(0,0,0);

        robot.SWEEP.pathPlanner.splineTo(24,0,1);

        robot.SWEEP.pathPlanner.splineTo(0,24, 1);
        robot.SWEEP.pathPlanner.splineTo(-24,0, 1);
        robot.SWEEP.pathPlanner.splineToConstantAngle(0,0,180,1);
        robot.SWEEP.pathPlanner.chill(0,0,180,2);

//        robot.SWEEP.pathPlanner.chill(0,0,180,2);
//
//        robot.SWEEP.pathPlanner.chill(0,0,0,20);
//        robot.SWEEP.pathPlanner.chill(0,0,0,20);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){

            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
//            robot.intake.updateIntake();
//            robot.operatorStateMachine.updateStateMachine();
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());

            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}