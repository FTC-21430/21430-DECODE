package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous
public class InitialSWEEPImplementation extends BaseAuto {

    private void defineRoute(){
        robot.SWEEP.pathPlanner.chill(0,0,0,1);

        robot.SWEEP.pathPlanner.splineTo(24,0,1);
        robot.SWEEP.pathPlanner.splineTo(0,24, 1);
        robot.SWEEP.pathPlanner.splineTo(-24,-24, 1);
        robot.SWEEP.pathPlanner.splineToConstantAngle(0,0,180,1);
        robot.SWEEP.pathPlanner.chill(0,0,180,2);
//        robot.SWEEP.pathPlanner.splineTo(0,0,1);
//        robot.SWEEP.pathPlanner.chill(0,0,0,20);
//        robot.SWEEP.pathPlanner.chill(0,0,0,20);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        robot.SWEEP.computeSplines();
        waitForStart();
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
