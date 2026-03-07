package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

public class InitialSWEEPImplementation extends BaseAuto {

    private void defineRoute(){
        robot.SWEEP.pathPlanner.chill(0,0,0,1);
        robot.SWEEP.pathPlanner.splineTo(0,0,1);
        robot.SWEEP.pathPlanner.splineTo(24,0,0.75);
        robot.SWEEP.pathPlanner.splineToConstantAngle(0,24, 0, 0.25);
        robot.SWEEP.pathPlanner.chill(0,24,180,2);
        robot.SWEEP.pathPlanner.splineToConstantAngle(-24,-24, 90, 0.5);
        robot.SWEEP.pathPlanner.splineTo(0,0,1);
        robot.SWEEP.pathPlanner.chill(0,0,0,3);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        robot.SWEEP.computeSplines();
        waitForStart();
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
