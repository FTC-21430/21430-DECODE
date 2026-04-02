package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;

public class RedChihuahuaSPLINE {package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

    @Autonomous
    public class RedChihuahuaSPLINE extends BaseAuto {


        /// Route definition methods:
        /// all units are in inches, degrees, and seconds.
        /// path.splineStart(x,y,angle) the start of the path, call once
        /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
        /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
        /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
        private void defineRoute(){
            PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
            path.splineStart(49,10,90); // the start of the path
            path.splineTo(63,54,8); //need to change angle to 154 to shoot
            path.splineTo(63,61,8);
            robot.chill(true,0.4);
            path.splineTo(49,10,8); //need to change angle to 154 to shoot
            path.splineTo(51,48,8); //need to change angle to 0 to pick up artifacts
            path.splineTo(36,48,8); //need to change angle to 0 to pick up artifacts
            robot.chill(true,0.6);
            path.splineTo(49,10,8); //need to change angle to 154 to shoot
            path.splineTo(28,9,8);
            robot.chill(true,3);
            path.splineEnd(28,9,90);
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
                //TODO: update robot system loops
                robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
                telemetry.update();
                robot.bulkSensorBucket.clearCache();
            }
        }
    }
}
