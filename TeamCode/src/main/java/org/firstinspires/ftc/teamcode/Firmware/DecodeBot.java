package org.firstinspires.ftc.teamcode.Firmware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerServoFirmware;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;

public class DecodeBot extends Robot{

    public Launcher launcher = null;
    public Spindexer spindexer = null;
    public Intake intake = null;
  
    //The PID values are a public because we need to tune it later and public makes it easier to do that
    public static final double P_CONSTANT = 0.15;
    public static final double I_CONSTANT = 0.1;
    public static final double D_CONSTANT = 0.02;
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto){
        pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
        this.opMode = opMode;

        this.telemetry = telemetry;

        // TODO: change the pod offset values to what they are on the competition robot, currently tuned for software testing bot
        odometry = new GobildaPinpointModuleFirmware(hardwareMap, 7.2,-18,reset);

        bulkSensorBucket = new BulkSensorBucket(hardwareMap);

        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap,telemetry);
        intake = new Intake(hardwareMap, telemetry);

        spindexer = new Spindexer(hardwareMap,telemetry);
        rotationControl = new RotationControl(0.3,0.025,0,0.0001,robotAngle);




        bulkSensorBucket.clearCache();





    }
    @Override
    //TODO:Call updates for sensors and actuators
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        spindexer.updateSpindexer();
        launcher.updateSpeedControl();
    }
}
