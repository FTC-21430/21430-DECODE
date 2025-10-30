package org.firstinspires.ftc.teamcode.Firmware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;

public class DecodeBot extends Robot{

    public Launcher launcher = null;
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto){

        this.opMode = opMode;

        this.telemetry = telemetry;

        bulkSensorBucket = new BulkSensorBucket(hardwareMap);

        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap,telemetry);

        bulkSensorBucket.clearCache();

    }
}
