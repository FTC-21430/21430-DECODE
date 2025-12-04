package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;

public class Lifter {
    private HardwareMap hardwareMap = null;
    private ElapsedTime runtime = null;

    public PIDFController pidfController = null;
    private double pCon;
    private double iCon;
    private double dCon;
    DcMotor liftRight;
    DcMotor liftLeft;

public Lifter(HardwareMap hardwareMap, ElapsedTime runtime, double pCon, double iCon, double dCon) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.pCon = pCon;
        this.iCon = iCon;
        this.dCon = dCon;
    liftLeft = hardwareMap.get(DcMotor.class, "liftL");
    liftRight = hardwareMap.get(DcMotor.class, "liftR");
    }

    pidfController = new PIDFController(pConstant, iConstant, dConstant, fConstant,runtime);

    public void liftUp (ElapsedTime runtime) {
    pidfController.update();
    liftLeft.setPower(pidfController.getPower());
    liftRight.setPower(pidfController.getPower());
    }
    public void liftDown() {
        double start = runtime;
        //TODO: find time to lift robot
        double liftTime;
        while (runtime < start+liftTime){
            liftRight.setPower(-0.5+pidfController.getFConstant());
            liftLeft.setPower(-0.5+pidfController.getFConstant());
        }
        liftLeft.setPower(0);
    }
}