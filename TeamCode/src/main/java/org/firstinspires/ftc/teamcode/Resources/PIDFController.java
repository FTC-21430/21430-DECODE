package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController extends PIDController{
    private double fConstant;

    /**
     * @param pConstant - Proportional Constant - used for tuning the Proportional factor.
     * @param iConstant - Integral Constant- used for tuning the Integral factor
     * @param dConstant - Derivative Constant - used for tuning the Derivative factor
     * @param fConstant - Feed forward - adds a constant power force to the output
     * @param runtime - The runtime instance from the main op-mode
     */
    public PIDFController(double pConstant, double iConstant, double dConstant, double fConstant,ElapsedTime runtime){
        super(pConstant, iConstant, dConstant, runtime);
        this.fConstant = fConstant;
    }
    public void updatePIDFConstants(double pConstant,double iConstant, double dConstant, double fConstant){
        updatePIDConstants(pConstant,iConstant,dConstant);
        this.fConstant = fConstant;
    }
    public double getFConstant(){
        return fConstant;
    }
}