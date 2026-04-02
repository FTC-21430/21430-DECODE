package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;

import java.util.ArrayList;

/**
 * Don't have a super solid plan for this yet, but it needs to be involved with the PathPlanning class to add to an action cue alongside a spline movement
 *
 * The actions contained within this class are robot specific, but lets try to make it easy to change and reuse for future seasons! if someone has a better idea, LMK! - Tobin
 *
 */
public class RobotActions {
    public enum Actions {
        INTAKE,
        LAUNCH_THREE,
        LAUNCH_SORTED_THREE,
        SCAN_MOTIF,
        READY_RAMP,
        IDLE
        //TODO: add extra possible actions the robot's mechanisms could do during autonomous to this list
    }

    private ArrayList<Action> actions;
    private DecodeBot bot;
    public RobotActions(DecodeBot decodeBot){
        // store the bot reference and initialize the actions list
        this.bot = decodeBot;
        this.actions = new ArrayList<>();
    }
    // trigger time implementation, constructor with the double as the trigger is used
    public void addAction(Actions actionType, double triggerTime){
        Action newAction = new Action(actionType, bot, triggerTime);
        actions.add(newAction);
    }
    // spline order implementation, constructor with the int as the trigger is used
    public void addAction(Actions actionType, int splineOrder){
        Action newAction = new Action(actionType, bot, splineOrder);
        actions.add(newAction);
    }
    Action[] compileActions(){
        // make the actions array list sync to a final array as the actions to be used.
        return actions.toArray(new Action[0]);
    }
}