package org.firstinspires.ftc.teamcode.Resources;


import com.qualcomm.robotcore.util.ElapsedTime;

// this class is for autonomous movement of the drivetrain.
public class PathFollowing {
  
  // the speed at which the robot should be following to the point
  private double followSpeed = 1;
  
  // these are the PID controllers for both the x and y axis relative to the field.
  // these powers get converted over to relative to the robot later on
  private PIDController xPID;
  private PIDController yPID;
  
  // these are the tuning variables for both PID controllers.
  private double pXConstant, pYConstant;
  private double dXConstant, dYConstant;
  
  // the output power we use from this class to move the drive train.
  // Stands for powerSideways and power Forwards
  double powerS, powerF;
  
  
  /**
   * the constructor for this class. this is used to set the class attributes
   * and run the constructors of both PID controllers
   * @param pX proportional constant X
   * @param pY proportional constant Y
   * @param dX derivative constant X
   * @param dY derivative constant Y
   * @param runtime the runtime object from the first SDK
   */
  public PathFollowing(double pX, double pY, double iX, double iY, double dX, double dY, ElapsedTime runtime){
   pXConstant = pX;
   pYConstant = pY;
   dXConstant = dX;
   dYConstant = dY;
  
    xPID = new PIDController(pXConstant, 0.2, dXConstant, runtime);
    yPID = new PIDController(pYConstant, 0.2, dYConstant, runtime);
  }
  
  
  /**
   * this is used as the update method for this class. you should call this every iteration before getting the powers S & F
   * @param robotX the robots CURRENT X position
   * @param robotY the robots CURRENT Y position
   * @param robotAngle the robots CURRENT angle in degrees.
   */
  public void followPath(double robotX, double robotY, double robotAngle){
    // run the math for both PID controllers
    xPID.update(robotX);
    yPID.update(robotY);
    
    // takes the output powers from the X and Y PID controllers and rotates them to be the robots X and Y movement.
    // here we also affectthese powers with follow speed. when programming auto routes you should use
    // the setter for follow speed instead of changing the speed for the drivetrain
    powerS = xPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - yPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) * followSpeed;
    powerF = xPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + yPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) * followSpeed;
  }
  
  // returns Power S after you run the followPath method
  public double getPowerS(){
    return powerS;
  }
  
  // returns Power F after you run the followPath method
  public double getPowerF(){
    return powerF;
  }
  
  // used to set the position the robot is trying to go to.
  public void setTargetPosition(double x, double y){
    xPID.setTarget(x);
    yPID.setTarget(y);
  }
  
  // used to set followSpeed
  public void setFollowSpeed(double speed){
    followSpeed = speed;
  }
  
  // returns followSpeed just in case you need it somewhere.
  public double getFollowSpeed(){ return followSpeed; }

  public void setAutoSpeed(double p, double i, double d){
    xPID.updateConstants(p,i,d);
    yPID.updateConstants(p,i,d);
  }
}