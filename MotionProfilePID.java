/*
  @author: Abinav
*/
package org.usfirst.frc.team5427.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import java.util.*;
import org.usfirst.frc.team5427.robot.*; 

public class MotionProfilePID extends Command {
  public ArrayList<Timepoint> trajectory;   //trajectory contains Timepoints within path
  public int currentStep;                   //index of the current feedforward Timepoint
  public int counter;                       //counter of the amount of execute() iterations
  public int frequencyDiv;                  //calculations in execute() runs every frequencyDiv iterations
  public double lastTimeRecorded;           //records the timestamp from the previous iteration
  public double lastPosError;
  public double Kp, Kd;                     //gain values for positional PID
  public double Ktheta;                     //gain values for angular PID
  public double leftSCGSpeed;
  public double rightSCGSpeed;
  public Path path;

  public MotionProfilePID(Path path) {
    requires(Robot.driveTrain);
    this.path = path;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lastTimeRecorded = Timer.getFPGATimestamp();
    trajectory = new ArrayList<>();
    // trajectory.add(new Timepoint(0.0, 0.0, 0.0, 0.0));
    // trajectory.add(new Timepoint(3.38, 0.0, -1, -1));
    // trajectory.add(new Timepoint(6.76, 0.0, -1, -1));
    // trajectory.add(new Timepoint(10.14, 0.88, -1, -1));
    // trajectory.add(new Timepoint(13.52, 0.88, -1, -1));

    Robot.ahrs.reset();
    currentStep = 0;
    frequencyDiv = 3;
    lastPosError = 0;
    Kp = 1;
    Kd = 1;
    Ktheta = 1;
  }

  // Called repeatedly when this Command is scheduled to run
  // runs at 50 Hz: 50 iterations per second.
  @Override
  protected void execute() {
        //currentStep needs to be updated?
        //Timepoint based of equidistance or time?

      counter = ++counter % frequencyDiv;                                                       //counter goes 1,2,...frequencyDiv and repeats
      double curTime = Timer.getFPGATimestamp();
      double dt = curTime - lastTimeRecorded;                                                   //time differential
      double curPosition = (Robot.encLeft.getDistance() + Robot.encRight.getDistance())/2;      //avg position from avg encoder values     
      double positionError = trajectory.get(currentStep).getPosition() - curPosition;           //position error (Manual PD Controller from PID)
      double trackError = trajectory.get(currentStep).getHeading() - Robot.ahrs.getAngle();     //track error for heading (Manual P Controller) 
      double derivativeError = (positionError - lastPosError)/dt;                               //derivative error (Manual PD Controller from PID)

        leftSCGSpeed = 
            -trajectory.get(currentStep).getVelocity()
            -trajectory.get(currentStep).getAcceleration()
            - Kp * positionError
            - Kd * derivativeError
            + Ktheta * trackError;

        rightSCGSpeed = 
            + trajectory.get(currentStep).getVelocity()
            + trajectory.get(currentStep).getAcceleration()
            + Kp * positionError
            + Kd * derivativeError
            + Ktheta * trackError;
      
        currentStep = counter == 0 ? currentStep+1 : currentStep;                               //updates step every frequency divide: 16.6 Hz (Iterations per second)

      Robot.driveTrain.tankDrive(-leftSCGSpeed, rightSCGSpeed);
      lastPosError = positionError;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //ends when on the last step AND when the frequency divide resets
    return currentStep == trajectory.size()-1 && counter == 0; //this might not work because statements might not be true
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveLeft.set(0);
    Robot.driveRight.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
}

//Custom Timepoint Object
class Timepoint{

  private double position;     //[inifinite] measured in inches, describes position of next Timepoint
  private double heading;      //[-180, 180] the angle of the segment's movement
  private double velocity;     //[-1.0, 1.0]
  private double acceleration; //[ 0.0, unknown]

  public Timepoint(double position, double heading, double velocity, double acceleration){
    this.position = position;
    this.heading = heading;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }

  //Timepoint Accessors
  public double getPosition(){return position;}
  public double getHeading(){return heading;}
  public double getVelocity(){return velocity;}
  public double getAcceleration(){return acceleration;}

  //Timepoint Mutators
  public void setPosition(double position){this.position = position;}
  public void setHeading(double heading){this.heading = heading;}
  public void setVelocity(double velocity){this.velocity = velocity;}
  public void setAcceleration(double acceleration){this.acceleration = acceleration;}

}
