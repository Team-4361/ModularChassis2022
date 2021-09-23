/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import org.graalvm.compiler.core.common.calc.CanonicalCondition;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.Library.Autonomous.BallVisionCamera;
import frc.Library.Chassis.TankDrive;
import frc.Library.Controllers.Drive;
import frc.Library.Controllers.PneumaticsControl;
import frc.Library.Controllers.TalonEncoder;
import frc.Library.Controllers.TurnControl;
import frc.Library.Controls.JoystickTank;
import frc.Library.Controls.XboxArcade;
import frc.Library.Controls.XboxTank;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // if mode = 1; frisbee shooter
  // if mode = 2; thor's hammer
  // if mode = 3; romulus.
  // if mode = 4; Xbox tank/arcade drive control (no operator)
  public static int mode = 5;
  SendableChooser<String> modularMode = new SendableChooser<>();


  //Define the Talons for the Tank Drive
  //DR MOTOR 01 = Left Front
  //DR Motor 02 = Right Front
  //DR Motor 03 = Left Back
  //DR Motor 04 = Right Back
  WPI_TalonSRX lDrive1 = new WPI_TalonSRX(3);
  WPI_TalonSRX lDrive2 = new WPI_TalonSRX(1);
  WPI_TalonSRX[] lDriveMotors = { lDrive1, lDrive2 };
  Drive lDrive = new Drive(lDriveMotors);

  WPI_TalonSRX rDrive1 = new WPI_TalonSRX(4);
  WPI_TalonSRX rDrive2 = new WPI_TalonSRX(2);
  WPI_TalonSRX[] rDriveMotors = { rDrive1, rDrive2 };
  Drive rDrive = new Drive(rDriveMotors);
  TankDrive theTank = new TankDrive(lDrive, rDrive);

  // Joystick 0 = Left Drive
  // Joystick 1 = Right Drive
  Joystick lStick = new Joystick(0);
  Joystick rStick = new Joystick(1);

  Joystick cannonControls = new Joystick(2);
  JoystickButton rightBumper = new JoystickButton(cannonControls, 6);
  JoystickButton leftBumper = new JoystickButton(cannonControls, 5);
  JoystickButton yBTN = new JoystickButton(cannonControls, 4);
  JoystickButton aBTN = new JoystickButton(cannonControls, 2);

  XboxController xCont = new XboxController(2);
  XboxArcade xContArCon = new XboxArcade(2, Hand.kLeft);
  // snowblower motor for frisbee shooter
  //WPI_TalonSRX modTalon1 = new WPI_TalonSRX(3);
  WPI_TalonSRX modTalon1 = new WPI_TalonSRX(8);
  // CIM motor for frisbee shooter
  WPI_TalonSRX modTalon2 = new WPI_TalonSRX(5);
  // modular talon 3
  WPI_TalonSRX modTalon3 = new WPI_TalonSRX(7);
  // modular talon 4
  WPI_TalonSRX modTalon4 = new WPI_TalonSRX(6);

  Shooter shooter = new Shooter(modTalon1, modTalon2);

  BallVisionCamera ballTracker;
  final String networkTableName = "photonvision";
  final String cameraName = "RoxCam2021-4361";

  HashMap<String, Double> dataMap;

  Encoder leftModularEncoder;
  Encoder rightModularEncoder;

  double distanceToTarget;
  double degreeToTarget;

  Boolean shouldMoveFoward;
  Boolean continueMoving;
  final float distancePerPulse = 0.63837f/250.0f;

  HashMap<String, Double> info;
  PIDController distancePIDController;
  PIDController rotationPIDController;

  DigitalInput rightLimit, leftLimit;

  Boolean ybtnReleased = false;
  Boolean abtnReleased = false;

  Boolean rightBumperReleased = false;
  Boolean leftBumperReleased = false;


  @Override
  public void robotInit() {
    ballTracker = new BallVisionCamera(networkTableName, cameraName, 0.1397, 0);

    dataMap = new HashMap<String, Double>();

    rightModularEncoder = new Encoder(2,3);
    leftModularEncoder = new Encoder(0,1);
    rightModularEncoder.reset();
    leftModularEncoder.reset();

    //Distance per pulse is in meters
    leftModularEncoder.setDistancePerPulse(distancePerPulse);
    rightModularEncoder.setDistancePerPulse(distancePerPulse);

    rightLimit = new DigitalInput(5);
    leftLimit = new DigitalInput(6);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * double[] stickVal = sticks.GetDrive(); stickVal[0] = stickVal[0]*-.8;
     * stickVal[1] = stickVal[1]*-.8;
     */
    
    if (mode == 1) {
      
      theTank.drive(-lStick.getY(), rStick.getY());
       
      
      // FRISBEE SHOOTER MODE
      if (xCont.getXButtonReleased()) {
        System.out.println("pffffft 0");
        modTalon2.set(0);
      }
      if (xCont.getXButtonPressed()) {
        System.out.println("pffffft 1");
        modTalon2.set(1);
      }

      if (xCont.getBButtonReleased()) {
        System.out.println("tsktsktsk 0");
        modTalon1.set(0);
      }
      if (xCont.getBButtonPressed()) {
        System.out.println("tsktsktsk 1");
        modTalon1.set(1);
      }
    }
    if (mode == 2) {
      theTank.drive(-lStick.getY(), rStick.getY());

      // HAMMER MODE
      if (xCont.getYButtonPressed()) {
        System.out.println("hammer BACK");
        modTalon1.set(-1);
        modTalon2.set(-1);
      }
      if (xCont.getYButtonReleased()) {
        System.out.println("hammer SETTLE");
        modTalon1.set(0);
        modTalon2.set(0);
      }
      if (xCont.getAButtonPressed()) {
        System.out.println("hammer FORWARD");
        modTalon2.set(1);
        modTalon1.set(1);
      }
      if (xCont.getAButtonReleased()) {
        System.out.println("hammer SETTLE 2");
        modTalon2.set(0);
        modTalon1.set(0);
      }
    }
    if (mode == 3) {
      theTank.drive(-lStick.getY(), rStick.getY());

      // BALL SHOOTER
      //boolean spinnyThingSpinningQuestionMark = false;
      //boolean indexThingSpinningQuestionMark = false;
      //if (xCont.getAButtonPressed()) {
      //  spinnyThingSpinningQuestionMark = !spinnyThingSpinningQuestionMark;
      //  if (spinnyThingSpinningQuestionMark) {
      //    modTalon3.set(1.0);
      //  } else if (!spinnyThingSpinningQuestionMark) {
      //    modTalon3.set(0.0);
      //  }
      //}

      //Agitator (spin - Talon 3)
      if (xCont.getAButtonPressed()){
        modTalon3.set(-1.0);
      }else if(xCont.getAButtonReleased()){
        modTalon3.set(0.0);
      }

      //Indexer and Shooter (Talon 4,2 resp.)
      if (xCont.getBButtonPressed()) {
        modTalon4.set(1.0);
        modTalon2.set(-1.0);
      } else if (xCont.getBButtonReleased()) {
        modTalon4.set(0.0);
        modTalon2.set(0.0);
      }
      //Shooter (Talon 2)
      //if (xCont.getYButtonPressed()) {
        //modTalon2.set(-1.0);
      //} else if (xCont.getYButtonReleased()) {
        //modTalon2.set(0.0);
      //}

      //Intake in (Talon 1)
      if(xCont.getRawButtonPressed(6)) {
        modTalon1.set(-1.0);
      } else if (xCont.getRawButtonReleased(6)) {
        modTalon1.set(0.0);
      }
      
      //Intake out
      if(xCont.getRawButtonPressed(5)) {
        modTalon1.set(1.0);
      } else if (xCont.getRawButtonReleased(5)) {
        modTalon1.set(0.0);
      }

      //if (xCont.getX`Pressed()) {
      //  indexThingSpinningQuestionMark = !indexThingSpinningQuestionMark;
      //  if (indexThingSpinningQuestionMark) {
      //    modTalon2.set(1.0);
      //  } else if (!indexThingSpinningQuestionMark) {
      //   modTalon2.set(0.0);
      //  }
      //}
    }

    if (mode == 4)// xbox tank/arcade control
    {
      theTank.drive(xContArCon.GetDrive());

    }

    //T-shirt Cannon
    if (mode == 5){
      theTank.drive(-lStick.getY(), rStick.getY());
      //Talon 4 for rotating
      if (rightLimit != null && leftLimit != null){
        if(!leftLimit.get()){
          if (leftBumper.get()/*xCont.getBumperPressed(GenericHID.Hand.kLeft)*/){
            modTalon1.set(1.0);
            leftBumperReleased = true;
          }else if(!leftBumper.get() && leftBumperReleased/*xCont.getBumperReleased(GenericHID.Hand.kLeft)*/){
            modTalon1.set(0.0);
            leftBumperReleased = false;
          }
        }
        else if(leftLimit.get()){
          modTalon1.set(0.0);
        }

        if(!rightLimit.get()){
          if (rightBumper.get()/*xCont.getBumperPressed(GenericHID.Hand.kRight)*/){
            modTalon1.set(-1.0);
            rightBumperReleased = true;
          }else if(!rightBumper.get() && rightBumperReleased/*xCont.getBumperReleased(GenericHID.Hand.kRight)*/){
            modTalon1.set(0.0);
            rightBumperReleased = false;
          }
        }
        else if(rightLimit.get()){
          modTalon1.set(0.0);
        }
      }

      if (aBTN.get()){
        modTalon2.set(-1.0);
        abtnReleased = true;
      }
      else if(!aBTN.get() && abtnReleased){
        modTalon2.set(0.0);
        abtnReleased = false;
      }

      if (yBTN.get()){
        modTalon2.set(1.0);
        ybtnReleased = true;
      }
      else if(!yBTN.get() && ybtnReleased){
        modTalon2.set(0.0);
        ybtnReleased = false;
      }
      
    }
      

    //System.out.println(leftModularEncoder.getDistance());

    // Smartdashboard Values

    try {
      // SmartDashboard.putNumber("Agitator Current", CAN[4].getOutputCurrent());
      // SmartDashboard.putNumber("Shooter Current", CAN[1].getOutputCurrent());
      // SmartDashboard.putNumber("Climber Current", CAN[5].getOutputCurrent());
      // SmartDashboard.putBoolean("Geared", gearing);
      // SmartDashboard.putBoolean("GearIn", !limit[0].get());
      // SmartDashboard.putNumber("Pusher Current", CAN[9].getOutputCurrent());

      // SmartDashboard.putBoolean("Xbox Control Mode", XboxMode);
    } catch (Exception e) {
      // System.out.println("Smartdashboard: " + e.getMessage());
    }

  }

  public void autonomousInit() {
    distancePIDController = new PIDController(0.4,0,0);
    rotationPIDController = new PIDController(8,0,0);
    HashMap<String, Double> info = ballTracker.getTargetGoal();
    distanceToTarget = info.get("Distance");
    degreeToTarget = info.get("Yaw");
    leftModularEncoder.reset();
    rightModularEncoder.reset();
    shouldMoveFoward = false;
    continueMoving = true;
  }

  
  final double degreeToMeterConst = 0.00833333;
  double motorPower;
  double motorPowerToTurn;

  double currentDistanceAwayFromTarget;
  double currentAngleAwayFromTargt;

  // Todo
  // Get second encoder working
  // Fix weird bug that forces you to invert one wheel to go straight

  // Making movement smooth
  // Move in all if statements and add the amount you need to turn to to the wheel that need to move faster in order for the robot to turn
  // Subtract the normal wheel movement from the wheel that is moving faster to find out when to tell the wheel that is moving faster to stop moving faster
  // Remove the resetting thing
  // You might need to slow down the wheel that is going at normal speed (I'll find out when I test it)
  
  //Try using the old code and just removing setting the motors to 0.

  Boolean hasTurned = false;
  
  //I assume the righhModularEncoder is positive.
  public void autonomousPeriodic() 
  {
    
      //If ball is right of the robot
      if( (degreeToTarget*degreeToMeterConst) > 0 && ((-leftModularEncoder.getDistance() - rightModularEncoder.getDistance()) + 0.04 < -degreeToTarget*degreeToMeterConst) && continueMoving)
      {
        hasTurned = true;

        //Rotate information
        currentAngleAwayFromTargt = (-degreeToTarget*degreeToMeterConst) - leftModularEncoder.getDistance();
        motorPowerToTurn = MathUtil.clamp(distancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);
      
        //Foward information
        currentDistanceAwayFromTarget = distanceToTarget + leftModularEncoder.getDistance();
        motorPower = MathUtil.clamp(distancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

        //Moving to Target
        theTank.drive(MathUtil.clamp(-motorPowerToTurn-motorPower, -1.0, 1.0), motorPower);
        System.out.println("To Right excess | Left: "+ MathUtil.clamp(motorPowerToTurn+motorPower, -1.0, 1.0) + "Right: " + motorPower);
      }
      //If ball is left of the camera
      else if(((-degreeToTarget*degreeToMeterConst) > 0 && (rightModularEncoder.getDistance() + leftModularEncoder.getDistance()) +0.04 < degreeToTarget*degreeToMeterConst) && continueMoving)
      {
        hasTurned = true;

        //Rotate information
        currentAngleAwayFromTargt = (degreeToTarget*degreeToMeterConst) - rightModularEncoder.getDistance();
        motorPowerToTurn = MathUtil.clamp(rotationPIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

        //Foward information
        currentDistanceAwayFromTarget = distanceToTarget + leftModularEncoder.getDistance();
        motorPower = MathUtil.clamp(distancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

        //Moving to Target
        theTank.drive(-motorPower, MathUtil.clamp(motorPowerToTurn+motorPower, -1.0, 1.0));
        System.out.println("To Left excess | Left: " + -motorPower + "Right: " + MathUtil.clamp(motorPowerToTurn+motorPower, -1.0, 1.0));
      }
      else if((-leftModularEncoder.getDistance() < distanceToTarget/4) && continueMoving)
      {
        if(hasTurned){
          leftModularEncoder.reset();
          rightModularEncoder.reset();
          hasTurned = false;
        }
        if(distanceToTarget < 0.04){
          //theTank.drive(0, 0);
          System.out.println("Robot Stop");
        }

        //Foward information
        currentDistanceAwayFromTarget = distanceToTarget + leftModularEncoder.getDistance();
        motorPower = MathUtil.clamp(distancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

        //Moving to target
        theTank.drive(-motorPower, motorPower);
        System.out.println("Straight with motor power " + motorPower + " Distance from target: " + distanceToTarget + "Encoder Distance Covered: " + leftModularEncoder.getDistance());
      }
      info = ballTracker.getTargetGoal();
      distanceToTarget = info.get("Distance");
      degreeToTarget = info.get("Yaw");
  }
}
