/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.photonvision.PhotonCamera;

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
import edu.wpi.first.wpilibj.util.Units;
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
  JoystickButton rightTriggerBTN = new JoystickButton(cannonControls, 8);
  JoystickButton leftTriggerBTN = new JoystickButton(cannonControls, 7);
  JoystickButton xBTN = new JoystickButton(cannonControls, 1);


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
  PIDController rotatingdistancePIDController;
  PIDController rotationPIDController;

  //DigitalInput rightLimit = new DigitalInput(6);
  DigitalInput leftLimit = new DigitalInput(8);


  Boolean ybtnReleased = false;
  Boolean abtnReleased = false;

  Boolean rightBumperReleased = false;
  Boolean leftBumperReleased = false;

  Boolean leftLimitReached = false;
  Boolean rightLimitReached =false;

  Boolean valveTimerStarted = false;
  Boolean runValveOpen = false;

  Timer valveTimer;

  int ticksToRightLimit;


  @Override
  public void robotInit() {
    ballTracker = new BallVisionCamera(networkTableName, cameraName, 0.4572, 0); 
    ticksToRightLimit = 145; //Something

    dataMap = new HashMap<String, Double>();

    rightModularEncoder = new Encoder(2,3);
    leftModularEncoder = new Encoder(0,1);
    rightModularEncoder.reset();
    leftModularEncoder.reset();

    //Distance per pulse is in meters
    leftModularEncoder.setDistancePerPulse(distancePerPulse);
    rightModularEncoder.setDistancePerPulse(distancePerPulse);

    // rightLimit = new DigitalInput(8);
    // leftLimit = new DigitalInput(5);

    valveTimer = new Timer();
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
      //System.out.println("Outside");
      if (/*rightLimit != null &&*/ leftLimit != null){

        if(!leftLimit.get()){
          //if(leftLimit.get() == false){
          leftLimitReached = true;
          if (leftBumper.get()/*xCont.getBumperPressed(GenericHID.Hand.kLeft)*/){
            modTalon1.set(1.0);
            leftBumperReleased = true;
            ticksToRightLimit += 1;
          }else if(!leftBumper.get() && leftBumperReleased/*xCont.getBumperReleased(GenericHID.Hand.kLeft)*/){
            modTalon1.set(0.0);
            leftBumperReleased = false;
            leftLimitReached = false;
          }
          //System.out.println("moving left");
        }
        else if(leftLimit.get() && leftLimitReached){
          leftLimitReached = false;
          modTalon1.set(0.0);
         // System.out.println("leftLimit running");
        }

        if(ticksToRightLimit > 0/*rightLimit.get()*/){
          //if(rightLimit.get() == false){
          rightLimitReached = true;
          if (rightBumper.get()/*xCont.getBumperPressed(GenericHID.Hand.kRight)*/){
            modTalon1.set(-1.0);
            rightBumperReleased = true;
            ticksToRightLimit -= 1;
          }else if(!rightBumper.get() && rightBumperReleased/*xCont.getBumperReleased(GenericHID.Hand.kRight)*/){
            modTalon1.set(0.0);
            rightLimitReached = false;
            rightBumperReleased = false;
          }
          //System.out.println("moving right");
        }
        else if(/*!rightLimit.get()*/(ticksToRightLimit <= 0) && rightLimitReached){
          modTalon1.set(0.0);
          rightLimitReached = false;
          //System.out.println("right limit running");
        }
        //System.out.println(rightLimit.get());

        System.out.println(ticksToRightLimit);
      }


      if (aBTN.get()){
        modTalon2.set(-0.45);
        abtnReleased = true;
      }
      else if(!aBTN.get() && abtnReleased){
        modTalon2.set(0.0);
        abtnReleased = false;
      }

      if (yBTN.get()){
        modTalon2.set(0.45);
        ybtnReleased = true;
      }
      else if(!yBTN.get() && ybtnReleased){
        modTalon2.set(0.0);
        ybtnReleased = false;
      }

      if(rightTriggerBTN.get() && leftTriggerBTN.get() && xBTN.get()){
        valveTimer.reset();
        valveTimer.start();
        runValveOpen = true;
      }

      if(runValveOpen){
        if(!valveTimer.advanceIfElapsed(1.0)){
          modTalon4.set(-1.0);
        }
        if(valveTimer.advanceIfElapsed(1.0)){
          modTalon4.set(0.0);
          runValveOpen = false;
        }
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
    distancePIDController = new PIDController(1,0,0);
    rotationPIDController = new PIDController(20,0,0);
    rotatingdistancePIDController = new PIDController(0.25,0,0);
    HashMap<String, Double> info = ballTracker.getTargetGoal();
    distanceToTarget = info.get("Distance");
    degreeToTarget = info.get("Yaw");
    leftModularEncoder.reset();
    rightModularEncoder.reset();
    shouldMoveFoward = false;
    continueMoving = true;
  }

  
  final double DEGREESTOMETERS = 0.00833333;
  double motorPower;
  double motorPowerToTurn;

  double currentDistanceAwayFromTarget;
  double currentTurningDistanceFromTarget;



  Boolean hasTurned = false;
  final double DISTANCETOTARGETFACTOR = 1.36d;







  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("photonvision");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  XboxController xboxController = new XboxController(0);
  
  //I assume the righhModularEncoder is positive.
  public void autonomousPeriodic() 
  {
    double forwardSpeed;
    double rotationSpeed;

        forwardSpeed = -1.0 * xboxController.getY(GenericHID.Hand.kRight);

        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = -1.0 * turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            rotationSpeed = xboxController.getX(GenericHID.Hand.kLeft);
        }

        // Use our forward/turn speeds to control the drivetrain
        theTank.drive(-forwardSpeed, forwardSpeed);
    }
      // //If ball is right of the robot
      // if( (degreeToTarget*DEGREESTOMETERS) - 0.04 > 0)
      // {
      //   hasTurned = true;

      //   //Rotate information
      //   currentTurningDistanceFromTarget = (degreeToTarget*DEGREESTOMETERS) - (-leftModularEncoder.getDistance() - rightModularEncoder.getDistance());
      //   motorPowerToTurn = MathUtil.clamp(rotationPIDController.calculate(currentTurningDistanceFromTarget, 0), -1.0, 1.0);
      
      //   //Foward information: May be faulty when it comes to math
      //   currentDistanceAwayFromTarget = (distanceToTarget*DISTANCETOTARGETFACTOR) - rightModularEncoder.getDistance();
      //   motorPower = MathUtil.clamp(rotatingdistancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

      //   //Moving to Target
      //   theTank.drive(MathUtil.clamp(-motorPowerToTurn-motorPower, -1.0, 1.0), motorPower);

      //   System.out.println("To Left excess | Left: "+ MathUtil.clamp(-motorPowerToTurn-motorPower, -1.0, 1.0) + " Right: " + motorPower + " Current distance to target: " + distanceToTarget*DISTANCETOTARGETFACTOR);
      //   //System.out.println("Moving left Current Distance To Target: " + currentDistanceAwayFromTarget + " Current Angle Away from Target: " + currentTurningDistanceFromTarget);
      // }
      // //If ball is left of the camera
      // else if(((-degreeToTarget*DEGREESTOMETERS) - 0.04 > 0))
      // {
      //   hasTurned = true;

      //   //Rotate information
      //   currentTurningDistanceFromTarget = (-degreeToTarget*DEGREESTOMETERS) - (rightModularEncoder.getDistance() + leftModularEncoder.getDistance());
      //   motorPowerToTurn = MathUtil.clamp(rotationPIDController.calculate(currentTurningDistanceFromTarget, 0), -1.0, 1.0);

      //   //Foward information
      //   currentDistanceAwayFromTarget = (distanceToTarget*DISTANCETOTARGETFACTOR) - rightModularEncoder.getDistance();
      //   motorPower = MathUtil.clamp(rotatingdistancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

      //   //Moving to Target
      //   theTank.drive(-motorPower, MathUtil.clamp(motorPowerToTurn+motorPower, -1.0, 1.0));

      //   System.out.println("To Right excess | Left: " + -motorPower + " Right: " + MathUtil.clamp(motorPowerToTurn+motorPower, -1.0, 1.0)  + " Current distance to target: " + distanceToTarget*DISTANCETOTARGETFACTOR);
      //   //System.out.println("Moving right Current Distance To Target: " + currentDistanceAwayFromTarget + " Current Angle Away from Target: " + currentTurningDistanceFromTarget);
      // }
      // else
      // {
      //   if(hasTurned){
      //     leftModularEncoder.reset();
      //     rightModularEncoder.reset();
      //     hasTurned = false;
      //   }

      //   //Foward information
      //   currentDistanceAwayFromTarget = (distanceToTarget*DISTANCETOTARGETFACTOR) - rightModularEncoder.getDistance();
      //   motorPower = MathUtil.clamp(distancePIDController.calculate(currentDistanceAwayFromTarget, 0), -1.0, 1.0);

      //   //Moving to target
      //   theTank.drive(-motorPower, motorPower);
      //   System.out.println("Straight with motor power " + motorPower + " Distance from target: " + distanceToTarget*DISTANCETOTARGETFACTOR + " Encoder Distance Covered: " + -leftModularEncoder.getDistance());
      //   //System.out.println("Moving Straight Current Distance To Target: " + currentDistanceAwayFromTarget + " Current Angle Away from Target: " + currentTurningDistanceFromTarget);
      // }
      
      // try {
      //   info = ballTracker.getTargetGoal();
      // } catch (NullPointerException e) {
      //   System.out.println("Lost sight of a target");
      //   theTank.drive(0, 0);
      // }
        
      //   distanceToTarget = info.get("Distance");
      //   degreeToTarget = info.get("Yaw");
  //}
}
