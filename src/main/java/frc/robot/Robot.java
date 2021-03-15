/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static int mode = 1;
  SendableChooser<String> modularMode = new SendableChooser<>();
  WPI_TalonSRX lDrive1 = new WPI_TalonSRX(1);
  WPI_TalonSRX lDrive2 = new WPI_TalonSRX(2);
  WPI_TalonSRX[] lDriveMotors = { lDrive1, lDrive2 };
  Drive lDrive = new Drive(lDriveMotors);
  WPI_TalonSRX rDrive1 = new WPI_TalonSRX(4);
  WPI_TalonSRX rDrive2 = new WPI_TalonSRX(6);
  WPI_TalonSRX[] rDriveMotors = { rDrive1, rDrive2 };
  Drive rDrive = new Drive(rDriveMotors);
  TankDrive theTank = new TankDrive(lDrive, rDrive);
  Joystick lStick = new Joystick(0);
  Joystick rStick = new Joystick(1);

  XboxController xCont = new XboxController(2);
  XboxArcade xContArCon = new XboxArcade(2, Hand.kLeft);
  // snowblower motor for frisbee shooter
  WPI_TalonSRX modTalon1 = new WPI_TalonSRX(3);
  // CIM motor for frisbee shooter
  WPI_TalonSRX modTalon2 = new WPI_TalonSRX(5);
  // modular talon 3
  WPI_TalonSRX modTalon3 = new WPI_TalonSRX(7);
  // modular talon 4
  WPI_TalonSRX modTalon4 = new WPI_TalonSRX(8);

  Shooter shooter = new Shooter(modTalon1, modTalon2);

  BallVisionCamera ballTracker;
  final String networkTableName = "photonvision";
  final String cameraName = "RoxCam2021-4361";

  HashMap<String, Double> dataMap;

  //The encoder is an E4P encoder
  Encoder modularEncoder;
  //E4P does 1000 pulses per rotation
  double distanceToTarget;
  double degreeToTarget;

  Boolean shouldMoveFoward;
  Boolean continueMoving;
  int cycleFinished;

  @Override
  public void robotInit() {
    ballTracker = new BallVisionCamera(networkTableName, cameraName, 0.3175, 0);

    dataMap = new HashMap<String, Double>();

    modularEncoder = new Encoder(0, 1);
    modularEncoder.reset();
    //Distance per pulse is in meters
    modularEncoder.setDistancePerPulse(0.63837/250.0); //try 250 and 500 for the pulse per rotation || Try 0.638371596 for the distance per rotation
    
    // taskTimer = new TimerTask(){

    // @Override
    // public void run() {
    // // TODO Auto-generated method stub
    // HashMap<String, Double> dataMap = new HashMap<String, Double>();

    // dataMap = ballTracker.getTargetGoal();
    // System.out.println("The yaw is " + dataMap.get("Yaw") + " and the distance to
    // the ball is" + dataMap.get("Distance"));
    // }

    // };

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
      boolean spinnyThingSpinningQuestionMark = false;
      boolean indexThingSpinningQuestionMark = false;
      if (xCont.getAButtonPressed()) {
        spinnyThingSpinningQuestionMark = !spinnyThingSpinningQuestionMark;
        if (spinnyThingSpinningQuestionMark) {
          modTalon3.set(1.0);
        } else if (!spinnyThingSpinningQuestionMark) {
          modTalon3.set(0.0);
        }
      }

      if (xCont.getBButtonPressed()) {
        modTalon4.set(1.0);
      } else if (xCont.getBButtonReleased()) {
        modTalon4.set(0.0);
      }

      if (xCont.getXButtonPressed()) {
        indexThingSpinningQuestionMark = !indexThingSpinningQuestionMark;
        if (indexThingSpinningQuestionMark) {
          modTalon2.set(1.0);
        } else if (!indexThingSpinningQuestionMark) {
          modTalon2.set(0.0);
        }
      }
    }

    if (mode == 4)// xbox tank/arcade control
    {
      theTank.drive(xContArCon.GetDrive());

    }

    System.out.println(modularEncoder.getDistance());

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

  HashMap<String, Double> info;

  public void autonomousInit() {
    HashMap<String, Double> info = ballTracker.getTargetGoal();
    distanceToTarget = info.get("Distance");
    degreeToTarget = info.get("Yaw");
    modularEncoder.reset();
    shouldMoveFoward = false;
    continueMoving = true;
    cycleFinished = 0;
  
  }

  //Ticks per revolution
  //5330RPM Free Speed
  //Calculate how far the wheel would go if it rotated once
  //Use the encoder to find out when it makes one revolution
  /*
    Initially: Find the Yaw and rotate to that point
                Find the distance and start moving
                Store the distance
                ____________________________________ Go a quarter of the way using the distance
    Second Step: Recaclulate the Yaw of that target or find a closer target and get the yaw
                  Find the distance and start moving
                  _______________________________Go a half of the way using the distance calculated from this step 
                  find out what it would be if had started
    Third Step: Repeat for half and three quarter until the target is reached
  */

  //0.00833333 repeating per degree
  
  public void autonomousPeriodic() 
  {

      //If ball is left of the robot
      if((modularEncoder.getDistance()-0.10 < -degreeToTarget*0.008333333D) && !shouldMoveFoward && continueMoving)
      {
        theTank.drive(-0.9, -0.9);
      }
      //If ball is right of the camera
      else if((-modularEncoder.getDistance()-0.10 < degreeToTarget*0.008333333D) && !shouldMoveFoward && continueMoving)
      {
        theTank.drive(0.9, 0.9);
      }
      else if((modularEncoder.getDistance() < distanceToTarget/4) && shouldMoveFoward && continueMoving)
      {
        theTank.drive(0.4, -0.4);
      }
      else
      {
        if(continueMoving)
        {
          theTank.drive(0, 0);
          shouldMoveFoward = !shouldMoveFoward;
          cycleFinished++;
          modularEncoder.reset();
          if(cycleFinished%2 == 0)
          {
            info = ballTracker.getTargetGoal();
            distanceToTarget = info.get("Distance");
            degreeToTarget = info.get("Yaw");
          }

          if(distanceToTarget < 0.04)
          {
            continueMoving =false;
          }
        }
      }
      
  }


  /*
    yawAwayFromBall is in degrees
  */
  public void adjustRobotToBallRotation(double yawAwayFromBall)
  {
    // while(!(yawAwayFromBall >= -8)  && !(yawAwayFromBall <= 8))
    // {
      if(yawAwayFromBall < -8)
      {
        theTank.drive(-0.7, -0.7);  
      }
    
      if(yawAwayFromBall > 8)
      { 
        theTank.drive(0.7, 0.7);
      }
    //}
  }
  public void adjustRobotToBallPosition()
  {
    
  }
}


