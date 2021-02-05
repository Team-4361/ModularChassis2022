/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public class Robot extends TimedRobot
{
  //if mode = 1; frisbee shooter
  //if mode = 2; thor's hammer
  //if mode = 3; romulus.
  //if mode = 4; Xbox tank/arcade drive control (no operator)
  public static int mode = 1;
  SendableChooser <String> modularMode = new SendableChooser<>();
  WPI_TalonSRX lDrive1 = new WPI_TalonSRX(1);
  WPI_TalonSRX lDrive2 = new WPI_TalonSRX(2);
  WPI_TalonSRX[] lDriveMotors = {lDrive1,lDrive2};
  Drive lDrive = new Drive(lDriveMotors);
  WPI_TalonSRX rDrive1 = new WPI_TalonSRX(4);
  WPI_TalonSRX rDrive2 = new WPI_TalonSRX(6);
  WPI_TalonSRX[] rDriveMotors = {rDrive1,rDrive2};
  Drive rDrive = new Drive(rDriveMotors);
  TankDrive theTank = new TankDrive(lDrive, rDrive);
  Joystick lStick = new Joystick(0);
  Joystick rStick = new Joystick(1);


  XboxController xCont = new XboxController(2);
  XboxArcade xContArCon = new XboxArcade(2, Hand.kLeft);
  //snowblower motor for frisbee shooter
  WPI_TalonSRX modTalon1 = new WPI_TalonSRX(3);
  //CIM motor for frisbee shooter
	WPI_TalonSRX modTalon2 = new WPI_TalonSRX(5);
	//modular talon 3
	WPI_TalonSRX modTalon3 = new WPI_TalonSRX(7);
	//modular talon 4
	WPI_TalonSRX modTalon4 = new WPI_TalonSRX(8);

  Shooter shooter = new Shooter(modTalon1, modTalon2);

  @Override
  public void robotInit()
  {
    
  }

  @Override
  public void teleopPeriodic()
  {
    /*double[] stickVal = sticks.GetDrive();
    stickVal[0] = stickVal[0]*-.8;
    stickVal[1] = stickVal[1]*-.8;*/
    
    

    if(mode == 1)
    {
      theTank.drive(-lStick.getY(),rStick.getY());

      //FRISBEE SHOOTER MODE
      if(xCont.getXButtonReleased())
      {
        System.out.println("pffffft 0");
        modTalon2.set(0);
      }
      if(xCont.getXButtonPressed())
      {
        System.out.println("pffffft 1");
        modTalon2.set(1);
      }
  
      if(xCont.getBButtonReleased())
      {
        System.out.println("tsktsktsk 0");
        modTalon1.set(0);
      }
      if(xCont.getBButtonPressed())
      {
        System.out.println("tsktsktsk 1");
        modTalon1.set(1);
      }
    }
    if(mode == 2)
    {
      theTank.drive(-lStick.getY(),rStick.getY());

			//HAMMER MODE
      if(xCont.getYButtonPressed())
      {
        System.out.println("hammer BACK");
        modTalon1.set(-1);
        modTalon2.set(-1);
      }
      if(xCont.getYButtonReleased())
      {
        System.out.println("hammer SETTLE");
        modTalon1.set(0);
        modTalon2.set(0);
      }
      if(xCont.getAButtonPressed())
      {
        System.out.println("hammer FORWARD");
        modTalon2.set(1);
        modTalon1.set(1);
      }
      if(xCont.getAButtonReleased())
      {
        System.out.println("hammer SETTLE 2");
        modTalon2.set(0);
        modTalon1.set(0);
      }
    }
    if (mode == 3)
    {
      theTank.drive(-lStick.getY(),rStick.getY());

			//BALL SHOOTER
			boolean spinnyThingSpinningQuestionMark = false;
			boolean indexThingSpinningQuestionMark = false;
			if(xCont.getAButtonPressed())
			{
				spinnyThingSpinningQuestionMark = !spinnyThingSpinningQuestionMark;
				if(spinnyThingSpinningQuestionMark)
				{
					modTalon3.set(1.0);
				}
				else if(!spinnyThingSpinningQuestionMark)
				{
					modTalon3.set(0.0);
				}
			}
			
			if(xCont.getBButtonPressed())
			{
				modTalon4.set(1.0);
			}
			else if(xCont.getBButtonReleased())
			{
				modTalon4.set(0.0);
			}

			if(xCont.getXButtonPressed())
			{
				indexThingSpinningQuestionMark = !indexThingSpinningQuestionMark;
				if(indexThingSpinningQuestionMark)
				{
					modTalon2.set(1.0);
				}
				else if(!indexThingSpinningQuestionMark)
				{
					modTalon2.set(0.0);
				}
			}
    }
    
    if (mode==4)//xbox tank/arcade control
    {
      theTank.drive(xContArCon.GetDrive());


    }




		//Smartdashboard Values

		try
		{
			//SmartDashboard.putNumber("Agitator Current", CAN[4].getOutputCurrent());
			//SmartDashboard.putNumber("Shooter Current", CAN[1].getOutputCurrent());
			//SmartDashboard.putNumber("Climber Current", CAN[5].getOutputCurrent());
			//SmartDashboard.putBoolean("Geared", gearing);
			//SmartDashboard.putBoolean("GearIn", !limit[0].get());
			//SmartDashboard.putNumber("Pusher Current", CAN[9].getOutputCurrent());

			//SmartDashboard.putBoolean("Xbox Control Mode", XboxMode);
		}
		catch (Exception e)
		{
			//System.out.println("Smartdashboard: " + e.getMessage());
		}
		
  }
}
