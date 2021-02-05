package frc.Library.Autonomous;

import frc.Library.Util.*;
import frc.Library.Chassis.*;
import frc.Library.Controllers.TurnControl;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;


public class AutonomousMethods
{
	//Autonomous Variables
	Counter RunNum;
	boolean hasRun;
	double circumference, robotWidth;
	Timer timer;
	
	//Controls
	Chassis chassis;
	
	boolean isEnc = false;
	public boolean isReversed = false, isNavx = true;
	
	public TurnControl turnControl;
	
	public AutonomousMethods(Counter RunNum, double circumference, boolean isNavx, Chassis chassis)
	{
		this( RunNum, circumference, isNavx, chassis, new TurnControl());
	}

	public AutonomousMethods(Counter RunNum, double circumference, boolean isNavx, Chassis chassis, TurnControl turnControl)
	{
		this.RunNum = RunNum;
		hasRun = false;
		this.circumference = circumference;
		timer = new Timer();
		
		this.chassis = chassis;
		
		this.isNavx = isNavx;
		
		this.turnControl = turnControl;
		
		isEnc = chassis.HasEncoder();
	}
	
	//Autonomous Commands
	public void goDistance(double dist, double speed)
	{
		double timeWarm = .5;
		double timeNeeded = timeWarm + ((dist / circumference) / ((speed * 5310) / (60 * 12.75)));
		
		if(!hasRun)
		{
			chassis.Forward(speed);
			timer.start();
		}

		chassis.Straight(speed);
		
		if(isEnc)
		{
			if(!hasRun)
			{
				chassis.ResetEncoders();
				hasRun = true;
			}
			
			double large = chassis.GetAverageDistance() / 256;
			
			chassis.Straight(speed);
			
			if(large * circumference > dist || timer.get() - 7 > timeNeeded)
			{
				System.out.println("Stop");
				
				chassis.Stop();
				
				hasRun = false;
				
				RunNum.Add();
			}
		}
		
		//For when the encoders break
		else if(!isEnc)
		{
			int dist2 = 0;
			
			if(!hasRun)
			{
				timer.start();
				hasRun = true;
			}
			
			if(timer.get()>timeNeeded)
			{
				chassis.Stop();
				
				hasRun = false;
				RunNum.Add();
				timeNeeded = 0;
				
				timer.stop();
				timer.reset();
			}
		}
	}

	public void turn(double angle, double speed)
	{
		if(isReversed) angle = -angle;
		
		if(isNavx)
			turnNavx(angle, speed);
		else
			turnEncoder(angle, speed);
	}
	
	private void turnEncoder(double angle, double speed)
	{
		double percent = Math.abs(angle)/360;
		if(!hasRun)
		{
			chassis.ResetEncoders();
		}
		if(!hasRun&&angle<0)
		{
			chassis.Turn(speed);
			hasRun = true;
		}
		else if(!hasRun&&angle>0)
		{
			chassis.Turn(-speed);
			hasRun = true;
		}
		else if(!hasRun&&angle==0)
			hasRun=true;
		
		double large = chassis.GetAverageDistance() * 255;
		
		if(large*circumference >= (19.5*Math.PI)*percent)
		{
			chassis.Stop();
			
			hasRun = false;
			RunNum.Add();
		}
	}

	private void turnNavx(double angle, double MaxSpeed)
	{
		if(!hasRun)
		{
			turnControl.SetSpeed(MaxSpeed);
			turnControl.SetFromPosition(angle);

			hasRun = true;
		}
	  
		double RotateRate = turnControl.GetRotateRate();
		chassis.Turn(RotateRate);
		
		if(turnControl.onTarget())
		{
			if(timer.get() == 0)
			{
				timer.reset();
				timer.start();
			}
			if(timer.get() > .25)
			{
				chassis.Stop();
				
				timer.stop();
				timer.reset();
				
				hasRun = false;
				RunNum.Add();
			}
		}
		else
		{
			timer.stop();
			timer.reset();
		}
	}
	
	public void wait(double time)
	{
		if(!hasRun)
		{
			timer.start();
			hasRun = true;
		}
		if(timer.get() >= time && hasRun)
		{
			timer.stop();
			timer.reset();
			hasRun = false;
			RunNum.Add();
		}
	}
	
}
