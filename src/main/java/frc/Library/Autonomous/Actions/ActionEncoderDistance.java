package frc.Library.Autonomous.Actions;

import frc.Library.Chassis.*;

public class ActionEncoderDistance implements Action
{
	double distance, speed;
	Chassis chassis = ActionObjects.chassis;
	
	public ActionEncoderDistance(double distance, double speed)
	{
		this.distance = distance;
		this.speed = speed;
		
		chassis.ResetEncoders();
	}
	
	public void Run()
	{
		chassis.Straight(speed);
	}
	
	public boolean IsFinished()
	{
		if(chassis.GetAverageDistance() > distance)
		{
			chassis.Stop();
			
			return true;
		}
		
		return false;
	}
}
