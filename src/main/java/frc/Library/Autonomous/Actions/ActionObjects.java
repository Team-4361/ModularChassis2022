package frc.Library.Autonomous.Actions;
import frc.Library.Chassis.*;

public class ActionObjects
{
	public static Chassis chassis;
	
	public static boolean HasEncoders()
	{
		return chassis != null && chassis.HasEncoder();
	}
}