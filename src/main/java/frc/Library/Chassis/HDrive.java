package frc.Library.Chassis;
import frc.Library.Controllers.Drive;

public class HDrive extends TankDrive implements Strafe
{
	Drive Middle;
	public HDrive(Drive Left, Drive Right, Drive Middle)
	{
		super(Left, Right);
		this.Middle = Middle;
	}
	
	public void drive(double lVal, double rVal, double mVal)
	{
		super.drive(lVal, rVal);
		Middle.drive(mVal);
	}

	public void Strafe(double val)
	{
		Middle.drive(val);
	}
	
	public void Stop()
	{
		super.Stop();
		Middle.drive(0);
	}
}
