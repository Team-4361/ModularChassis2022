package frc.Library.Chassis;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Encoder;
import frc.Library.Controllers.*;
import frc.Library.Util.Constant;

public class TankDrive implements Chassis
{
	Drive Left, Right;
	Encoder lEnc, rEnc;
	double DIAMETER;
	TalonEncoder lTalonEnc, rTalonEnc;

	boolean isTalonEncoder;
	
	public TankDrive(Drive Left, Drive Right)
	{
		this.Left = Left;
		this.Right = Right;
	}
	
	public TankDrive(Drive Left, Drive Right, Encoder lEnc, Encoder rEnc, double diameter)
	{
		this(Left, Right);
		
		this.lEnc = lEnc;
		this.rEnc = rEnc;
		this.DIAMETER = diameter;

		isTalonEncoder = false;
	}
	
	public TankDrive(Drive Left, Drive Right, TalonEncoder lTalonEnc, TalonEncoder rTalonEnc, double diameter)
	{
		this(Left, Right);

		this.lTalonEnc = lTalonEnc;
		this.rTalonEnc = rTalonEnc;
		this.DIAMETER = diameter;

		isTalonEncoder = true;
	}
	
	public void Forward(double value)
	{
		drive(value, -value);
	}
	
	public void Straight(double value)
	{
		if(!HasEncoder())
			Forward(value);
		
		double SpeedChange = .1 * value;
		
		if(getLeftDistance() > getRightDistance())
			drive(value - SpeedChange, -value);
		else if(getRightDistance() < getLeftDistance())
			drive(value, -(value - SpeedChange));
		else
			Forward(value);
	}
	
	public void Turn(double value)
	{
		Left.drive(value);
		Right.drive(value);
	}
	
	public double GetAverageDistance()
	{
		if(!HasEncoder())
			return 0;
		
		return (getLeftDistance() + getRightDistance())/2;
	}
	public double getLeftDistance()
	{
		if(!HasEncoder())
			return 0;
		
		//Change if there is talon encoders or not
		double distance;
		if(isTalonEncoder)
			distance = Math.abs(lTalonEnc.getDistance());
		else
			distance = Math.abs(lEnc.getDistance());

		return distance * DIAMETER*Math.PI;
	}
	public double getRightDistance()
	{
		if(!HasEncoder())
			return 0;
		
		//Change if there is talon encoders or not
		double distance;
		if(isTalonEncoder)
			distance = Math.abs(rTalonEnc.getDistance());
		else
			distance = Math.abs(rEnc.getDistance());

		return distance * DIAMETER*Math.PI;
	}
	
	public void Stop()
	{
		Left.drive(0);
		Right.drive(0);
	}
	
	public void drive(double lVal, double rVal)
	{
		Left.drive(lVal);
		Right.drive(rVal);
	}

	public void drive(double[] values)
	{
		Left.drive(values[0]);
		Right.drive(values[1]);
	}
	
	public boolean HasEncoder()
	{
		if(isTalonEncoder)
			return !(lTalonEnc == null || rTalonEnc == null);
		else
			return !(lEnc == null || rEnc == null);
	}
	
	public void ResetEncoders()
	{
		if(isTalonEncoder)
		{
			lTalonEnc.resetEncoder();
			rTalonEnc.resetEncoder();
		}
		else
		{
			lEnc.reset();
			rEnc.reset();
		}
	}
}
  