package frc.Library.Controllers;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import frc.Library.Util.*;

public class TurnControl implements PIDOutput
{
	
	private double rotateToAngleRate;
	private AHRS navx;

	private PIDController turnController;
	
	static double kP = 0.00;
	static double kI = 0.00;
	static double kD = 0.00;
	static double kF = 0.00;
	static double kToleranceDegrees = 0;

	private float YOffset = 0;
	
	public TurnControl()
	{
		Constants cons = new Constants();
		cons.LoadConstants();
		
		kP = cons.GetDouble("kP");
		kI = cons.GetDouble("kI");
		kD = cons.GetDouble("kD");
		kF = cons.GetDouble("kF");
		kToleranceDegrees = cons.GetDouble("kToleranceDegrees");
		
		
		try 
		{
			navx = new AHRS(SPI.Port.kMXP);
		}
		catch (RuntimeException ex )
		{
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		
		turnController = new PIDController(kP, kI, kD, kF, navx, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		Enable();
	}
	public TurnControl(double kP, double kI, double kD, double kF, double kToleranceDegrees)
	{
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kToleranceDegrees = kToleranceDegrees;
		
		
		try 
		{
			navx = new AHRS(SPI.Port.kMXP);
		}
		catch (RuntimeException ex )
		{
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		
		turnController = new PIDController(kP, kI, kD, kF, navx, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		Enable();
	}
	
	public void ResetNavx()
	{
		navx.reset();
	}
	
	public void SetFromPosition(double value)
	{
		ResetNavx();
		SetAngle(value);
	}
	
	public void SetAngle(double value)
	{
		turnController.setSetpoint(value * 1f);
	}
	
	public double GetAngle()
	{
		return navx.getAngle();
	}
	
	public AHRS GetNavx()
	{
		return navx;
	}
	
	public PIDController GetPIDController()
	{
		return turnController;
	}
	
	public void SetSpeed(double speed)
	{
		turnController.setOutputRange(-speed, speed);
	}
	
	public double GetRotateRate()
	{
		return rotateToAngleRate;
	}
	
	public boolean onTarget()
	{
		return turnController.onTarget();
	}
	
	public void Enable()
	{
		turnController.enable();
	}
	
	public void Disable()
	{
		turnController.disable();
	}
	
	public void pidWrite(double output)
	{
		rotateToAngleRate = output;
	}

	public float getYAxis()
	{
		return navx.getPitch() - YOffset;
	}

	public void setYOffset(float offset)
	{
		YOffset = offset;
	}

	public void setYOffset()
	{
		YOffset = navx.getPitch();
	}

	public float getYOffset()
	{
		return YOffset;
	}
}
