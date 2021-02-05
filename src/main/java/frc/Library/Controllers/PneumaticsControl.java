package frc.Library.Controllers;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class PneumaticsControl
{
	private Compressor comp;
	private AnalogInput Sensor;
	private double Offset;
	
	public PneumaticsControl(int comp)
	{
		this.comp = new Compressor(comp);
		SmartDashboard.putBoolean("PneumaticControl", SmartDashboard.getBoolean("PneumaticControl", false));
	}
	
	public PneumaticsControl(int comp, int PressSensor, double Offset)
	{
		this(comp);
		Sensor = new AnalogInput(PressSensor);
		this.Offset = Offset;
	}
	
	public void DisplayCompressor()
	{
		SmartDashboard.putData("Compressor", comp);
	}
	
	public void DisplayVoltage()
	{
		SmartDashboard.putNumber("Pressure", Sensor.getVoltage());
	}
	
	public void DisplayPSI()
	{
		SmartDashboard.putNumber("Pressure", GetPSI());
	}
	
	public double GetPSI()
	{
		return 250.0 * (Sensor.getVoltage() / Offset) - 25.0;
	}
	
	public Compressor GetCompressor()
	{
		return comp;
	}
	
	public void SystemSwitch()
	{
		boolean forceSwitch = SmartDashboard.getBoolean("PneumaticControl", false);
		double PSI = GetPSI();
		if(!forceSwitch)
		{
			ChangeCompressorState(false);
		}
		else if(PSI < 80)
		{
			ChangeCompressorState(true);
		}
		else if(PSI > 110)
		{
			ChangeCompressorState(false);
		}
		
	}
	
	public void ChangeCompressorState(boolean state)
	{
		comp.setClosedLoopControl(state);
	}
}
