package frc.Library.Controls;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickArcade extends Joystick
{
	public JoystickArcade(int pos)
	{
		super(pos);
	}
	
	public double[] GetDriveValue()
	{
		double[] val = new double[2];
		//Left
		val[0] = getX() - getY();
		//Right
		val[1] = getX() + getY();
		
		return val;
	}
}
