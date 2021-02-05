package frc.Library.Chassis;

public interface Chassis
{
	public void drive(double lVal, double rVal);
	public void Forward(double value);
	public void Straight(double value);
	public void Turn(double value);
	public void Stop();
	
	public double GetAverageDistance();
	public boolean HasEncoder();
	public void ResetEncoders();
}
