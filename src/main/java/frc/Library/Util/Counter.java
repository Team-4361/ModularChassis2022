package frc.Library.Util;

public class Counter 
{
	private int count = 0;

	public Counter()
	{
		
	}
	
	public void Add()
	{
		count++;
	}
	
	public void Subract()
	{
		count--;
	}
	
	public void Set(int value)
	{
		count = value;
	}
	
	public int Get()
	{
		return count;
	}
	
	public void Reset()
	{
		count = 0;
	}
}
