package frc.Library.Util;

public class Change
{
	boolean currentState = true;
	
	public boolean State(boolean value)
	{
		boolean state;
		
		if(currentState && value)
		{
			state = true;
		}
		else
		{
			state = false;
		}
		
		
		if(currentState && value)
		{
			currentState = false;
		}
		
		if(!currentState && !value)
		{
			currentState = true;
		}
		
		return state;
	}
}
