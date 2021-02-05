package frc.Library.Util;

public class Constant {
	
	public static Constants AllConstant;
	
	
	private String Name;
	private String Value;
	
	public Constant(String Name, String Value)
	{
		this.Name = Name;
		this.Value = Value;
	}
	
	public String getName()
	{
		return Name;
	}
	
	public String getValue()
	{
		return Value;
	}
	
	public static Constants GetConstants()
	{
		Constants cons = new Constants();
		cons.LoadConstants();
		
		return cons;
	}
}
