package frc.Library.Util;
import java.io.*;
import java.util.ArrayList;

public class Constants extends ArrayList<Constant>
{
	private static final String PathFile = "/home/lvuser/Constants.txt";
	public boolean Loaded = false;
	
	public void LoadConstants()
	{
		this.clear();
		
		FileReader ConstFile;
		
		try
		{
			ConstFile = new FileReader(PathFile);
		}
		catch (FileNotFoundException ee)
		{
			System.out.println("File not found.");
			return;
		}
		
		BufferedReader Reader = new BufferedReader(ConstFile);
		
		try
		{
			String line;
			while((line = Reader.readLine()) != null)
			{
				if(!line.equals("") && !line.contains("///"))
				{
					String[] temp = line.split("=");
					Constant TempConst = new Constant(temp[0], temp[1]);

					System.out.println("Loaded " + TempConst.getName() + " : " + TempConst.getValue());
					this.add(TempConst);
				}
				else
					System.out.println(line);
			} 
			
			Reader.close();
			Loaded = true;
		}
		catch (Exception ee)
		{
			System.out.println("Problem reading file.");
			ee.printStackTrace();
		}
	}
	
	public int GetInt(String name)
	{
		for(Constant c : this)
		{
			if(c.getName().equals(name))
			{
				return Integer.parseInt(c.getValue());
			}
		}
		return 0;
	}
	
	public double GetDouble(String name)
	{
		for(Constant c : this)
		{
			if(c.getName().equals(name))
			{
				return Double.parseDouble(c.getValue());
			}
		}
		return 0.0;
	}
	
	public String GetString(String name)
	{
		for(Constant c : this)
		{
			if(c.getName().equals(name))
			{
				return c.getValue();
			}
		}
		return "";
	}
}