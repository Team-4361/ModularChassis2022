package frc.Library.Autonomous.Actions;

public class ActionMulti implements Action
{
	private ActionList list;
	public ActionMulti(ActionList list)
	{
		this.list = list;
	}
	
	public void Run()
	{
		for(Action action : list)
		{
			action.Run();
		}
	}
	
	public boolean IsFinished()
	{
		if(list.Count() == 0)
			return true;
		
		return list.Get(0).IsFinished();
	}
}
