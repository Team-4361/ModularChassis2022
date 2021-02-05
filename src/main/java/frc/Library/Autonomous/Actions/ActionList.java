package frc.Library.Autonomous.Actions;

import java.util.*;

public class ActionList implements Iterable<Action>
{
	private List<Action> list;
	
	public ActionList()
	{
		list = new ArrayList<Action>();
	}
	
	public void Add(Action action)
	{
		list.add(action);
	}
	
	public void Add(ActionList list)
	{
		for(Action action : list)
		{
			this.list.add(action);
		}
	}
	
	public Action Get(int index)
	{
		return this.list.get(index);
	}
	
	public int Count()
	{
		return list.size();
	}
	
	public Iterator<Action> iterator()
	{
		return null;
	}
	
	
}
