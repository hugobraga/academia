package msc;

import java.util.ArrayList;
import java.util.Iterator;

public class Group {
	private static int GROUP_ID = 0;
	ArrayList<Set> sets;
	int id;
	int marked;
	
	public Group() {
		id = GROUP_ID++;
		marked = 0;
		sets = new ArrayList<Set>();
	}
	
	public void print() {
		System.out.println("imprimindo Grupo");
		for (int i = 0; i < sets.size(); i++) {
			System.out.println("set "+i);
			Iterator<ElkinItem> it = sets.get(i).iterator();
			int j = 0;
			while (it.hasNext()) {
				ElkinItem item = it.next();
				System.out.println("Item "+j+": pseudo-informed: "+item.getPseudo().getInformed().getId()+", pseudo-uninformed: "+item.getPseudo().getUninformed().getId()+", outro no: "+item.getNode().getId());
				j++;
			}
		}
		System.out.println("fim imprimindo Grupo");
	}
	
	public void addSet(Set set) {
		sets.add(set);
	}
	
	public Set getSet(int i) {
		return sets.get(i);
	}
	
	Iterator<Set> iterator() {
		return sets.iterator();
	}
	
	public int size() {
		return sets.size();
	}
	
	public int getId() {
		return id;
	}
	
	public int marked() {
		return marked;
	}
	
	public void mark() {
		marked = 1;
	}
	
	public void unMark() {
		marked = 0;
	}
}
