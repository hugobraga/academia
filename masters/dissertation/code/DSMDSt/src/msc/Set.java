package msc;

import java.util.ArrayList;
import java.util.Iterator;

public class Set {
	ArrayList<ElkinItem> items;
	
	public Set() {
		items = new ArrayList<ElkinItem>();
	}
	
	public void addItem(ElkinItem item) {
		items.add(item);
	}
	
	public int size() {
		return items.size();
	}
	
	public ArrayList<ElkinItem> getItems() {
		return items;
	}
	
	Iterator<ElkinItem> iterator() {
		return items.iterator();
	} 
	
	public void print() {
		System.out.println("imprimindo Set");
		int j = 0;
		Iterator<ElkinItem> it = iterator();
		while (it.hasNext()) {
			ElkinItem item = it.next();
			if (item.getPseudo() == null)
				System.out.println("outro no: "+item.getNode().getId());
			else
				System.out.println("Item "+j+": pseudo-informed: "+item.getPseudo().getInformed().getId()+", pseudo-uninformed: "+item.getPseudo().getUninformed().getId()+", outro no: "+item.getNode().getId());
			j++;
		}
		System.out.println("fim imprimindo Set");
	}
	
	public boolean contain(ElkinItem item) {
		for (int i = 0; i < items.size(); i++) {
			//if (items.get(i).)
		}
		return true;
	}
	
	public int intersection(ArrayList<ElkinItem> set2Comp/*, int max*/) {
		Iterator<ElkinItem> it = set2Comp.iterator();
		int counter = 0;
		ArrayList<ElkinItem> list2Print = new ArrayList<ElkinItem>();
		while (it.hasNext()) {
			ElkinItem item = it.next();
			if (items.contains(item)) {
				list2Print.add(item);
				counter++;
			}
		}
		return counter;
	}
}
