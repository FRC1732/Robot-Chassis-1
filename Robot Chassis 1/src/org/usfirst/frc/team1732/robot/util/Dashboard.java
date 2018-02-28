package org.usfirst.frc.team1732.robot.util;

import java.util.LinkedList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {

	public Dashboard() {
		new Thread(this::loop).start();
	}

	private void sleepExactly() {
		try {
			Thread.sleep(40);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	private void loop() {
		while (true) {
			entries.forEach(this::call);
			sleepExactly();
		}
	}
	
	private void call(Entry e) {
		e.putToDashboard();
	}
	
	private static LinkedList<Entry> entries = new LinkedList<>();
	
	public void add(String name, Supplier<?> sup) {
		entries.add(new Entry(name, sup));
	}
	
	private class Entry {
		private final String name;
		private final Supplier<?> sup;
		public Entry(String name, Supplier<?> sup) {
			this.name = name;
			this.sup = sup;
		}
		
		public void putToDashboard() {
			Object o = sup.get();
			if(o instanceof Number) {
				SmartDashboard.putNumber(name, ((Number) o).doubleValue());
			}else if(o instanceof Boolean) {
				SmartDashboard.putBoolean(name, (Boolean) o);
			}else if(o instanceof String) {
				SmartDashboard.putString(name, (String) o);
			}else if(o instanceof Sendable) {
				SmartDashboard.putData((Sendable) o);
			}
		}
	}
}
