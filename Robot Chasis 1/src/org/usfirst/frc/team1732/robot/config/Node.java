package org.usfirst.frc.team1732.robot.config;

import java.util.ArrayList;

public class Node {

	private Object data;
	private String name;
	private ArrayList<Node> children;

	public Node(String name) {
		this(name, null);
	}

	public Node(String name, Object data) {
		this.data = data;
		this.name = name;
		children = new ArrayList<Node>();
	}

	private void addSingleData(String child, Object data) {
		addNode(new Node(child, data));
	}

	public Node addData(Object... data) {
		for (int i = 0; i < data.length; i += 2) {
			addSingleData((String) data[i], data[i + 1]);
		}
		return this;
	}

	public Node addData(Node data) {
		addNode(data);
		return this;
	}

	public Node addNode(Node child) {
		if (getNode(child.name) != null)
			return this;
		children.add(child);
		return this;
	}

	public Node addNode(String child) {
		Node n = new Node(child);
		children.add(n);
		return n;
	}

	public Node getNode(String child) {
		for (Node n : children) {
			if (n.name.equals(child)) {
				return n;
			}
		}
		return null;
	}

	public <T> T getData(String child) {
		return getNode(child).getOwnData();
	}

	public <T> T getOwnData() {
		return (T) data;
	}

}