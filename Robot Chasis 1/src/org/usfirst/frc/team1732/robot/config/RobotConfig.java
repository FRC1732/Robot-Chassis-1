package org.usfirst.frc.team1732.robot.config;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.xml.sax.SAXException;

/*
 * More stuff to recieve/load a config from network tables would be added to this class, but for now it will be kind of emptys
 */
public class RobotConfig {

	private static final String savedConfigPath = "config/RobotConfig.xml";
	private static final String tableName = "XML Configs";
	private static final String configTableEntry = "Robot Config";

	public static Document getSavedRobotConfig() throws SAXException, IOException, ParserConfigurationException {
		return ConfigUtils.makeDOMdoc(new File(savedConfigPath));
	}

}