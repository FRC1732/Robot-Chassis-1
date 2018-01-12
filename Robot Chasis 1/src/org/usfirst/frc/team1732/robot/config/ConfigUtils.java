package org.usfirst.frc.team1732.robot.config;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Result;
import javax.xml.transform.Source;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.TransformerFactoryConfigurationError;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

/*
 * I got a lot of this code from here: http://www.mkyong.com/java/how-to-read-xml-file-in-java-dom-parser/
 * 
 * Tutorial for xml notation: https://www.w3schools.com/xml/xml_usedfor.asp
 * 
 * This is tools for creating and saving a config
 */
public class ConfigUtils {

	/*
	 * Makes a DOM document from the text of an XML file
	 * 
	 * This method would be used when receiving the XML file as a string through the
	 * network tables
	 * 
	 * The caller should handle exceptions if they happen and stop the robot program
	 * from dying.
	 * 
	 * Because this is only for receiving the xml files from the driverstation, if
	 * an exception happens it is probably a problem with the syntax of the xml file
	 * and should be fixed on the driverstation then tried again
	 */
	public static Document makeDOMdoc(String XMLFileText)
			throws ParserConfigurationException, SAXException, IOException {
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		InputSource source = new InputSource(new StringReader(XMLFileText));

		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(source);
		doc.getDocumentElement().normalize();

		return doc;
	}

	public static Document makeDOMdoc(File XMLfile) throws SAXException, IOException, ParserConfigurationException {
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();

		DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		Document doc = dBuilder.parse(XMLfile);
		doc.getDocumentElement().normalize();

		return doc;
	}

	public static String fileToString(String pathToFile) throws IOException {
		byte[] encoded = Files.readAllBytes(Paths.get(pathToFile));
		return new String(encoded, Charset.defaultCharset());
	}

	/*
	 * Makes a new xml file, or overwrites existing file
	 */
	public static void saveDom(Document doc, String pathToFile, String name)
			throws TransformerFactoryConfigurationError, TransformerException {
		Transformer transformer = TransformerFactory.newInstance().newTransformer();
		Result output = new StreamResult(new File(pathToFile + name));
		Source input = new DOMSource(doc);

		transformer.transform(input, output);
	}

	/*
	 * Returns the first element within the document with that name
	 */
	public static Element getElement(Document doc, String elementName) {
		return (Element) doc.getElementsByTagName(elementName).item(0);
	}

	/*
	 * Returns the first element within the top element with that name
	 */
	public static Element getElement(Element top, String elementName) {
		return (Element) top.getElementsByTagName(elementName).item(0);
	}

	public static String getText(Element top, String elementName) {
		return getElement(top, elementName).getTextContent();
	}

	public static int getInteger(Element top, String elementName) {
		return Integer.parseInt(getText(top, elementName));
	}

	public static boolean getBoolean(Element top, String elementName) {
		return Boolean.parseBoolean(getText(top, elementName));
	}

	public static double getDouble(Element top, String elementName) {
		return Double.parseDouble(getText(top, elementName));
	}
}