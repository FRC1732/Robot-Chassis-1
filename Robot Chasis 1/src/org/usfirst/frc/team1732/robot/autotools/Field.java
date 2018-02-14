package org.usfirst.frc.team1732.robot.autotools;

import java.awt.geom.Rectangle2D;

/*
 * Class for data about the field, like coordinates of stuff
 * Where everything SHOULD be
 * 
 * Origin is close left corner when standing at driverstation
 * positive X direction is going outwards
 * positive Y direction is towards right side
 * Put in default coordinates
 * 
 * All units in inches
 * 
 * Some assumptions coordinate system is based on:
 * 1. DriverWallToSideWall is correct. This being wrong at competition will shift the whole Y axis (doesn't matter if we don't drive close to the wall)
 * 2. Driverstation wall - make sure it isn't tilted
 */
public class Field {

	// private static final double DriverWallToSideWall = 29.69;

	public final Scale SCALE = new Scale();
	public final Switch SWITCH = new Switch();
	public final Zones ZONES = new Zones();

	public class Scale {
		public final Rectangle2D LEFT_PLATE = new Rectangle2D.Double(299.65, 72, 48, 36);
		public final Rectangle2D RIGHT_PLATE = new Rectangle2D.Double(299.65, 216, 48, 36);
		public final Rectangle2D PLATFORM = new Rectangle2D.Double(261.47, 97, 54, 130);
		public final Rectangle2D LEFT_NULL_ZONE = new Rectangle2D.Double(288, 0, 72, 95);
		public final Rectangle2D RIGHT_NULL_ZONE = new Rectangle2D.Double(288, 229, 72, 95);
	}

	public class Switch {
		public final Rectangle2D LEFT_PLATE = new Rectangle2D.Double(140, 85, 56, 44);
		public final Rectangle2D RIGHT_PLATE = new Rectangle2D.Double(140, 195, 56, 44);
		public Rectangle2D[] CUBES = { new Rectangle2D.Double(195, 85, 13, 13),
				new Rectangle2D.Double(195, 113, 13, 13), new Rectangle2D.Double(195, 141, 13, 13),
				new Rectangle2D.Double(195, 169, 13, 13), new Rectangle2D.Double(195, 197, 13, 13),
				new Rectangle2D.Double(195, 225, 13, 13), };
	}

	public class Zones {
		public final Rectangle2D POWER_CUBE_ZONE = new Rectangle2D.Double(98, 140, 42, 45);
		public final Rectangle2D EXCHANGE_ZONE = new Rectangle2D.Double(0, 102, 36, 48);
		public final Rectangle2D EXCHANGE_OPENING = new Rectangle2D.Double(0, 122.125, 0, 21);
	}
}