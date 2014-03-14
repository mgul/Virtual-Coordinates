package environment;

import java.awt.geom.Rectangle2D;
import java.io.Serializable;

import yaes.sensornetwork.Environment;

/**
 * Defines the environment for the Virtual Coordinate Sensor Network
 * @author Saad Khan
 *
 */
public class VCEnvironmentHelper implements Serializable{
	/**
	 * 
	 */
	private static final long serialVersionUID = -8484536725849716492L;

	public static Environment generateVCEnvironment() {
		Environment envr = new Environment();
		envr.setFullArea(new Rectangle2D.Double(0, 0, 1500, 1000));
		envr.setInterestArea(new Rectangle2D.Double(200, 200, 1000, 500));
		envr.setSensorDistributionArea(new Rectangle2D.Double(200, 200, 1000,
				500));
		return envr;
	}
}
