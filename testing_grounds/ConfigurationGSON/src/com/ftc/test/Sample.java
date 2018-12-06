/**
 * 
 */
package com.ftc.test;

/**
 * @author trinity
 *
 */
public class Sample {

	public static final String[] CRATER_PARAMETERS = {"turn", "postRT","knock"};
	public static final String[] DEPOT_PARAMETERS = {"turn", "postRT", "knock", "turn_depot", "approach_depot", "turn_claim"}; 
	
	
	/** --------- FUNCTIONS ----------
	 * public
	 * 	boolean findGoldTF()
	 * 	boolean findGoldOCV()
	 * private
	 * 	void getCameraImagesFromVuforia()
	 */
	
	public static boolean findGoldTF() {
		System.out.println("Pretending to find gold using TensorFlow...");
		
		// many lines of code later...
		
		return false;
	}
	
	public static boolean findGoldOCV() {
		System.out.println("Pretending to find gold using OpenCV...");
		
		getCameraImagesFromVuforia();
		// many lines of code later...
		
		return false;
	}
	
	private static void getCameraImagesFromVuforia() { //List<Bitmap>
		System.out.println("Pretending to get images from Vuforia...");
	}

}
