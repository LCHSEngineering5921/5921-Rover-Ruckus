/**
 * 
 */
package com.ftc.test;

/**
 * @author trinity
 *
 */
public class Turn {

	/** --------- FUNCTIONS ----------
	 * public
	 * 	boolean findGoldTF()
	 * 	boolean findGoldOCV()
	 * private
	 * 	void getCameraImagesFromVuforia()
	 */
	
	public static boolean findGoldTF() {
		System.out.println("Attempt find gold using TensorFlow");
		
		// many lines of code later...
		
		return false;
	}
	
	public static boolean findGoldOCV() {
		System.out.println("Attempt find gold using OpenCV");
		
		getCameraImagesFromVuforia();
		// many lines of code later...
		
		return false;
	}
	
	private static void getCameraImagesFromVuforia() { //List<Bitmap>
		System.out.println("Getting images from Vuforia...");
	}

}
