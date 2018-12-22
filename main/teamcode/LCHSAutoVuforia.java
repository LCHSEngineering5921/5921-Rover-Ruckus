package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class LCHSAutoVuforia {

    private static final String TAG = "LCHSAutoVuforia";

    // Vuforia and Tensorflow variables from sample ConceptTensorFlowObjectDetection
    private static final String VUFORIA_KEY = "\"AblqOtv/////AAAAGdNKXPkjgkKGpGIKO9aGT7wPTFlXhki/3S+80P7yBeTE37kLJOvzyL57aik4RAD6LXyHPDRMdo4PJnxwaCzQj1+8CxEv1Z/dTr1IcjnZmsGC1d6zui4rxx1docFTjjpebkimp6A95reel2toZqogkexy/txx0KPHyF09E6jzZSV9AjFlexbSLe1/SfqoMa4TRLIBgWK3/Sj/TYY/b0WAwJOJYnFoqZAtvEyasqnBG9wulo/A6UKtKmRxmiWQ6QY/x2NAi4giPONyaWRAvFvSWUQjRkTFzCc9npy10RMX4qHZK0ow79YfMnyZqnRwHAR4Fp4HzPgdnmlf0yK/lQ3jxCLKT9cDdhNcayDeOrx6Y197";
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targetsRoverRuckus;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private boolean vuforiaActivated = false;

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private LCHSValues.MineralVisionSystem mineralVision;
    private LinearOpMode linearOpMode;
    private LCHSHardwareMap robot;
    private boolean debugVerbose = false;

    public LCHSAutoVuforia(LCHSValues.MineralVisionSystem pMineralVision, LinearOpMode pLinear, LCHSHardwareMap pHardwareMap, boolean pDebugVerbose) {
        mineralVision = pMineralVision;
        linearOpMode = pLinear; // get access to LinearOpMode public fields and methods
        robot = pHardwareMap;
        debugVerbose = pDebugVerbose;

         /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = BACK;
        //**parameters.fillCameraMonitorViewParent = false; //** TRY THIS - even default constructor shows the monitor

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        //** LATER - only load the trackables we need:
        // "Red-Footprint" and "Blue-Rover" - since we don't know our alliance (it's not encoded in the opmode).
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 230;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 158;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 230;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        if (mineralVision == LCHSValues.MineralVisionSystem.OPENCV) {
            // Initialize for frame capture.
            vuforia.setFrameQueueCapacity(1);
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        }
    }

    // Required during Tensorflow initialization.
    public VuforiaLocalizer getVuforiaForTensorflow() {
        return vuforia;
    }

    // Turn on for Vumark recognition
   public void activateVuforia() {
       if (!vuforiaActivated) {
           targetsRoverRuckus.activate();
           vuforiaActivated = true;
       }
   }

   // Turn off when done with Vumark recognition.
   public void deactivateVuforia() {
       if (vuforiaActivated) {
           targetsRoverRuckus.deactivate();
           vuforiaActivated = false;
       }
   }

    // Checks if the robot can see a Vumark.
    public boolean readVumark() {
        LCHSValues.Alliance alliance = LCHSValues.Alliance.UNKNOWN;
        OpenGLMatrix lastLocation = null;
        boolean retVal = false;

        if (!vuforiaActivated)
            throw new AutonomousRobotException(TAG, "readVumark(): Vuforia is not activated");

        for (int i = 0; i < 5; i++) { // attempt vuforia 5 times
            for (VuforiaTrackable trackable : allTrackables) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {

                    if ((trackable.getName().equals("Blue-Rover")))
                        alliance = LCHSValues.Alliance.BLUE;
                    else if ((trackable.getName().equals("Red-Footprint")))
                        alliance = LCHSValues.Alliance.RED;

                    if (alliance != LCHSValues.Alliance.UNKNOWN) {
                        lastLocation = robotLocationTransform;
                        RobotLog.dd(TAG, "Found Vuforia target " + trackable.getName() + " at loop count " + i);
                        break;
                    }
                }
            }

            if (alliance != LCHSValues.Alliance.UNKNOWN)
                break;
            android.os.SystemClock.sleep(100);
        }


        if (lastLocation == null) {
            RobotLog.dd(TAG, "Did not find the Vuforia target");
            retVal = false;
        } else {
            retVal = true;

            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            linearOpMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            RobotLog.dd(TAG, "Vuforia target location: X " + translation.get(0) / mmPerInch
                    + ", Y " + translation.get(1) / mmPerInch + ", Z " + translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            linearOpMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            RobotLog.dd(TAG, "Vuforia heading " + rotation.thirdAngle);
            //** MOVE this RobotLog.dd(TAG, "IMU heading " + motion.getIMUHeading());
        }

        return retVal;
    }

       public List<Bitmap> getVuforiaImages() throws InterruptedException {

        List<Bitmap> imageCollection = new ArrayList<>();
        List<Bitmap> cameraImages = new ArrayList<>();

        RobotLog.dd(TAG, "In getMostRecentVuforiaImage");

        //** TOO MANY iterations?
        for (int imX = 0; imX < 10; imX++) {
            cameraImages = getVuforiaFrameQueue();
            if (!cameraImages.isEmpty())
                imageCollection.addAll(cameraImages);
            android.os.SystemClock.sleep(100);
        }

       return imageCollection;
    }

    // Gets camera images from Vuforia's frame queue.
    private List<Bitmap> getVuforiaFrameQueue() throws InterruptedException {

        List<Bitmap> bitmapReturn = new ArrayList<>();

        // from 2016-2017?? Thread.sleep(500); // Allow a little time to settle

        /* LCHS 2019 this callback method comes from the sample ConceptVuforiaNavigationWebcam
         * but would require thread synchronization (Condition with timeout) to work here.
        // Access camera images via the Vuforia frame object.
        vuforiaFrameBitmap = null;
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                vuforiaFrameBitmap = vuforia.convertFrameToBitmap(frame);
            }
        }));
        */

        //** Possibly increase the queue depth ...
        RobotLog.dd(TAG, "Vuforia frame queue size " + vuforia.getFrameQueue().size());
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        if (frame == null) {
            RobotLog.dd(TAG, "No frame available from Vuforia");
            return bitmapReturn; // return empty
        }

        long numImages = frame.getNumImages();
        RobotLog.dd(TAG, "Got " + numImages + " images from Vuforia");

        // Return all RGB images.
        Bitmap oneBitmap;
        for (int i = 0; i < numImages; i++) {
            //imageFormat =  frame.getImage(i).getFormat();
            RobotLog.dd("LCHS", "Got an image of type " + frame.getImage(i).getFormat());
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                RobotLog.dd("LCHS", "Got an RGB image at image index " + i);
                Image rgb = frame.getImage(i);
                oneBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                oneBitmap.copyPixelsFromBuffer(rgb.getPixels());
                bitmapReturn.add(oneBitmap);
            }
        }

        frame.close(); // release the frame according to R. Atkinson
        return bitmapReturn;
    }


}
