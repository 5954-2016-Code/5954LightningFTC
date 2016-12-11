package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class VisionSystem {

    public static final String TAG = "Vuforia Images";
    public OpenGLMatrix robotLocationTransform = null;
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackables vortexImages;
    public OpenGLMatrix blueTargetLocationOnField;
    public OpenGLMatrix redTargetLocationOnField;
    public List<VuforiaTrackable> allTrackables;
    public VuforiaTrackable targetWheels;
    public VuforiaTrackable targetTools;
    public VuforiaTrackable targetLegos;
    public VuforiaTrackable targetGears;
    public VectorF lastTranslation;
    public Orientation lastOrientation;

    /**
     * We use units of mm here because that's the recommended units of measurement for the
     * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
     *      <ImageTarget name="stones" size="247 173"/>
     * You don't *have to* use mm here, but the units here and the units used in the XML
     * target configuration files *must* correspond for the math to work out correctly.
     */
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your Motors
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    public VisionSystem(){
    }

    public void init() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        //Key requested from vuforia by Jim T. to be used for FTC Team 5954 Lightning Boltz
        parameters.vuforiaLicenseKey = "AeRdQvz/////AAAAGcMAAb4w60VfokjyjAgA1AmOBx4sQhg3Su9jPDF8zg75+eD2qLald9iAXLAPqj6H7VNpaIK4s7J+o3GYV5K7KpbWdMqE1IbixAkzxd3pmhCOMnn+6tW7yxs4DeXR0NeZyAv4lSr36rJSeF5LGrwbxl4FDNZKI4lnV0tLR8UggOUG0E+4Rcuu8Mnwgg9L/q8z7n6VwsmVSMVzpggXXmWhq3qz43bHD5vmVZQAVDUqUnIPIACGh9VfLlEnQGvxYVS9CXc4bfWqfl3VUFeXhtcH3QODRBsRyE48BbuAjiEUliCUUWGtTYzrB+Cv2wg6xxrA6li86yyHMsjmAzMSiKhJX1hKjxr4WUk5u72ylsD1wAIJ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        vortexImages = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        targetWheels = vortexImages.get(0);
        targetWheels.setName("Wheels");  // Wheels

        targetTools  = vortexImages.get(1);
        targetTools.setName("Tools");  // Tools

        targetLegos  = vortexImages.get(2);
        targetLegos.setName("Legos");  // Legos

        targetGears  = vortexImages.get(3);
        targetGears.setName("Gears");  // Gears

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(vortexImages);

        /* To place the Gears Target on the Red Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
                * - Then we rotate it  90 around the field's Z access to face it away from the audience.
                * - Finally, we translate it back along the X axis towards the red audience wall.
        */
        redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL to the Left of Red Driver Station Wall. Our translation here
                is a negative translation in X.*/
                //change from 1/2 to .41666666667--gears and .75--tools
                .translation(-mmFTCFieldWidth/(float).41666666667, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        targetGears.setLocation(redTargetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

       /*
        * To place the Wheels Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        blueTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                To the Right of the Blue Driver Station wall
                Our translation here is a positive translation in Y.*/
                //change from 1/2 to .41666666667--wheels and .75--legos
                .translation(0, mmFTCFieldWidth/(float).41666666667, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        targetWheels.setLocation(blueTargetLocationOnField);
        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the Motors. Here, we
         * put the phone on the right hand side of the Motors with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * Motors's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */

        //TODO: This is for landscape mode. How do we set it to portrait mode?
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0 /*-90 for right hand side of Motors*/, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)targetWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)targetGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }

    public void startTracking()
    {
        vortexImages.activate();
    }

    public boolean isTargetLocated(ButtonPushSystem.BeaconColor teamColor)
    {
        if (teamColor == ButtonPushSystem.BeaconColor.Red) {
            return ((VuforiaTrackableDefaultListener)targetGears.getListener()).isVisible();
        }
        else //if (tmColor == teamColor.BLUE)
        {
            return ((VuforiaTrackableDefaultListener)targetWheels.getListener()).isVisible();
        }
    }

    public OpenGLMatrix updateLastLocation(ButtonPushSystem.BeaconColor teamColor)
    {
        if (teamColor == ButtonPushSystem.BeaconColor.Red)
        {
            robotLocationTransform = ((VuforiaTrackableDefaultListener)targetGears.getListener()).getUpdatedRobotLocation();
        }
        else
        {
            robotLocationTransform = ((VuforiaTrackableDefaultListener)targetWheels.getListener()).getUpdatedRobotLocation();
        }

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
        }

        lastTranslation = lastLocation.getTranslation();
        lastOrientation = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return lastLocation;
    }

    public float get_xAngle()
    {
        return lastOrientation.firstAngle;
    }

    public float get_yAngle()
    {
        return lastOrientation.secondAngle;
    }

    public float get_zAngle()
    {
        return lastOrientation.thirdAngle;
    }

    public float get_xMillimeters()
    {
        return lastTranslation.getData()[0];
    }

    public float get_yMillimeters()
    {
        return lastTranslation.getData()[1];
    }

    public float get_zMillimeters()
    {
        return lastTranslation.getData()[2];
    }

    public String lastLocationToString()
    {
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "Motors=%s", format(lastLocation));
            return format(lastLocation);
        }
        return "Unknown";
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    public String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

//Orientation toString code
//    @Override public String toString()
//    {
//        if (this.angleUnit == AngleUnit.DEGREES)
//            return String.format("{%s %s %.0f %.0f %.0f}", this.axesReference.toString(), this.axesOrder.toString(), this.firstAngle, this.secondAngle, this.thirdAngle);
//        else
//            return String.format("{%s %s %.3f %.3f %.3f}", this.axesReference.toString(), this.axesOrder.toString(), this.firstAngle, this.secondAngle, this.thirdAngle);
//    }

//    Vectorf toString code
//    @Override public String toString()
//    {
//        StringBuilder result = new StringBuilder();
//        result.append("{");
//        for (int i = 0; i < this.length(); i++)
//        {
//            if (i > 0) result.append(" ");
//            result.append(String.format("%.2f", this.data[i]));
//        }
//        result.append("}");
//        return result.toString();
//    }

}
