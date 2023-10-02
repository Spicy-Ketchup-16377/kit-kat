/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package Tessts;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import AprilTagsss.AprilTagDetectionPipeline;

@Disabled
@Autonomous
public class NEWLEFT_Auto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST1 = 0; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 1;
    int ID_TAG_OF_INTEREST3 = 2;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();



            if(currentDetections.size() != 0)
            {

                double tagFound = 0;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST1)
                    {
                        /** tag id 0 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 1;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST2)
                    {
                        /** tag id 0 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 2;
                        break;
                    }
                    else if(tag.id == ID_TAG_OF_INTEREST3)
                    {
                        /** tag id 0 is detected**/
                        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                        tagOfInterest = tag;
                        tagFound = 3;
                        break;
                    }
                }

                if(tagFound == 1)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if(tagFound == 2)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else if(tagFound == 3)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        Pose2d startPose = new Pose2d(-36, -62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory START = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36,-12)) //set up
                .build();
        //turn
        Trajectory HIGH1 = drive.trajectoryBuilder(START.end().plus(new Pose2d(0,0, Math.toRadians(145))))
                .lineTo(new Vector2d(-27,-4))
                .build();
        //wait (arm stuff happens) place preload
        Trajectory BACKHIGH1 = drive.trajectoryBuilder(HIGH1.end())
                //.lineToLinearHeading(new Pose2d(36,12,Math.toRadians(0))) //pose to begin cycles
                .lineTo(new Vector2d(-36,-12))
                .build();
        //turn
        Trajectory CONES = drive.trajectoryBuilder(BACKHIGH1.end().plus(new Pose2d(0,0, Math.toRadians(128))))
                .lineTo(new Vector2d(-64,-9))
                .build();
        //wait (arm stuff) grab cone1
        Trajectory BACKCONES = drive.trajectoryBuilder(CONES.end())
                .lineTo(new Vector2d(-57,-9))
                .build();

        Trajectory HIGH_HEADING = drive.trajectoryBuilder(BACKCONES.end())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(45)))
                .build();
        Trajectory HIGH2 = drive.trajectoryBuilder(HIGH_HEADING.end())
                .lineTo(new Vector2d(-26,-2))
                .build();
        Trajectory BACKHIGH2 = drive.trajectoryBuilder(HIGH2.end())
                .lineTo(new Vector2d(-36,-12)) //set up
                .build();


        //parking
//        Trajectory REDLEFTZONE3 = drive.trajectoryBuilder(traj5.end())
//                .lineToLinearHeading(new Pose2d(-1,-11, Math.toRadians(270)))
//                .build();
//        Trajectory REDLEFTZONE2 = drive.trajectoryBuilder(traj5.end())
//                .lineToLinearHeading(new Pose2d(-35,-14,Math.toRadians(270)))
//                .build();
//        Trajectory REDLEFTZONE1 = drive.trajectoryBuilder(traj5.end())
//                .lineToLinearHeading(new Pose2d(-60,-11,Math.toRadians(270)))
//                .build();

        /*
         * Insert your autonomous code here, probably using the tag pose to decide your configuration.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */

            drive.followTrajectory(START);
            drive.turn(Math.toRadians(135));
            drive.followTrajectory(HIGH1);
//            //arm stuff (preload)
            drive.followTrajectory(BACKHIGH1);
            drive.turn(Math.toRadians(135));
            drive.followTrajectory(CONES);
////            //grab cone1
            drive.followTrajectory(BACKCONES);
//            drive.followTrajectory(HIGH_HEADING);
//            drive.followTrajectory(HIGH1);
//            drive.followTrajectory(BACKHIGH1);
//            drive.turn(Math.toRadians(-45));
        }
        else //TODO:auto code
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            drive.followTrajectory(START);
            drive.turn(Math.toRadians(135));
            drive.followTrajectory(HIGH1);
//            //arm stuff (preload)
            drive.followTrajectory(BACKHIGH1);
            drive.turn(Math.toRadians(135));
            drive.followTrajectory(CONES);
////            //grab cone1
            drive.followTrajectory(BACKCONES);
//            drive.followTrajectory(HIGH_HEADING);
//            drive.followTrajectory(HIGH2);
//            drive.followTrajectory(BACKHIGH2);
//            drive.turn(Math.toRadians(-45));



            // e.g.
//            if(tagOfInterest.id == 0)
//            {
//                // do something
//                drive.followTrajectory(REDLEFTZONE1);
//
//            }
//            else if(tagOfInterest.id == 1)
//            {
//                // do something else
//                drive.followTrajectory(REDLEFTZONE2);
//            }
//            else if(tagOfInterest.id == 2)
//            {
//                // do something else
//                drive.followTrajectory(REDLEFTZONE3);
//            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}