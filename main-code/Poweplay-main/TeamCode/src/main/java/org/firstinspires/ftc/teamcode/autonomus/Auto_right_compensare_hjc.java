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

package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auto_RIGHT_compensare_red", group="Linear Opmode")
@Config
@Disabled

public class Auto_right_compensare_hjc extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    DetectionPipeline detectionPipeline;
    private SampleMecanumDrive drive;
    private RobotUtils robot;


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

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.

        detectionPipeline = new DetectionPipeline();
        robot = new RobotUtils(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //TODO: START POSE E DIFERIT FATA DE STARTPOSE DIN Auto_Near_BLUE
        Pose2d startPose = new Pose2d(35,-64,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence place_preload = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(11.5,-61,Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(9.5,-27.5,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(20.5,-63,Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(9,-23,Math.toRadians(180)),Math.toRadians(90))
                .addTemporalMarker(0.3, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.5, () -> {
                    robot.goUpAuto();
                })
                .build();

        TrajectorySequence alignment_for_preload_offset = drive.trajectorySequenceBuilder(place_preload.end())
                .lineToLinearHeading(new Pose2d(17,-13.75,Math.toRadians(180)))

                .build();

        TrajectorySequence get_cone_1_from_stack = drive.trajectorySequenceBuilder(alignment_for_preload_offset.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(65,-13.75,Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    robot.goDownAuto_Further();

                })
                .addTemporalMarker(0.3,()->
                {
                    robot.goSliderCon1dinStack();
                })
                .build();

        TrajectorySequence place_cone_1 = drive.trajectorySequenceBuilder(get_cone_1_from_stack.end())
                .setReversed(false)

                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(5,-13.75,Math.toRadians(270)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .addTemporalMarker(0.05, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.goUpAuto();
                })
                .build();
        //TODO: DIFFERS FROM get_cone_1_from_stack !!!;
        TrajectorySequence get_cone_2_from_stack = drive.trajectorySequenceBuilder(place_cone_1.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .lineToLinearHeading(new Pose2d(67,-13.75,Math.toRadians(180)))

                .addTemporalMarker(0.8, () -> {
                    robot.goDownAuto_Further();

                })
                .addTemporalMarker(1,()->
                {
                    robot.goSliderCon2dinStack();
                })
                .build();
        TrajectorySequence place_cone_2 = drive.trajectorySequenceBuilder(get_cone_2_from_stack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(5,-13.75,Math.toRadians(270)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .addTemporalMarker(0.05, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.goUpAuto();
                })
                .build();

        TrajectorySequence get_cone_3_from_stack = drive.trajectorySequenceBuilder(place_cone_2.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .lineToLinearHeading(new Pose2d(70,-13.75,Math.toRadians(180)))

                .addTemporalMarker(0.8, () -> {
                    robot.goDownAuto_Further();

                })
                .addTemporalMarker(1,()->
                {
                    robot.goSliderCon3dinStack();
                })
                .build();


        TrajectorySequence place_cone_3 = drive.trajectorySequenceBuilder(get_cone_3_from_stack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(10,-13.75,Math.toRadians(270)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .addTemporalMarker(0.05, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.goUpAuto();
                })
                .build();

        TrajectorySequence get_cone_4_from_stack = drive.trajectorySequenceBuilder(place_cone_3.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .lineToLinearHeading(new Pose2d(73,-13.75,Math.toRadians(180)))

                .addTemporalMarker(0.8, () -> {
                    robot.goDownAuto_Further();

                })
                .addTemporalMarker(1,()->
                {
                    robot.goSliderCon4dinStack();
                })
                .build();

        TrajectorySequence place_cone_4 = drive.trajectorySequenceBuilder(get_cone_4_from_stack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(11.5,-13.75,Math.toRadians(270)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .addTemporalMarker(0.05, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.goUpAuto();
                })
                .build();

        TrajectorySequence get_cone_5_from_stack = drive.trajectorySequenceBuilder(place_cone_4.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)),
                         SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .lineToLinearHeading(new Pose2d(71,-13.75,Math.toRadians(180)))

                .addTemporalMarker(0.8, () -> {
                    robot.goDownAuto_Further();

                })
                .addTemporalMarker(1,()->
                {
                    robot.goSliderCon5dinStack();
                })
                .build();
        TrajectorySequence place_cone_5 = drive.trajectorySequenceBuilder(get_cone_4_from_stack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(20,-13.75,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(12.5,-13.75,Math.toRadians(270)),
                       SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(180),13.61),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                .addTemporalMarker(0.05, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.goUpAuto();
                })
                .build();


        robot.inchideGheara();

        robot.hoverSlider();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
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

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        camera.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(10);

        drive.followTrajectorySequence(place_preload);
        sleep(50);
        robot.deschideGheara();
        sleep(150);
        drive.followTrajectorySequence(alignment_for_preload_offset);
        drive.followTrajectorySequence(get_cone_1_from_stack);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(place_cone_1);
        telemetry.addData("move",detectionPipeline.getMoveAmount("red"));
        telemetry.update();

        if(Math.abs(detectionPipeline.getMoveAmount("red")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(place_cone_1.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("red"))
                     .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        sleep(100);
        robot.deschideGheara();


        drive.followTrajectorySequence(get_cone_2_from_stack);
        robot.inchideGheara();
        sleep(200);
        drive.followTrajectorySequence(place_cone_2);


        if(Math.abs(detectionPipeline.getMoveAmount("red")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(place_cone_2.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("red"))
                     .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        telemetry.addData("move",detectionPipeline.getMoveAmount("red"));
        telemetry.update();
        sleep(100);
        robot.deschideGheara();


        drive.followTrajectorySequence(get_cone_3_from_stack);
        robot.inchideGheara();
        sleep(200);
        drive.followTrajectorySequence(place_cone_3);


        if(Math.abs(detectionPipeline.getMoveAmount("red")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(place_cone_3.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("red"))
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }
        sleep(100);
        robot.deschideGheara();

        drive.followTrajectorySequence(get_cone_4_from_stack);
        robot.inchideGheara();
        sleep(200);
        drive.followTrajectorySequence(place_cone_4);


        if(Math.abs(detectionPipeline.getMoveAmount("red")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(place_cone_4.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("red"))
                     .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }
        sleep(100);
        robot.deschideGheara();

        TrajectorySequence alignPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(15,-13.75,Math.toRadians(180)))
                .build();

        TrajectorySequence parkMid = drive.trajectorySequenceBuilder(alignPark.end())
                .lineToLinearHeading(new Pose2d(35,-13.75,Math.toRadians(0)))
                .addTemporalMarker(0.3, () -> {
                    robot.goDownAutoFinal();

                })
                .addTemporalMarker(0.5,()->
                {
                    robot.goDownSlider();
                })
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(alignPark.end())
                .lineToLinearHeading(new Pose2d(16,13,Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                    robot.goDownAutoFinal();

                })
                .addTemporalMarker(0.3,()->
                {
                    robot.goDownSlider();
                })

                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(alignPark.end())
                .lineToLinearHeading(new Pose2d(50,-13.75,Math.toRadians(0)))
                .addTemporalMarker(0.3, () -> {
                    robot.goDownAutoFinal();

                })
                .addTemporalMarker(0.5,()->
                {
                    robot.goDownSlider();
                })
                .build();

        drive.followTrajectorySequence(alignPark);

        if(tagOfInterest == null){
            drive.followTrajectorySequence(parkMid);
        }else{
            switch(tagOfInterest.id){
                case 1:
                    drive.followTrajectorySequence(parkLeft);
                    break;
                case 2:
                    drive.followTrajectorySequence(parkMid);
                    break;
                case 3:
                    drive.followTrajectorySequence(parkRight);
                    break;
            }
        }

        sleep(200);

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}