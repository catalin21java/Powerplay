package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Auto_Left_Mid_Red", group="AUTINOMOUSGOOD")
@Config
/**
 * se aliniaza cu al treilea pin de la stanga la dreapta al teilului din fata teilului in care este pus!
 */



// TODO: line 70-73 . first rotate down , after slider down and auto fix





public class Auto_Left_Mid_Red extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;

    DetectionPipeline detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;


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
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(aprilTagDetectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        robot = new RobotUtils(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DetectionPipeline pipeline = new DetectionPipeline();
        int PosSlider=-540;

        Pose2d startPose = new Pose2d(-60.5,35,0);
        drive.setPoseEstimate(startPose);


        TrajectorySequence pune_preload = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46,35,Math.toRadians(-45)),
                        SampleMecanumDrive.getVelocityConstraint(45,45,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(45))//43
                .lineToLinearHeading(new Pose2d(-21.5 ,34,Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(65,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65)
                )
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();
                })
                .addTemporalMarker(0.31, () -> {
                    robot.goMediumUpAuto();
                })
                .build();
//-7.5,-31

        TrajectorySequence aliniere_con1 = drive.trajectorySequenceBuilder(pune_preload.end())
                .lineToLinearHeading(new Pose2d(-10,38,Math.toRadians(-90)))
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAuto_New();
                })
                .addTemporalMarker(0.9,()->
                {
                    robot.goSliderCon1dinStack();
                })
                .build();

        TrajectorySequence intake_con1 = drive.trajectorySequenceBuilder(aliniere_con1.end())
                .lineToConstantHeading(new Vector2d(-10,63),
                        SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();

        TrajectorySequence outtake_con1 = drive.trajectorySequenceBuilder(intake_con1.end())
                .splineToLinearHeading(new Pose2d(-15,31,Math.toRadians(-135)),Math.toRadians(-110),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goMediumUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con2 = drive.trajectorySequenceBuilder(outtake_con1.end())
                .lineToLinearHeading(new Pose2d(-10,42,Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))//90
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAuto_New();
                })
                .build();
        TrajectorySequence intake_con2 = drive.trajectorySequenceBuilder(aliniere_con2.end())
                .lineToConstantHeading(new Vector2d(-10,63),
                        SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))

                .addTemporalMarker(0.3,()->
                {
                    robot.goSliderCon2dinStack();
                })
                .build();

        TrajectorySequence outtake_con2 = drive.trajectorySequenceBuilder(intake_con2.end())
                .splineToLinearHeading(new Pose2d(-15,31,Math.toRadians(-135)),Math.toRadians(-110),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();

                })
                .addTemporalMarker(0.23, () -> {

                    robot.goMediumUpAuto();
                })
                .build();

        TrajectorySequence aliniere_con3 = drive.trajectorySequenceBuilder(outtake_con2.end())
                .lineToLinearHeading(new Pose2d(-10,42,Math.toRadians(-90)),
                SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                SampleMecanumDrive.getAccelerationConstraint(55))//90
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAuto_New();
                })

                .build();
        TrajectorySequence intake_con3 = drive.trajectorySequenceBuilder(aliniere_con3.end())
                .lineToConstantHeading(new Vector2d(-10,63.25),
                        SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.3,()->
                {
                    robot.goSliderCon3dinStack();
                })
                .build();
        TrajectorySequence outtake_con3 = drive.trajectorySequenceBuilder(intake_con3.end())
                .splineToLinearHeading(new Pose2d(-14.75,30.5,Math.toRadians(-135)),Math.toRadians(-110),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goMediumUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con4 = drive.trajectorySequenceBuilder(outtake_con3.end())
                .lineToLinearHeading(new Pose2d(-10,42,Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))//90
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAuto_New();
                })

                .build();
        TrajectorySequence intake_con4 = drive.trajectorySequenceBuilder(aliniere_con4.end())
                .lineToConstantHeading(new Vector2d(-10,63.5),
                        SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.3,()->
                {
                    robot.goSliderCon4dinStack();
                })
                .build();
        TrajectorySequence outtake_con4 = drive.trajectorySequenceBuilder(intake_con4.end())
                .splineToLinearHeading(new Pose2d(-14.75,30.5,Math.toRadians(-135)),Math.toRadians(-110),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();

                })
                .addTemporalMarker(0.25, () -> {

                    robot.goMediumUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con5 = drive.trajectorySequenceBuilder(outtake_con4.end())
                .lineToLinearHeading(new Pose2d(-10,42,Math.toRadians(-90)),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))//90
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAuto_Cone5();
                })

                .build();
        TrajectorySequence intake_con5 = drive.trajectorySequenceBuilder(aliniere_con5.end())
                .lineToConstantHeading(new Vector2d(-10,64),
                        SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.3,()->
                {
                    robot.goSliderCon5dinStack();
                })
                .build();

        TrajectorySequence outtake_con5 = drive.trajectorySequenceBuilder(intake_con5.end())
                .splineToLinearHeading(new Pose2d(-14.75,30,Math.toRadians(-135)),Math.toRadians(-110),
                        SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .addTemporalMarker(0.01, () -> {
                    robot.goMediumSlider();

                })
                .addTemporalMarker(0.3, () -> {

                    robot.goMediumUpAuto();
                })
                .build();

        robot.inchideGheara();
        sleep(500);
        robot.hoverSlider();

        sleep(2000);
        detectionPipeline.setGridSize(10);

        String zoneColor = "red";
        telemetry.addData("zoneColor",zoneColor);
        telemetry.update();

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

        webcam.setPipeline(detectionPipeline);

//        if (isStopRequested()) return;


        drive.followTrajectorySequence(pune_preload);
        sleep(50);
        robot.deschideGheara();
        sleep(50);
        drive.followTrajectorySequence(aliniere_con1);
        drive.followTrajectorySequence(intake_con1);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con1);
        sleep(50);
        robot.deschideGheara();
        sleep(75);
        drive.followTrajectorySequence(aliniere_con2);
        drive.followTrajectorySequence(intake_con2);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con2);
        sleep(50);
        robot.deschideGheara();
        sleep(75);
        drive.followTrajectorySequence(aliniere_con3);
        drive.followTrajectorySequence(intake_con3);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con3);

        double moveAmount = detectionPipeline.getMoveAmountMedium("red");
        if(Math.abs(moveAmount) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con3.end())
                    .strafeLeft(moveAmount)
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        telemetry.addData("move",detectionPipeline.getMoveAmount("red"));

        sleep(50);
        robot.deschideGheara();
        sleep(75);

        drive.followTrajectorySequence(aliniere_con4);
        drive.followTrajectorySequence(intake_con4);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con4);

        moveAmount = detectionPipeline.getMoveAmountMedium("red");
        if(Math.abs(moveAmount) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con4.end())
                    .strafeLeft(moveAmount)
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        telemetry.addData("move",detectionPipeline.getMoveAmount("red"));

        sleep(50);
        robot.deschideGheara();
        sleep(75);

        drive.followTrajectorySequence(aliniere_con5);
        drive.followTrajectorySequence(intake_con5);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con5);

        moveAmount = detectionPipeline.getMoveAmountMedium("red");

        if(Math.abs(moveAmount) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con5.end())
                    .strafeLeft(moveAmount)
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(55))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        sleep(50);
        robot.deschideGheara();
        sleep(75);
        TrajectorySequence parkGreen = drive.trajectorySequenceBuilder(outtake_con5.end())
                .lineToLinearHeading(new Pose2d(-9,40 ,Math.toRadians(-179)))
                .addTemporalMarker(0.5, () -> {
                    robot.goDownAutoFinal();
                })
                .addTemporalMarker(0.9, () -> {
                    robot.goDownSlider();
                })
                .build();

        drive.followTrajectorySequence(parkGreen);
//        TrajectorySequence alin = drive.trajectorySequenceBuilder(parkGreen.end())
//                .lineToLinearHeading(new Pose2d(-9,37 ,Math.toRadians(90)))
//                .addTemporalMarker(0.4, () -> {
//                    robot.goDownSlider();
//                })
//                .build();

        TrajectorySequence parkRed = drive.trajectorySequenceBuilder(parkGreen.end())
                .lineToLinearHeading(new Pose2d(-9,63 ,Math.toRadians(-179)))

                .build();

        TrajectorySequence parkYellow = drive.trajectorySequenceBuilder(parkGreen.end())
                .lineToLinearHeading(new Pose2d(-9,10 ,Math.toRadians(-179)))

                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(parkGreen.end())
                .lineToLinearHeading(new Pose2d(-20,37 ,Math.toRadians(-179)))

                .build();

        if(tagOfInterest == null){
            //drive.followTrajectorySequence(alin);
            drive.followTrajectorySequence(moveForward);
        }else{
            switch(tagOfInterest.id){
                case 1:
                    // drive.followTrajectorySequence(alin);
                    drive.followTrajectorySequence(parkRed);

                    break;
                case 2:
                    //drive.followTrajectorySequence(alin);
                    drive.followTrajectorySequence(moveForward);
                    break;
                case 3:
                    //drive.followTrajectorySequence(alin);
                    drive.followTrajectorySequence(parkYellow);

                    break;
            }
        }

        sleep(750);
        telemetry.addData("Object", robot.hasDetectedObject());
        telemetry.addData("Servo",robot.gheara.getPosition());
        telemetry.addData("slider1",robot.slider1.getCurrentPosition());
        telemetry.addData("slider2",robot.slider2.getCurrentPosition());
        telemetry.addData("rotire",robot.rotire.getCurrentPosition());


        telemetry.update();

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }



}