package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Auto_RIGHT_Blue", group="COMPENSARE")
@Config
/**
 * se aliniaza cu al treilea pin de la stanga la dreapta al teilului din fata teilului in care este pus!
 */



// TODO: line 70-73 . first rotate down , after slider down and auto fix





public class Auto_Right_Blue extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-60.5,-35,0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence fata_prin_signal = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-5,-38,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(65,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65)
                )
                .addTemporalMarker(0.3, () -> {
                    robot.goUpSlider();
                })
                .addTemporalMarker(0.5, () -> {
                    robot.goUpAuto();
                })
                .build();

        TrajectorySequence pune_preload = drive.trajectorySequenceBuilder(fata_prin_signal.end())
                .lineToLinearHeading(new Pose2d(-7.25,-30.5,Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(35,35,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(35))//43
                .build();
//-7.5,-31

        TrajectorySequence aliniere_con1 = drive.trajectorySequenceBuilder(pune_preload.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-13,-53.5,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(68,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(68))//90
                .addTemporalMarker(0.6, () -> {
                    robot.goDownAuto_New();
                })
                .addTemporalMarker(0.7,()->
                {
                    robot.goSliderCon1dinStack();
                })
                .build();

        TrajectorySequence intake_con1 = drive.trajectorySequenceBuilder(aliniere_con1.end())
                .lineToConstantHeading(new Vector2d(-13,-65),
                        SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();

        TrajectorySequence outtake_con1 = drive.trajectorySequenceBuilder(intake_con1.end())
                .splineToLinearHeading(new Pose2d(-7.25,-33.5,Math.toRadians(45)),Math.toRadians(75),
                        SampleMecanumDrive.getVelocityConstraint(65,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .addTemporalMarker(0.01, () -> {
                    robot.goUpSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con2 = drive.trajectorySequenceBuilder(outtake_con1.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-13,-53.5,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(68,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(68))//90
                .addTemporalMarker(0.6, () -> {
                    robot.goDownAuto_New();
                })
                .addTemporalMarker(0.7,()->
                {
                    robot.goSliderCon2dinStack();
                })
                .build();
        TrajectorySequence intake_con2 = drive.trajectorySequenceBuilder(aliniere_con2.end())
                .lineToConstantHeading(new Vector2d(-13,-65.5),
                        SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        TrajectorySequence outtake_con2 = drive.trajectorySequenceBuilder(intake_con2.end())
                .splineToLinearHeading(new Pose2d(-7.5,-34,Math.toRadians(45)),Math.toRadians(75),
                        SampleMecanumDrive.getVelocityConstraint(65,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .addTemporalMarker(0.01, () -> {
                    robot.goUpSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con3 = drive.trajectorySequenceBuilder(outtake_con2.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45,Math.toRadians(80),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(-13,-53.5,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(68,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(68))//90
                .addTemporalMarker(0.6, () -> {
                    robot.goDownAuto_New();
                })
                .addTemporalMarker(0.7,()->
                {
                    robot.goSliderCon3dinStack();
                })
                .build();
        TrajectorySequence intake_con3 = drive.trajectorySequenceBuilder(aliniere_con3.end())
                .lineToConstantHeading(new Vector2d(-13,-66.5),
                        SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        TrajectorySequence outtake_con3 = drive.trajectorySequenceBuilder(intake_con3.end())
                .splineToLinearHeading(new Pose2d(-7.5,-34,Math.toRadians(45)),Math.toRadians(75),
                        SampleMecanumDrive.getVelocityConstraint(65,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .addTemporalMarker(0.01, () -> {
                    robot.goUpSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con4 = drive.trajectorySequenceBuilder(outtake_con3.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(-13,-53.5,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(68,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(68))//90
                .addTemporalMarker(0.6, () -> {
                    robot.goDownAuto_New();
                })
                .addTemporalMarker(0.7,()->
                {
                    robot.goSliderCon4dinStack();
                })
                .build();
        TrajectorySequence intake_con4 = drive.trajectorySequenceBuilder(aliniere_con4.end())
                .lineToConstantHeading(new Vector2d(-13,-67),
                        SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        TrajectorySequence outtake_con4 = drive.trajectorySequenceBuilder(intake_con4.end())
                .splineToLinearHeading(new Pose2d(-7.75,-34.5,Math.toRadians(45)),Math.toRadians(75),
                        SampleMecanumDrive.getVelocityConstraint(65,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .addTemporalMarker(0.01, () -> {
                    robot.goUpSlider();

                })
                .addTemporalMarker(0.2, () -> {

                    robot.goUpAuto();
                })
                .build();
        TrajectorySequence aliniere_con5 = drive.trajectorySequenceBuilder(outtake_con4.end())
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .lineToLinearHeading(new Pose2d(-13,-53.5,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(68,Math.toRadians(225),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(68))//90
                .addTemporalMarker(0.6, () -> {
                    robot.goDownAuto_Further();
                })
                .addTemporalMarker(0.7,()->
                {
                    robot.goSliderCon5dinStack();
                })
                .build();
        TrajectorySequence intake_con5 = drive.trajectorySequenceBuilder(aliniere_con5.end())
                .lineToConstantHeading(new Vector2d(-13,-67), //esti prost frate
                        SampleMecanumDrive.getVelocityConstraint(60,Math.toRadians(90),13.62),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        TrajectorySequence outtake_con5 = drive.trajectorySequenceBuilder(intake_con5.end())
                .splineToLinearHeading(new Pose2d(-8,-33.5 ,Math.toRadians(45)),Math.toRadians(75),
                        SampleMecanumDrive.getVelocityConstraint(50,225,13.62),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0.1, () -> {
                    robot.goUpSlider();

                })
                .addTemporalMarker(0.4, () -> {

                    robot.goUpAuto();
                })
                .build();

        robot.inchideGheara();
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

        if (isStopRequested()) return;

        ElapsedTime timeElapsed = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        drive.followTrajectorySequence(fata_prin_signal);
        drive.followTrajectorySequence(pune_preload);
        int col = detectionPipeline.getColumnDetected("yellow");

        if(col != 5 && col != 6 && col != 7) // la 0 sta pe loc si da patherorr
        {
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(pune_preload.end())
                    .strafeLeft(1)
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(45))
                    .build();

            Pose2d pose = drive.getPoseEstimate();
            if(col < 5 && col != 0)
                reposition = drive.trajectorySequenceBuilder(pune_preload.end())
                        .strafeLeft((5 - col) * 0.45)
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(220), 13.62),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
            else if(col > 7 )
                reposition = drive.trajectorySequenceBuilder(pune_preload.end())
                        .strafeRight((col - 7) * 0.45)
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(220), 13.62),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();
            else if(col == 0)
                reposition = drive.trajectorySequenceBuilder(pune_preload.end())
                        .strafeRight(1.5)
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(220), 13.62),
                                SampleMecanumDrive.getAccelerationConstraint(30))
                        .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        sleep(100);
        robot.deschideGheara();

        sleep(150);
        drive.followTrajectorySequence(aliniere_con1);
        drive.followTrajectorySequence(intake_con1);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con1);

        if(Math.abs(detectionPipeline.getMoveAmount("blue")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con1.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("blue"))
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(45))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }
        sleep(50);
        robot.deschideGheara();
        sleep(100);

        drive.followTrajectorySequence(aliniere_con2);
        drive.followTrajectorySequence(intake_con2);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con2);

        if(Math.abs(detectionPipeline.getMoveAmount("blue")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con2.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("blue"))
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(45))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }

        sleep(50);
        robot.deschideGheara();
        sleep(100);

        drive.followTrajectorySequence(aliniere_con3);
        drive.followTrajectorySequence(intake_con3);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con3);

        if(Math.abs(detectionPipeline.getMoveAmount("blue")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con3.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("blue"))
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(45))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }
        sleep(50);
        robot.deschideGheara();
        sleep(100);
        drive.followTrajectorySequence(aliniere_con4);
        drive.followTrajectorySequence(intake_con4);
        robot.inchideGheara();
        sleep(150);
        drive.followTrajectorySequence(outtake_con4);


        if(Math.abs(detectionPipeline.getMoveAmount("blue")) > 0.5) {
            Pose2d pose = drive.getPoseEstimate();
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con4.end())
                    .strafeLeft(detectionPipeline.getMoveAmount("blue"))
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(45))
                    .build();


            drive.followTrajectorySequence(reposition);
            drive.setPoseEstimate(new Pose2d(pose.getX(),pose.getY(),drive.getPoseEstimate().getHeading()));
            drive.update();
        }
        sleep(50);
        robot.deschideGheara();
        sleep(100);

        if(timeElapsed.time() < 24000) {
            drive.followTrajectorySequence(aliniere_con5);
            drive.followTrajectorySequence(intake_con5);
            robot.inchideGheara();
            sleep(150);
            drive.followTrajectorySequence(outtake_con5);


            if (Math.abs(detectionPipeline.getMoveAmount("blue")) > 0.5) {
                Pose2d pose = drive.getPoseEstimate();
                TrajectorySequence reposition = drive.trajectorySequenceBuilder(outtake_con5.end())
                        .strafeLeft(detectionPipeline.getMoveAmount("blue"))
                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 13.62),
                                SampleMecanumDrive.getAccelerationConstraint(45))
                        .build();


                drive.followTrajectorySequence(reposition);
                drive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), drive.getPoseEstimate().getHeading()));
                drive.update();
            }
            sleep(50);
            robot.deschideGheara();
            sleep(100);

        }
        TrajectorySequence parkGreen = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-21,-40 ,Math.toRadians(0)))
                .addTemporalMarker(0.9, () -> {
                    robot.goDownAutoFinal();
                    robot.goDownSlider();
                })
                .build();

        TrajectorySequence alin = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-15,-40 ,Math.toRadians(0)))
                .addTemporalMarker(1.2, () -> {
                    robot.goDownAutoFinal();
                    robot.goDownSlider();
                })
                .build();
        drive.followTrajectorySequence(alin);
        TrajectorySequence parkRed = drive.trajectorySequenceBuilder(alin.end())
                .strafeLeft(25)
                .build();

        TrajectorySequence parkYellow = drive.trajectorySequenceBuilder(alin.end())
                .strafeRight(25)
                .build();

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(alin.end())
                .back(18)
                .build();

        if(tagOfInterest == null){
            drive.followTrajectorySequence(parkGreen);
        }else{
            switch(tagOfInterest.id){
                case 1:
                    drive.followTrajectorySequence(parkRed);

                    break;
                case 2:
                    drive.followTrajectorySequence(parkGreen);
                    break;
                case 3:
                    drive.followTrajectorySequence(parkYellow);

                    break;
            }
        }

        sleep(1000);
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