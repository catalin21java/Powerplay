package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);





                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

        RoadRunnerBotEntity NewAutoBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, Math.toRadians(420), Math.toRadians(420), 13.61)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10.5,64.75,Math.toRadians(-90)))//0
                        .splineToLinearHeading(new Pose2d(-15,30,Math.toRadians(-135)),Math.toRadians(-110),
                                SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                                SampleMecanumDrive.getAccelerationConstraint(55))

                                .build()
                );

        RoadRunnerBotEntity NewAutoBlue2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, Math.toRadians(420), Math.toRadians(420), 13.61)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60,35,Math.toRadians(90)))//0
                        .lineToLinearHeading(new Pose2d(-45,35,Math.toRadians(-45)),
                                SampleMecanumDrive.getVelocityConstraint(45,45,13.62),
                                SampleMecanumDrive.getAccelerationConstraint(45))//43
                        .lineToLinearHeading(new Pose2d(-22 ,35,Math.toRadians(-90)),
                                SampleMecanumDrive.getVelocityConstraint(65,225,13.62),
                                SampleMecanumDrive.getAccelerationConstraint(65)
                        )
                        .lineToLinearHeading(new Pose2d(-10.5,36,Math.toRadians(110)))
                        .lineToConstantHeading(new Vector2d(-10.5,63.25),
                                SampleMecanumDrive.getVelocityConstraint(55,Math.toRadians(80),13.62),
                                SampleMecanumDrive.getAccelerationConstraint(55))
                        .splineToLinearHeading(new Pose2d(-16,31.5,Math.toRadians(-135)),Math.toRadians(-125),
                                SampleMecanumDrive.getVelocityConstraint(55,225,13.62),
                                SampleMecanumDrive.getAccelerationConstraint(55))
                        .build()


                );

        RoadRunnerBotEntity testpreload = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, Math.toRadians(80), Math.toRadians(80), 13.61)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60.5,-35,0))//0

                        .splineToLinearHeading( new Pose2d(-22 ,-34,Math.toRadians(90)),Math.toRadians(11),
                                SampleMecanumDrive.getVelocityConstraint(55,80,13.62),
                                SampleMecanumDrive.getAccelerationConstraint(55))

                        .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)

                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(testpreload)
                //.addEntity(NewAutoBlue_CONCEPT)
                //.addEntity(NewAutoBlue_CONCEPT2)

                .start();

    }

}

