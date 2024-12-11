package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;

@Autonomous
public class SpecimenSamplePark extends OpMode {
    SpecimenClaw scoringClaw;
    ScoringArm scoringArm;
    Pose2d beginPose;
    PinpointDrive drive;
    //BTWWWW heading is in radians jsyk\
    //MATH.PI IS 90 DEGREES
    //North is looking at baskets

    public void init() {
        scoringClaw = new SpecimenClaw(hardwareMap);
        scoringArm = new ScoringArm(hardwareMap);
        beginPose = new Pose2d(-17.5, 66, -Math.PI / 2);
        drive = new PinpointDrive(hardwareMap, beginPose);
    }

    public void start() {
        Actions.runBlocking(scoringClaw.close()); //collect
        Actions.runBlocking(scoringArm.score()); //score pos
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-4, 42)) //go to front of bar
                        .strafeTo(new Vector2d(-4, 35)) // clip it in
                        .build()
        );
        Actions.runBlocking(scoringClaw.open());
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.setTangent(Math.PI/2)
                        .strafeTo(new Vector2d(-6, 54)) // backs up
                        .build()
        );


        Actions.runBlocking(scoringClaw.approach());
        Actions.runBlocking(scoringArm.collect());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(Math.PI)
                        .strafeTo(new Vector2d(-38, 61)) // go to collect 2nd specimen
                        .build()
        );
        Actions.runBlocking(scoringClaw.close());
        Actions.runBlocking(scoringArm.score());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.setTangent(Math.PI/2)
                        .turnTo(-Math.PI/2)
                        .strafeTo(new Vector2d(-2, 54))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(3, 42)) //go to front of bar
                        .strafeTo(new Vector2d(3, 36)) // clip it in
                        .waitSeconds(0.5)

                        .build()
        );
        Actions.runBlocking(scoringClaw.open());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .strafeTo(new Vector2d(-6, 57)) // backs up
                        .build()
        );
        //it stops here idk why
        Actions.runBlocking(scoringClaw.close());
        Actions.runBlocking(scoringArm.collect());
       // goes to first sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .strafeTo(new Vector2d(-45,72 )) // go to sample
                        .build()
        );


        /*Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .strafeTo(new Vector2d(-60.5, 64)) // go to park
                        .build()
        );
        Actions.runBlocking(scoringClaw.close());
        Actions.runBlocking(scoringArm.collect());
        */

    }

    public void loop() {}
}
