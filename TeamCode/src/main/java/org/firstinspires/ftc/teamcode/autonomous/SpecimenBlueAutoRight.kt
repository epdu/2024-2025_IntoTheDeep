package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive
import org.firstinspires.ftc.teamcode.subsystems.ScoringArm
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw

@Autonomous
class SpecimenBlueAutoRight : OpMode() {
    var scoringClaw: SpecimenClaw? = null
    var scoringArm: ScoringArm? = null
    var beginPose: Pose2d? = null
    var drive: PinpointDrive? = null

    override fun init() {
        scoringClaw = SpecimenClaw(hardwareMap)
        scoringArm = ScoringArm(hardwareMap)
        beginPose = Pose2d(-17.5, 66.0, -Math.PI / 2)
        drive = PinpointDrive(hardwareMap, beginPose)
    }

    override fun start() {
        //Actions.runBlocking(scoringClaw.close());
        //Actions.runBlocking(scoringArm.score());
        runBlocking(
            drive!!.actionBuilder(beginPose)
                .strafeTo(Vector2d(-6.0, 42.0))
                .strafeTo(Vector2d(-6.0, 38.0))
                .build()
        )
        //Actions.runBlocking(scoringClaw.open());
        drive!!.updatePoseEstimate()
        runBlocking(
            drive!!.actionBuilder(drive!!.pose)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(Vector2d(-24.0, 48.0), Math.PI)
                .splineToConstantHeading(Vector2d(-42.0, 14.0), Math.PI)
                .build()
        )
        //Actions.runBlocking(scoringClaw.close());
    }

    override fun loop() {}
}
