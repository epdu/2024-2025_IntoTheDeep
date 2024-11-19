package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive

@TeleOp
abstract class CompetitionTeleop : OpMode() {
    private lateinit var drive: PinpointDrive
    private lateinit var g1: PandaGamepad
    private lateinit var g2: PandaGamepad
    private lateinit var claw: Claw
    private lateinit var sampleClaw: SampleClaw
    private lateinit var collectionArm: CollectionArm
    private lateinit var scoringArm: ScoringArm
    private var headingOffset: Double = 0.0
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    private var runningActions: MutableList<Action> = ArrayList()
    private var lastTime: Double = 0.0

    override fun init() {
        drive = PinpointDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        g1 = PandaGamepad(gamepad1)
        g2 = PandaGamepad(gamepad2)
        claw = Claw(hardwareMap)
        collectionArm = CollectionArm(hardwareMap)
        scoringArm = ScoringArm(hardwareMap)
    }

    fun nearest_compass (curHeading: Double, direction: String): Double {
        var head_out = 0.0
        if (curHeading >= 0 && curHeading < 90){ head_out = 90.0 }
        else if (curHeading >= 90 && curHeading < 180){ head_out = 180.0}
        else if (curHeading >= -180 && curHeading < -90){ head_out = -90.0}
        else if (curHeading >= -90 && curHeading < 0){ head_out = 0.0 }

        if (direction ==  "left") {return head_out - 90.0}
        else if (direction == "right") {return head_out}
        else {return 0.0}
    }

    override fun start() {
        lastTime = runtime
    }

    override fun loop() {
        // measure time between loops
        val newTime = runtime
        val deltaTime: Double = newTime - lastTime
        lastTime = newTime

        //update gamepad values
        g1.update()
        g2.update()

        //run and update actions
        val packet = PandaTelemetryPacket(telemetry)

        val newActions: MutableList<Action> = ArrayList()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        dash.sendTelemetryPacket(packet)

        //update drive Pose
        drive.updatePoseEstimate()

        /* driver 1 */
        val rawHeading = drive.getPinpoint().heading
        val heading: Rotation2d = Rotation2d.fromDouble(rawHeading - headingOffset)

        val input = Vector2d(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble()
        )
        //if (g1.rightBumper.isInactive() and g1.leftBumper.isInactive()) //don't set drive if bumpers
        //    drive.setDrivePowers(
        //        PoseVelocity2d(
        //            heading.inverse().times(input),
        //            ((gamepad1.left_trigger - gamepad1.right_trigger) * 1 / 2).toDouble()
        //        )
        //    )

        if (g1.b.justPressed()) headingOffset = rawHeading
        /*
        if (g1.leftBumper.justPressed()) {
            //Turn left to the nearest 90 degree increment
            val beginPose = Pose2d(0.0, 0.0, rawHeading-headingOffset)
            var headTo = nearest_compass(rawHeading-headingOffset, "left")
            headTo += -headingOffset
            runBlocking(
                drive.actionBuilder(beginPose)
                    .turnTo(headTo)
                    .build()
            )
        }
        if (g1.rightBumper.justPressed()){
            //Turn right to the nearest 90 degree increment
            val beginPose = Pose2d(0.0, 0.0, rawHeading-headingOffset)
            var headTo = nearest_compass(rawHeading-headingOffset, "right")
            headTo += -headingOffset
            runBlocking(
                drive.actionBuilder(beginPose)
                    .turnTo(-90.0)
                    .build()
            )
        }
        */
        /* driver 2 */
        // Sample Arm***********************************************************
        if (g2.dpadDown.justPressed()) runningActions.add(claw.approach())
        if (g2.dpadLeft.justPressed()) runningActions.add(claw.close())
        if (g2.dpadRight.justPressed()) runningActions.add(claw.open())
        if (g2.dpadUp.justPressed()) runningActions.add(claw.close())
        if (g2.leftBumper.justActive()) runningActions.add(claw.inBox())

        if (g2.a.justPressed()) runningActions.add(sampleClaw.open())
        if (g2.y.justPressed()) runningActions.add(sampleClaw.close())
        if (g2.x.justPressed()) runningActions.add(sampleClaw.open())

        /*

        if (g2.a.justPressed()) scoringArm.score()
        if (g2.b.justPressed()) scoringArm.collect()*/

        if (g2.leftStickY.isActive()) runningActions.add(scoringArm.manual(g2.leftStickY.component * deltaTime))



    }

    protected abstract val allianceColor: AllianceColor
}