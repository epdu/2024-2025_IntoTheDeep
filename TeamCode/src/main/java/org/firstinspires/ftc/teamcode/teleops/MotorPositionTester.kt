package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx


@TeleOp
class MotorPositionTester : OpMode() {
    private lateinit var collectionArm: DcMotorEx
    private lateinit var scoringArm: DcMotorEx
    private lateinit var g2: PandaGamepad
    //stop being slow and flipping init please

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        g2 = PandaGamepad(gamepad2)
        collectionArm = hardwareMap.get(DcMotorEx::class.java, "collectionArm")
        scoringArm = hardwareMap.get(DcMotorEx::class.java, "scoringArm")
        collectionArm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        scoringArm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        collectionArm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        scoringArm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun loop() {
        g2.update()

        collectionArm.power = gamepad2.left_stick_y.toDouble()
        scoringArm.power = gamepad2.right_stick_y.toDouble()

        telemetry.addData("scoringArm", scoringArm.currentPosition)
        telemetry.addData("collectionArm", collectionArm.currentPosition)
    }


}