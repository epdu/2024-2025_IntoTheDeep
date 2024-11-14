package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

@TeleOp
class MotorPositionTester(hardwareMap: HardwareMap) : OpMode() {
    private val collectionArmMotor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "collectionArmMotor")
    private lateinit var g2: PandaGamepad
    override fun init() {
        g2 = PandaGamepad(gamepad2)

    }

    override fun loop() {
        g2.update()
        val packet = TelemetryPacket()

        collectionArmMotor.power = gamepad2.left_stick_y.toDouble()

        packet.put("collectionArmMotor", collectionArmMotor.currentPosition)
    }


}