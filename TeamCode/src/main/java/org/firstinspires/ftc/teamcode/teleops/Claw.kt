package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Claw (hardwareMap: HardwareMap) {

    private val leftClaw: Servo = hardwareMap.get(Servo::class.java, "leftClaw")
    private val rightClaw: Servo = hardwareMap.get(Servo::class.java, "rightClaw")

    inner class SetPosition(val dt: Double, private val leftPosition: Double, private val rightPosition: Double) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (beginTs < 0) {
                beginTs = now()
                leftClaw.position = leftPosition
                rightClaw.position = rightPosition
            }
            val t = now() - beginTs
            p.put("test", true)

            return t < dt
        }
    }

    fun close(): Action = SetPosition(1.0, 0.46, 0.7)
    fun open(): Action = SetPosition(1.0, 0.495, 0.645)
    fun inBox(): Action = SetPosition(1.0, 0.83, 0.0)
    fun approach(): Action = SetPosition(1.0, 0.6, 0.6)
}