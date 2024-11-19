package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class SampleClaw (hardwareMap: HardwareMap) {

    private val pulley: Servo = hardwareMap.get(Servo::class.java, "sampleServo")

    inner class SetPosition(val dt: Double, private val pulleyPosition: Double) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            if (beginTs < 0) {
                beginTs = now()
                pulley.position = pulleyPosition
            }
            val t = now() - beginTs
            p.put("test", true)

            return t < dt
        }
    }

    fun close(): Action = SetPosition(1.0, 0.46)
    fun open(): Action = SetPosition(1.0, 0.3)
}