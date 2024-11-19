package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap


class ScoringArm(hardwareMap: HardwareMap) {


    private val motor = hardwareMap.get(DcMotor::class.java, "scoringArm")
    private val scoringPosition = 2170 //INPUT SCORING MOTOR POSITION HERE
    private val collectionPosition = 40 //INPUT COLLECTION MOTOR POSITION HERE
    private val minPosition = 0
    private val maxPosition = 2800
    private val motorPower = 0.6

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    inner class GoToPosition (private val position: Int) : Action {
        private var initialized = false
        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                motor.power = motorPower
                motor.targetPosition = position
                initialized = true
            }
            val position = motor.currentPosition
            packet.put("armPosition", position)
            return motor.isBusy
        }
    }


    fun score(): Action = GoToPosition(scoringPosition)
    fun collect(): Action = GoToPosition(collectionPosition)
    fun manual(input: Double): Action {
        val maxSpeed = 5.0
        var targetPosition = (motor.currentPosition + input * maxSpeed)
        targetPosition = clamp(targetPosition, minPosition.toDouble(), maxPosition.toDouble())
        return GoToPosition(targetPosition.toInt())
    }
}