package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.clamp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ScoringArm(hardwareMap: HardwareMap) {


    private val motor = hardwareMap.get(DcMotor::class.java, "scoringArm")
    private val scoringPosition = 2170 //INPUT SCORING MOTOR POSITION HERE
    private val collectionPosition = 40 //INPUT COLLECTION MOTOR POSITION HERe
    public var minPosition = 0
    private val maxPosition = 2500
    private val motorPower = 0.6
    var targetPosition = 0.0

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = motorPower
        motor.direction = DcMotorSimple.Direction.FORWARD
    }

    inner class GoToPosition (private val position: Int) : Action {
        private var initialized = false
        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = position.toDouble()
                motor.targetPosition = position
                initialized = true
            }
            val position = motor.currentPosition
            packet.put("Target Position", motor.targetPosition)
            packet.put("Current Position", motor.currentPosition)
            return motor.isBusy
        }
    }

    inner class Manual (private val input: Double) : Action {
        val maxSpeed = 600.0

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            targetPosition += input * maxSpeed
            targetPosition = clamp(targetPosition, minPosition.toDouble(), maxPosition.toDouble())
            motor.targetPosition = targetPosition.toInt()
            packet.put("Target Position", motor.targetPosition)
            packet.put("Current Position", motor.currentPosition)
            return false
        }
    }


    fun score(): Action = GoToPosition(scoringPosition)
    fun collect(): Action = GoToPosition(collectionPosition)
    fun manual(input: Double): Action = Manual(input)
}