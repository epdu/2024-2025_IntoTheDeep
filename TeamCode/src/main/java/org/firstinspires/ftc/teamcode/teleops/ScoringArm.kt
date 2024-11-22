package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ScoringArm(hardwareMap: HardwareMap) {


    private val motor = hardwareMap.get(DcMotor::class.java, "scoringArm")
    private val scoringPosition = 2260 //INPUT SCORING MOTOR POSITION HERE 2170
    private val betweenBars = 1300  // 1200
    private val collectionPosition = 40 //INPUT COLLECTION MOTOR POSITION HERE 40
    var minPosition = 0
    private val maxPosition = 2500
    private var scoringArmOffset = 0
    private val motorPower = 0.75
    var targetPosition = 0.0
    var armState = ArmState.Down


    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = motorPower
    }

    enum class ArmState(val position: Int) {
        Up(2260),
        Between(1500),
        Down(40),
        Manual(-1)
    }

    inner class GoToPosition(private val state: ArmState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble() - scoringArmOffset
                motor.targetPosition = targetPosition.toInt()
                armState = state
                initialized = true
            }
            motor.currentPosition
            packet.put("Target Position", motor.targetPosition)
            packet.put("Current Position", motor.currentPosition)
            return motor.isBusy
        }
    }

    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            armState = ArmState.Manual
            targetPosition += input * maxSpeed
            //targetPosition = clamp(targetPosition, minPosition.toDouble(), maxPosition.toDouble())
            motor.targetPosition = targetPosition.toInt()
            packet.put("Target Position", motor.targetPosition)
            packet.put("Current Position", motor.currentPosition)
            return false
        }
    }

    fun resetArmPosition() {
        //Only to be used in the collection position!
        scoringArmOffset = collectionPosition - motor.currentPosition
    }


    fun goThroughBars(): Action = GoToPosition(ArmState.Between)
    fun score(): Action = GoToPosition(ArmState.Up)
    fun collect(): Action = GoToPosition(ArmState.Down)
    fun manual(input: Double): Action = Manual(input)
}