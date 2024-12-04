package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ScoringArm(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ArmState(val position: Int) {
        Score(2400),
        ThroughBars(1500),
        Collect(40),
        Manual(-1)
    }

    var armState = ArmState.Collect

    private val scoringArm = hardwareMap.get(DcMotor::class.java, "scoringArm")

    private val power = 0.75

    private var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        scoringArm.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        scoringArm.direction = DcMotorSimple.Direction.REVERSE
        scoringArm.targetPosition = 0
        scoringArm.mode = DcMotor.RunMode.RUN_TO_POSITION
        scoringArm.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ArmState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble() - scoringArmOffset
                scoringArm.targetPosition = targetPosition.toInt()
                armState = state
                initialized = true
            }
            scoringArm.currentPosition
            packet.put("Target Position", scoringArm.targetPosition)
            packet.put("Current Position", scoringArm.currentPosition)
            return scoringArm.isBusy
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */
    inner class Manual(private val input: Double) : Action {
        val maxSpeed = 600.0 //tics/second

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            armState = ArmState.Manual
            targetPosition += input * maxSpeed
            scoringArm.targetPosition = targetPosition.toInt()
            packet.put("Target Position", scoringArm.targetPosition)
            packet.put("Current Position", scoringArm.currentPosition)
            return false
        }
    }

    /**
     * Only use in the collect position; used to reset the positions of the arm
     */
    fun resetArmPosition() {
        scoringArmOffset = ArmState.Collect.position - scoringArm.currentPosition
    }

    fun throughBars(): Action = SetState(ArmState.ThroughBars)
    fun score(): Action = SetState(ArmState.Score)
    fun collect(): Action = SetState(ArmState.Collect)
    fun manual(input: Double): Action = Manual(input)
}