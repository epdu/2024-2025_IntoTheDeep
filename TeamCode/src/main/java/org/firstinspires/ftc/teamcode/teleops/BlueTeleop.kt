package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "CompetitionTeleop")
class BlueTeleop : CompetitionTeleop() {
    override val allianceColor: AllianceColor = AllianceColor.BLUE
}