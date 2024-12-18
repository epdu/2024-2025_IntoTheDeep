package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim

object MeepMeepAutoSpecimenSample {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(800)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                .followTrajectorySequence { drive: DriveShim ->
                    drive.trajectorySequenceBuilder(
                        Pose2d(
                            -17.5,
                            66.0,
                            -Math.PI/2
                        )
                    )
                        .setTangent(0.0)
                        .splineToConstantHeading(Vector2d(-4.0,42.0), -Math.PI/2)
                        .strafeTo(Vector2d(-4.0, 35.0))
                        .waitSeconds(1.0)
                        .strafeTo(Vector2d(-6.0, 54.0))
                        .waitSeconds(1.0)
                        .setTangent(Math.PI/2)
                        .splineToLinearHeading(Pose2d(Vector2d(-38.0,61.0), Math.PI), Math.PI)
                        .waitSeconds(1.0)
                        .setTangent(0.0)
                        .splineToLinearHeading(Pose2d(Vector2d(-4.0,42.0), -Math.PI/2), -Math.PI/2)
                        .strafeTo(Vector2d(-4.0,35.0))
                        .waitSeconds(1.0)
                        .strafeTo(Vector2d(-6.0, 57.0))
                        .waitSeconds(1.0)
                        .setTangent(Math.PI / 2)
                        .splineToConstantHeading(Vector2d(-24.0, 48.0), Math.PI)
                        .splineToConstantHeading( Vector2d(-42.0, 14.0), Math.PI)
                        .waitSeconds(1.0)
                        .strafeTo(Vector2d(-42.0,64.0)) //go to 1st speicemen
                        .strafeTo(Vector2d(-42.0,7.0)) //go to 1st speicemen

                        .strafeTo(Vector2d(-46.0,7.0)) //strafe over
                        .strafeTo(Vector2d(-46.0,64.0))//go to 2nd speicemen
                        .strafeTo(Vector2d(-46.0,7.0))//go to 2nd speicemen
                        .strafeTo(Vector2d(-50.0,7.0)) //strafe over
                        .strafeTo(Vector2d(-50.0,64.0)) //go to 3rd speicemen


                        .build()
                }

        meepMeep.setBackground(Background.GRID_BLUE)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}