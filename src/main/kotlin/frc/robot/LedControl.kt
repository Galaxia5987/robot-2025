package frc.robot

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import frc.robot.subsystems.leds.BLINKING_TIME
import frc.robot.subsystems.leds.LED_SPACING
import frc.robot.subsystems.leds.SCROLLING_SPEED

val INTAKE: LEDPattern =
    LEDPattern.solid(Color.kWhiteSmoke).blink(BLINKING_TIME)
val CLIMB_LED: LEDPattern =
    LEDPattern.rainbow(255, 128)
        .scrollAtAbsoluteSpeed(SCROLLING_SPEED, LED_SPACING)
