package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class Leds {
    // RGB colors for the LED strip
    private final int[] coneRGB = new int[]{255, 255, 0};
    private final int[] cubeRGB = new int[]{255, 0, 255};

    private final double MAX_BRIGHTNESS = 0.5;
    CANdle candle;
    CANdleConfiguration configs;

    private final int LED_COUNT = 66;

    public Leds() {
        candle = new CANdle(0);

        configs = new CANdleConfiguration();
        configs.brightnessScalar = MAX_BRIGHTNESS;
        configs.stripType = CANdle.LEDStripType.GRB;
        configs.disableWhenLOS = true;
        configs.statusLedOffWhenActive = true;
        configs.vBatOutputMode = CANdle.VBatOutputMode.On;
        candle.configAllSettings(configs);
    }

    // changes whether leds are enabled
    public void setLedEnabled(boolean enabled) {
        if (enabled) {
            candle.configBrightnessScalar(MAX_BRIGHTNESS);
        } else {
            candle.configBrightnessScalar(0);
        }
    }

    // sets the color of led
    public void setLedColor(boolean isCube) {
        if (isCube) {
            candle.setLEDs(cubeRGB[0], cubeRGB[1], cubeRGB[2]);
        } else {
            candle.setLEDs(coneRGB[0], coneRGB[1], coneRGB[2]);
        }
    }
}
