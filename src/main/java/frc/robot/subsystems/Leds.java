package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    // RGB colors for the LED strip
    private final int[] coneRGB = new int[]{255, 200, 0};
    private final int[] cubeRGB = new int[]{255, 0, 255};

    private final double MAX_BRIGHTNESS = 1.0;
    CANdle candle;
    CANdleConfiguration configs;

    // initialises the LED stuffs
    ShuffleboardTab tab = Shuffleboard.getTab("General");

    GenericEntry isEnabledEntry = tab.add("LEDs Enabled", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    GenericEntry isCubeEntry = tab.add("Cube Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

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

    @Override
    public void periodic() {
        this.setLedColor(isCubeEntry.getBoolean(false));
        this.setLedEnabled(true);
    }
}
