package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    // RGB colors for the LED strip
    private final int[] coneRGB = new int[]{255, 200, 0};
    private final int[] cubeRGB = new int[]{255, 0, 255};

    boolean isAnimating = true;

    private final double MAX_BRIGHTNESS = 1.0;
    private CANdle candle;

    private String state;

    CANdleConfiguration configs;

    // initialises the LED stuffs
    ShuffleboardTab tab = Shuffleboard.getTab("General");

    GenericEntry isEnabledEntry = tab.add("LEDs Enabled", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    GenericEntry isCubeEntry = tab.add("Cube Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();


    //NetworkTableEntry rainbow;
    GenericEntry rainbowEntry = tab.add("Rainbow Mode", false)
            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private final int LED_COUNT = 66;

    public Leds() {
        candle = new CANdle(0);

        configs = new CANdleConfiguration();
        configs.brightnessScalar = MAX_BRIGHTNESS;
        configs.stripType = CANdle.LEDStripType.GRB;
        configs.disableWhenLOS = false;
        configs.statusLedOffWhenActive = true;
        configs.vBatOutputMode = CANdle.VBatOutputMode.On;
        candle.configAllSettings(configs);

        //rainbow = NetworkTableInstance.getDefault().getTable("led").getEntry("rainbow");

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
    public void setLedColor(boolean isCube, boolean isRainbow) {
        if (isRainbow) {
            if (state != "rainbow") {
                state = "rainbow";
                candle.animate(new RainbowAnimation(1, 0.7, LED_COUNT), 1);
            }
        } else if(!isCube) {
            if (state != "cone") {
                state = "cone";
                candle.clearAnimation(1);
                candle.setLEDs(coneRGB[0], coneRGB[1], coneRGB[2]);
                isAnimating = false;
            }
        } else {
            if (state != "cube") {
                state = "cube";
                isAnimating = false;
                candle.clearAnimation(1);
                candle.setLEDs(cubeRGB[0], cubeRGB[1], cubeRGB[2]);
            }
        }
    }



    @Override
    public void periodic() {
        this.setLedColor(isCubeEntry.getBoolean(false), rainbowEntry.getBoolean(false));
        // this.setLedEnabled(true);
    }
}
