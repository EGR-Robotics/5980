package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CANDLE;

public class Candle {
    private final CANdle m_candle = new CANdle(CANDLE.CAN_ID);

    public Candle() {
        CANdleConfiguration candleConfig = new CANdleConfiguration();

        candleConfig.disableWhenLOS = false;
        candleConfig.stripType = LEDStripType.RGB;
        // candleConfig.brightnessScalar = 0.1; // dim the LEDs to half brightness
        // configALL.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfig, 100);

    }

    
    public void Blink(){
        StrobeAnimation strobe = new StrobeAnimation(0, 255,0, 0, 0.5, CANDLE.kLED_TOTAL);

        m_candle.animate(strobe);
    }

}
