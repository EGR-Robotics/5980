package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import com.ctre.phoenix.led.LarsonAnimation;

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

    
    public void EGR(){
        LarsonAnimation strobe = new LarsonAnimation(0, 255,0 );

        m_candle.animate(strobe);
    }
}
