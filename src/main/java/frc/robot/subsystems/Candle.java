package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

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

        off();
    }

    // public void coral(){
    // }

    public void EGR(){
        LarsonAnimation larson = new LarsonAnimation(0, 0,255, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7);
        LarsonAnimation larson2 = new LarsonAnimation(0, 0,255, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7,7);
        LarsonAnimation larson3 = new LarsonAnimation(0, 0, 255, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7, 14);
        LarsonAnimation larson4 = new LarsonAnimation(0, 0, 255, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7, 21);

        LarsonAnimation larson6 = new LarsonAnimation(255, 255,0, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7,49);
        LarsonAnimation larson7 = new LarsonAnimation(255, 255, 0, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7, 56);
        LarsonAnimation larson8 = new LarsonAnimation(255, 255, 0, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7, 63);
        LarsonAnimation larson5 = new LarsonAnimation(255, 255,0, 0, 0.75, CANDLE.LED_TOTAL, BounceMode.Center, 7,70);
        
        m_candle.animate(larson, 0);
        m_candle.animate(larson2, 1);
        m_candle.animate(larson3, 2);
        m_candle.animate(larson4, 3);
        m_candle.animate(larson5, 4);
        m_candle.animate(larson6, 5);
        m_candle.animate(larson7, 6);
        m_candle.animate(larson8, 7);
    }
    
    public void GetCoral(){
        StrobeAnimation strobe1 = new StrobeAnimation(255, 0, 0);

        m_candle.animate(strobe1, 0);
    }
    
    public void stopAnimations(){
        m_candle.clearAnimation(0);
        m_candle.clearAnimation(1);
        m_candle.clearAnimation(2);
        m_candle.clearAnimation(3);
        m_candle.clearAnimation(4);
        m_candle.clearAnimation(5);
        m_candle.clearAnimation(6);
        m_candle.clearAnimation(7);
    }
    
    public void off(){
        stopAnimations();
        
        m_candle.setLEDs(255, 0, 0);
    }
    
    public void green(){
        stopAnimations();

        m_candle.setLEDs(0, 255, 0);
    }
    
}
