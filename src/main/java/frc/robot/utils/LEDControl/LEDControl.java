package frc.robot.utils.LEDControl;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.utils.logger.Logger;

public class LEDControl {
  private static Optional<LEDControl> inst = Optional.empty();
  public static LEDControl getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new LEDControl());
    }
    return inst.get();
  }

  private Constants cnst = Constants.getInstance();

  private final ReentrantLock queueLock = new ReentrantLock();
  private List<LEDPattern> PatternQueue = new ArrayList<LEDPattern>();

  private AddressableLED LEDs = new AddressableLED(cnst.LED_STRING_PORT);
  private AddressableLEDBuffer LEDsBuff = new AddressableLEDBuffer(cnst.LED_STRING_LENGTH);

  private Thread ledThread;

  private LEDControl(){
    LEDs.setLength(LEDsBuff.getLength());
    LEDs.setData(LEDsBuff);
    LEDs.start();

    startLedThread();
  }

  public void startLedThread(){
    ledThread = new Thread(() -> {
      while (true) {
        try {
          queueLock.lock();
          update();
          queueLock.unlock();
          Thread.sleep(20);
        } catch(InterruptedException e){
          Logger.error("LEDControl : InterruptedException :  :( ");
        }
      }    
    });
    ledThread.start();
  }

  private void update(){
    wipeExpired();
    int[][] buffer = new int[cnst.LED_STRING_LENGTH][];

    for(int i = 0; i < PatternQueue.size(); i++){
      buffer = PatternQueue.get(i).update(buffer);
    }
    for(int i = 0; i < LEDsBuff.getLength(); i++){
      //if(buffer[i] != null) System.out.println(buffer[i][0] +" "+ buffer[i][1] +" "+ buffer[i][2]);
      if(buffer[i] != null) LEDsBuff.setRGB(i, buffer[i][0], buffer[i][1], buffer[i][2]);
    }
    LEDs.setData(LEDsBuff);
  }

  private void wipeExpired(){
    for(LEDPattern pattern : PatternQueue){
      if(pattern.hasExpired()) PatternQueue.remove(pattern);
    }
  }

  public static void setPattern(LEDPattern newPattern){
    LEDControl This = LEDControl.getInstance();
    This.queueLock.lock();
    if(This.PatternQueue.size() == 0){
      This.PatternQueue.add(newPattern);
      This.queueLock.unlock();
      return;
    } 
    for(int i = 0; i < This.PatternQueue.size(); i++){
      if(newPattern.priority > This.PatternQueue.get(i).priority){ 
        This.PatternQueue.add(i, newPattern);
      } else if(newPattern.priority == This.PatternQueue.get(i).priority){
        This.PatternQueue.remove(i);
        This.PatternQueue.add(i, newPattern);
      }
    }
    This.queueLock.unlock();
  }
}