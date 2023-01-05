package frc.robot.utils;

import java.time.Duration;
import java.time.Instant;

/**
 * Class to help with timing things asynchronously.
 */
public class AsyncTimer {
  public int duration;

  private Instant start;
  private Instant timed;
  private boolean paused;

  private void update() {
    if (!paused) {
      timed = Instant.now();
    }
  }

  /**
   * Creates a new AsyncTimer and starts it.
   * 
   * @param duration The duration of the timer in milliseconds.
   */
  public AsyncTimer(int duration) {
    this.start = Instant.now();
    this.timed = this.start;
    this.paused = false;
    this.duration = duration;
  }

  /**
   * Creates a new AsyncTimer.
   * 
   * @param duration The duration of the timer in milliseconds.
   * @param paused   Whether the timer should be paused after instantiation or
   *                 not.
   */
  public AsyncTimer(int duration, boolean paused) {
    this.start = Instant.now();
    this.timed = this.start;
    this.paused = paused;
    this.duration = duration;
  }

  /**
   * Pauses the timer.
   */
  public void pause() {
    paused = true;
  }

  /**
   * Unpauses the timer.
   */
  public void unpause() {
    paused = false;
  }

  /**
   * Gets if the timer is paused.
   * 
   * @return True if the timer is paused.
   */
  public boolean isPaused() {
    return paused;
  }

  /**
   * Toggles the pause.
   */
  public void togglePaused() {
    if (isPaused()) {
      unpause();
    } else {
      pause();
    }
  }

  /**
   * Returns the number of milliseconds until the timer has finished.
   * 
   * @return The number of ms until the timer has finished.
   */
  public long timeUntilFinished() {
    update();
    return Duration.between(start, timed).toMillis();
  }

  /**
   * Checks if the timer has finished.
   * 
   * @return true if the timer's duration has been elapsed and false if not.
   */
  public boolean isFinished() {
    return timeUntilFinished() >= duration;
  }
}
