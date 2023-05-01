package frc.robot.utils.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.constants.Constants;

public class Logger {
  private static Optional<Logger> inst = Optional.empty();

  private ArrayList<LogLine> cache = new ArrayList<LogLine>();

  private final ReentrantLock cacheLock = new ReentrantLock();

  private final DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_hh-mm");

  private Thread thread;

  private boolean paused = false;
  private boolean stopped = false;

  private File file;
  private String filePath;
  private boolean fileCreationFailed = false;

  private Logger() {
    findFilePath();
    makeFile();
    launchThread();
  }

  public static Logger getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new Logger());
    }
    return inst.get();
  }

  public static void header(String content) {
    LogLine line = new LogLine(content, LogLevel.HEADER);
    addLine(line);
  }

  public static void info(String content) {
    LogLine line = new LogLine(content, LogLevel.INFO);
    addLine(line);
  }

  public static void warn(String content) {
    LogLine line = new LogLine(content, LogLevel.WARNING);
    addLine(line);
  }

  public static void error(String content) {
    LogLine line = new LogLine(content, LogLevel.ERROR);
    addLine(line);
  }

  private static void addLine(LogLine line) {
    Logger logger = getInstance();
    logger.cacheLock.lock();
    logger.cache.add(line);
    logger.cacheLock.unlock();
  }

  private void findFilePath() {
    if (paused) {
      return;
    }

    try {
      for (String tmpPath : Constants.file.PATH_USB) {
        if (new File(tmpPath).exists()) {
          filePath = tmpPath;
          System.out.println("Logger : File Path Found : " + filePath);
          return;
        }
      }
      fileCreationFailed = true;
      System.out.println("Logger : File Path Not Found : Logger printing to console");
    } catch (SecurityException e) {
      fileCreationFailed = true;
      System.out.println("Logger : File Path Security Exception : Logger printing to console");
    }
  }

  private void makeFile() {
    if (fileCreationFailed || paused) {
      return;
    }

    Date date = Calendar.getInstance().getTime();
    file = new File(filePath + dateFormat.format(date) + ".txt");

    try {
      for (int i = 0; !file.createNewFile(); i++) {
        if (i > Constants.logger.REPEAT_LIMIT_LOGGER_CREATION) {
          fileCreationFailed = true;
          System.out.println("Logger : File Creation Failed : Timed Out : Logger printing to console");
          return;
        }
        file = new File(filePath + dateFormat.format(date) + "_(" + i + ").txt");
      }
    } catch (IOException e) {
      fileCreationFailed = true;
      System.out.println("Logger : File Creation Failed : IOException : " + e + " : Logger printing to console");
    } catch (SecurityException e) {
      fileCreationFailed = true;
      System.out.println("Logger : File Creation Failed : Security Exception : " + e + " : Logger printing to consol");
    }
  }

  public void save() {
    if (cache.isEmpty() || paused) {
      return;
    }

    cacheLock.lock();

    if (fileCreationFailed) {
      while (!cache.isEmpty()) {
        System.out.println(cache.remove(0).toString());
      }
      cacheLock.unlock();
      return;
    }

    try {
      FileWriter writer = new FileWriter(file, true);
      while (!cache.isEmpty()) {
        writer.write(cache.remove(0).toString());
        writer.write(System.lineSeparator());
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("Logger : File Save Failed : IOExeption : " + e);
    } catch (SecurityException e) {
      System.out.println("Logger : File Save Failed : Security Exeption : " + e);
    } finally {
      if (!cache.isEmpty()) {
        System.out.println("Logger : Save Error : Dumping Cache to Console");
      }
      while (!cache.isEmpty()) {
        System.out.println(cache.remove(0).toString());
      }
      cacheLock.unlock();
    }
    cacheLock.unlock();
  }

  /** reversibly stops operation of the logger */
  public void pause() {
    if (!stopped) {
      paused = true;
    }
  }

  /** restarts the logger when paused */
  public void unpause() {
    if (!stopped) {
      paused = false;
    }
  }

  /** irreversably kills the logger */
  public void stop() {
    if (!stopped) {
      pause();
      stopped = true;
    }
  }

  private void launchThread() {
    if (stopped) {
      return;
    }
    thread = new Thread(() -> {
      try {
        while (!stopped) {
          save();
          Thread.sleep(1000 / Constants.logger.SAVE_RATE);
        }
      } catch (InterruptedException e) {
        System.out.println("Logger : Save Thread Interrupted : " + e);
      }
    });
    thread.start();
  }
}
