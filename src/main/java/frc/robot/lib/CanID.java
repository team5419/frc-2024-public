package frc.robot.lib;

public class CanID {
    private int id;
    private String canbus;

    public CanID (int id, String canbus) {
      this.id = id;
      this.canbus = canbus;
    }

    public int getID() {
      return id;
    }

    public String getCanbus() {
      return canbus;
    }
}