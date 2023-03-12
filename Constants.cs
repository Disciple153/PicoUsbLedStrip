// Version.cs
public static class Constants
{
    //build
    public static string ROM_ID = "DD";
    public static int LED_STRIP_LENGTH = 98;
    public static int DATA_LENGTH = LED_STRIP_LENGTH * 3;
    public static int SERIAL_PAGE_SIZE = 512; // If there are problems transmitting bytes, lower this.
    public static int PC_TO_PICO_TIMEOUT_MS = 100;

    public enum DisplayMode : byte
    {
        Solid,
        Pulse
    };
}