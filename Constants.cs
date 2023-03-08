// Version.cs
public static class Constants
{
    //build
    public static string ROM_ID = "DeskDisplay";
    public static int LED_STRIP_LENGTH = 96;
    public static int DATA_LENGTH = LED_STRIP_LENGTH * 3;
    public static int SERIAL_PAGE_SIZE = 512; // If there are problems transmitting bytes, lower this.

    public enum DisplayMode
    {
        Solid,
        Pulse
    };
}
