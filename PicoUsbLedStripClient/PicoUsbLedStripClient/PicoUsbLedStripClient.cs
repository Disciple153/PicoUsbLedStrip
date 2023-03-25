// See https://aka.ms/new-console-template for more information
using Microsoft.Win32;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using System.Windows.Forms;

class PicoUsbLedStripClient
{
    const string VID = "2E8A";
    const string PID = "000A";
    const int MAX_RETRIES = 5;

    static void Main(string[] args)
    {
        Dictionary<string, Constants.DisplayMode> displayModes = new Dictionary<string, Constants.DisplayMode>();
        Dictionary<string, string> parameters = new Dictionary<string, string>();
        List<string> positionalArgs = new List<string>();
        SerialPort? serialPort = null;
        List<byte> payload;
        string result;
        ushort loopTime, numLeds;
        int index;

        // Build Dcttionary mapping strings to DisplayModes
        foreach (Constants.DisplayMode mode in Enum.GetValues<Constants.DisplayMode>())
        {
            displayModes.Add(mode.ToString().ToLower(), mode);
        }

        // Define available options
        string[][] options = new string[][]
        {
            new string[] { "colors", "c" },
            new string[] { "looptime", "l" },
            new string[] { "help", "h" },
            new string[] { "numleds", "n" }
        };

        // Get all command line arguments
        index = 0;
        while (index < args.Length)
        {
            string arg = args[index++];

            if (arg.StartsWith("-"))
            {
                string parameterName = getParameterName(arg, options);
                if (parameterName != null) 
                { 
                    switch (parameterName.ToLower())
                    {
                        default:
                            parameters.Add(parameterName.ToLower(), args[index++]);
                            break;
                        case ("help"):
                            parameters.Add(parameterName.ToLower(), "");
                            break;
                    }
                }
                else
                {
                    throw new Exception("Invalid option: " + arg);
                }
            }
            else
            {
                positionalArgs.Add(arg);
            }
        }

        //GetScreenColors();

        if (parameters.ContainsKey("help"))
        {
            Console.WriteLine(
                "PicoUsbLedStripClient.exe: PicoUsbLedStripClient.exe <DisplayMode> [-c <Colors>] [-l <LoopTime>]\n"
                + "    Send a command to the PicoUsbLedStripHost.\n"
                + "\n"
                + "    DisplayModes:\n"
                + "        Solid:              Shows a static color or gradient.\n"
                + "        Pulse:              Shows colors while pulsing from minimum to maximum brightness over looptime.\n"
                + "        Stream:             Shows a static color or gradient without saving to flash memory. This is useful for streaming data to the PicoUsbLedStripHost.\n"
                + "        Scroll:             Scrolls colors across the LED strip with a period of looptime.\n"
                + "        SpectrumAnalyzer    Displays an audio spectrum analysis as brightness over the colors while the colors scroll.\n"
                + "\n"
                + "    Options:\n"
                + "        -c  --colors    A comma delimited list of colors. Each color may be in hexadecimal or plaintext format (ex: green). Each color will fade into the next in a circular pattern.\n"
                + "        -l  --looptime  A 16 bit unsigned integer representing the number of milliseconds in an animation cycle. May be in decimal or hexadecimal format (ex 512: or 0x200)."
            );
            return;
        }

        try
        {
            // Get the PicoUsbLedStripHost serialPort
            serialPort = GetSerialPort();

            List<Color> colors = new List<Color>();

            // Make sure serialPort was successfully found
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("PicoUsbLedStripHost not found.");
            }

            // Get DisplayMode
            Constants.DisplayMode displayMode = displayModes[positionalArgs[0].ToLower()];

            // Get colors
            if (parameters.ContainsKey("colors"))
            {
                foreach (string color in parameters["colors"].Split(','))
                {
                    if (char.IsDigit(color[0]))
                    {
                        colors.Add(Color.FromArgb(Convert.ToInt32("00" + color, 16)));
                    }
                    else
                    {
                        colors.Add(Color.FromName(color));
                    }
                }
            }

            // Add metadata to payload
            switch (displayMode)
            {
                case Constants.DisplayMode.Solid:
                case Constants.DisplayMode.Stream:
                    payload = new List<byte>
                    {
                        (byte)displayMode
                    };

                    // Add colors to payload
                    foreach (Color color in fadeThroughColors(Constants.LED_STRIP_LENGTH, colors.ToArray()))
                    {
                        payload.Add(color.R);
                        payload.Add(color.G);
                        payload.Add(color.B);
                    }

                    break;
                case Constants.DisplayMode.Scroll:
                case Constants.DisplayMode.Pulse:
                case Constants.DisplayMode.SpectrumAnalyzer:
                    // Get loopTime
                    if (parameters["looptime"].StartsWith("0x"))
                    {
                        loopTime = Convert.ToUInt16(parameters["looptime"], 16);
                    }
                    else
                    {
                        loopTime = Convert.ToUInt16(parameters["looptime"]);
                    }

                    payload = new List<byte>
                    {
                        (byte)displayMode,
                        (byte)loopTime, // Pulse Speed Low
                        (byte)(loopTime >> 8), // Pulse Speed High
                    };

                    // Add colors to payload
                    foreach (Color color in fadeThroughColors(Constants.LED_STRIP_LENGTH, colors.ToArray()))
                    {
                        payload.Add(color.R);
                        payload.Add(color.G);
                        payload.Add(color.B);
                    }

                    break;
                case Constants.DisplayMode.Config:

                    // Get numLeds
                    if (parameters["numleds"].StartsWith("0x"))
                    {
                        numLeds = Convert.ToUInt16(parameters["numleds"], 16);
                    }
                    else
                    {
                        numLeds = Convert.ToUInt16(parameters["numleds"]);
                    }

                    payload = new List<byte>
                    {
                        (byte)numLeds,
                        (byte)(numLeds >> 8),
                    };
                    break;
                default:
                    payload = new List<byte>();
                    break;
            }

            // Write data to Pico
            result = WriteToSerialPort(serialPort, payload);

            // Print result
            switch (result)
            {
                case Constants.SUCCESS:
                    Console.WriteLine("Success");
                    break;

                case Constants.FAILURE:
                    Console.WriteLine("Failure");
                    break;

                case Constants.TIMEOUT:
                    Console.WriteLine("Timeout");
                    break;

                default:
                    Console.WriteLine("Error");
                    break;
            }
        }
        finally
        {
            // Close serialPort
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
            }
        }
    }

    private static Color[] fadeThroughColors(int resolution, Color[] colors)
    {
        Color[] fade = new Color[resolution];
        Color color1;
        Color color2;
        float brightness;
        int colorIndex;

        for (int i = 0; i < resolution; i++)
        {
            brightness = (i % ((float)resolution / colors.Length)) / (resolution / colors.Length);
            colorIndex = (i * colors.Length) / resolution;
            color1 = colors[colorIndex];

            colorIndex++;

            if (colorIndex >= colors.Length)
            {
                colorIndex = 0;
            }

            color2 = colors[colorIndex];

            fade[i] = Color.FromArgb(
                (byte)((color1.R * (1 - brightness)) + (color2.R * brightness)),
                (byte)((color1.G * (1 - brightness)) + (color2.G * brightness)),
                (byte)((color1.B * (1 - brightness)) + (color2.B * brightness))
            );
        }

        return fade;
    }

    private static Color[] fadeThroughColorsCos(int resolution, Color[] colors)
    {
        Color[] fade = new Color[resolution];
        Color color1;
        Color color2;
        float brightness;
        int colorIndex;

        for (int i = 0; i < resolution; i++)
        {
            brightness = (i % ((float)resolution / colors.Length)) / (resolution / colors.Length);
            colorIndex = (i * colors.Length) / resolution;
            color1 = colors[colorIndex];

            colorIndex++;

            if (colorIndex >= colors.Length)
            {
                colorIndex = 0;
            }

            color2 = colors[colorIndex];

            fade[i] = Color.FromArgb(
                (byte)((color1.R * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2))) 
                     + (color2.R * (     (Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.G * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2))) 
                     + (color2.G * (     (Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.B * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2))) 
                     + (color2.B * (     (Math.Cos(Math.PI * brightness) + 1) / -2)))
            );
        }

        return fade;
    }

    private static string WriteToSerialPort(SerialPort serialPort, List<byte> payload, int retries = 0)
    {
        LinkedList<byte> rawPacket;
        List<byte> packet;
        int count, index;
        byte hash;
        string response = "E";
        ushort length;

        // Discard any garbage
        serialPort.DiscardInBuffer();

        // Ask to transmit
        serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);

        // Receive ready response
        serialPort.ReadTo(Constants.ROM_ID + "\r\n");

        index = 0;
        while (index < payload.Count())
        {
            // Extract a packet with a size <= Constants.SERIAL_PAGE_SIZE
            if (index + Constants.SERIAL_PAGE_SIZE > payload.Count())
                count = payload.Count() - index;
            else
                count = Constants.SERIAL_PAGE_SIZE;

            packet = payload.GetRange(index, count);

            // Build packet
            rawPacket = new(packet);

            // Build hash
            hash = 0;

            foreach (byte b in rawPacket)
            {
                hash += b;
            }

            // Append hash (length is not included)
            rawPacket.AddLast(hash);

            // If this is the first packet, prepend the total transmission length
            if (index == 0)
            {
                length = (ushort)payload.Count();

                rawPacket.AddFirst((byte)(length >> 8));
                rawPacket.AddFirst((byte)length);
            }

            // Write packet
            serialPort.Write(rawPacket.ToArray(), 0, rawPacket.Count);

            // Get response
            response = serialPort.ReadTo("!\r\n");

            switch (response[response.Length - 1] + "!")
            {
                // If Success, send next packet
                case Constants.SUCCESS:
                    index += Constants.SERIAL_PAGE_SIZE;
                    break;

                // If Failure, resend packet
                case Constants.FAILURE:
                    break;

                // If Timeout, retry from beginning
                case Constants.TIMEOUT:
                    if (retries < MAX_RETRIES)
                        return WriteToSerialPort(serialPort, payload, retries + 1);
                    else
                        throw new Exception("Too many retries");

                // Otherwise, unknown transmission error.
                default:
                    throw new Exception("Unknown transmission error");
            }
        }

        return response[response.Length - 1] + "!";
    }

    // FIXME Windows only
    static List<string> ComPortNames(String VID, String PID)
    {
        String pattern = String.Format("^VID_{0}.PID_{1}", VID, PID);
        Regex _rx = new(pattern, RegexOptions.IgnoreCase);
        List<string> comports = new();

        RegistryKey rk1 = Registry.LocalMachine;
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");

        foreach (String s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);
            foreach (String s in rk3.GetSubKeyNames())
            {
                //Console.WriteLine("RK3: " + s);
                if (_rx.Match(s).Success)
                {
                    RegistryKey rk4 = rk3.OpenSubKey(s);
                    foreach (String s2 in rk4.GetSubKeyNames())
                    {
                        RegistryKey rk5 = rk4.OpenSubKey(s2);
                        string location = (string)rk5.GetValue("LocationInformation");
                        RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");
                        string portName = (string)rk6.GetValue("PortName");
                        if (!String.IsNullOrEmpty(portName) && SerialPort.GetPortNames().Contains(portName))
                            comports.Add((string)rk6.GetValue("PortName"));
                    }
                }
            }
        }
        return comports;
    }

    private static SerialPort? GetSerialPort()
    {
        string deviceName;
        SerialPort? serialPort = null;

        foreach (String portName in ComPortNames(VID, PID))
        {
            Console.WriteLine(portName);

            serialPort = new SerialPort(portName);
            serialPort.ReadTimeout = 5000;
            serialPort.WriteTimeout = 1000;
            serialPort.BaudRate = 115200;
            serialPort.RtsEnable = true;
            serialPort.DtrEnable = true;

            try
            {
                serialPort.Open();
            }
            catch (IOException)
            {
                continue;
            }

            try
            {
                serialPort.DiscardInBuffer();
                serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);
                deviceName = serialPort.ReadLine().Trim('\r', '\n');
                Console.WriteLine("Device Name: " + deviceName);

                if (deviceName == Constants.ROM_ID)  
                {
                    serialPort.ReadTo(Constants.TIMEOUT + "\r\n");
                    break;
                }
                else
                {
                    serialPort = null;
                }
            }
            catch (TimeoutException)
            {
                serialPort.Close();
                continue;
            }
        }

        return serialPort;
    }

    private static string? getParameterName(string argument, string[][] options)
    {
        string? parameterName = null;
        int index = 0;
        argument = argument.TrimStart('-');

        while (index < options.Length && parameterName == null)
        {
            string[] parameter = options[index++];

            if (parameter.Contains(argument))
            {
                parameterName = parameter[0];
            }
        }

        return parameterName;
    }

    private Dictionary<string,string> GetHostConfig(SerialPort serialPort)
    {
        Dictionary<string, string> hostConfig = new();

        List<byte> payload = new List<byte>
        {
            (byte)Constants.DisplayMode.GetConfig
        };

        string result = WriteToSerialPort(serialPort, payload);

        if (result == Constants.SUCCESS)
        {
            foreach (string token in serialPort.ReadLine().Split(','))
            {
                string[] keyval = token.Split('=');

                hostConfig.Add(keyval[0], keyval[1]);
            }
        }

        return hostConfig;
    }


    //// TODO: This solution is slow. Try different approaches:
    //// https://stackoverflow.com/questions/1483928/how-to-read-the-color-of-a-screen-pixel
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern IntPtr GetDesktopWindow();
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern IntPtr GetWindowDC(IntPtr window);
    //[DllImport("gdi32.dll", SetLastError = true)]
    //public static extern uint GetPixel(IntPtr dc, int x, int y);
    //[DllImport("user32.dll", SetLastError = true)]
    //public static extern int ReleaseDC(IntPtr window, IntPtr dc);

    //public static Color GetColorAt(int x, int y)
    //{
    //    IntPtr desk = GetDesktopWindow();
    //    IntPtr dc = GetWindowDC(desk);
    //    int a = (int)GetPixel(dc, x, y);
    //    ReleaseDC(desk, dc);
    //    return Color.FromArgb(255, (a >> 0) & 0xff, (a >> 8) & 0xff, (a >> 16) & 0xff);
    //}

    //public static List<Color> GetScreenColors()
    //{
    //    List<Color> colors = new();
    //    Rectangle screenBounds = Screen.PrimaryScreen.Bounds;

    //    for (int i = 0; i < Constants.LED_STRIP_LENGTH; i++)
    //    {
    //        colors.Add(GetColorAt(100, 100));

    //    }

    //    return colors;
    //}
}