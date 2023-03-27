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
        Constants.DisplayMode displayMode;
        Dictionary<string, Constants.DisplayMode> displayModes = new Dictionary<string, Constants.DisplayMode>();
        Dictionary<string, string> parameters;
        List<string> positionalArgs;
        string? result = null;

        // Build Dictionary mapping strings to DisplayModes
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
            new string[] { "numleds", "n" },
            new string[] { "id", "i" },
            new string[] { "port", "p"},
        };

        // Parse arguments
        (positionalArgs, parameters) = parseArgs(args, options); // TODO use out

        //GetScreenColors();

        // PARSE PARAMETERS
        displayModes.TryGetValue(positionalArgs[0], out displayMode);


        if (positionalArgs[0].ToLower() == "help" || parameters.ContainsKey("help") || positionalArgs.Count() < 1)
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
                + "        Config:             Applies configuration options like numleds to the Pico."
                + "\n"
                + "    Options:\n"
                + "        -c  --colors    A comma delimited list of colors. Each color may be in hexadecimal or plaintext format (ex: green). Each color will fade into the next in a circular pattern.\n"
                + "        -l  --looptime  A 16 bit unsigned integer representing the number of milliseconds in an animation cycle. May be in decimal or hexadecimal format (ex 512: or 0x200)."
                + "        -n  --numleds   A 16 bit unsigned integer representing the number of LEDs on the LED strip."
            );
        }
        else if (positionalArgs[0].ToLower() == "ls")
        {
            GetSerialPort();
        }
        else if (displayMode == Constants.DisplayMode.Config)
        {
            result = Configure(parameters);
        }
        else
        {
            result = Display(displayMode, parameters);
        }

        if (result != null)
        {
            PrintResult(result);
        }
    }

    /**************************************************************************/
    // COMMANDS
    /**************************************************************************/

    private static string Display(Constants.DisplayMode displayMode, Dictionary<string, string> parameters)
    {
        AssertParameters(parameters, new string[] { "id", "colors" });

        Dictionary<Constants.Config, string> hostConfig;
        SerialPort? serialPort = null;
        List<byte> payload = new();
        List<Color> colors;
        string result;
        ushort loopTime, numLeds;

        try
        {
            // Get the PicoUsbLedStripHost serialPort
            serialPort = GetSerialPort(id: parameters["id"]);
            colors = GetColors(parameters["colors"]);
            hostConfig = GetHostConfig(serialPort);
            numLeds = ParseUShort(hostConfig[Constants.Config.LedStripLength]);

            // Add metadata to payload
            switch (displayMode)
            {
                case Constants.DisplayMode.Solid:
                case Constants.DisplayMode.Stream:
                    break;
                case Constants.DisplayMode.Scroll:
                case Constants.DisplayMode.Pulse:
                case Constants.DisplayMode.SpectrumAnalyzer:
                    AssertParameters(parameters, new string[] { "looptime" });

                    // Get loopTime
                    loopTime = ParseUShort(parameters["looptime"]);

                    payload.Add((byte)loopTime); // Pulse Speed Low
                    payload.Add((byte)(loopTime >> 8)); // Pulse Speed Low

                    break;
                default:
                    throw new Exception("How did you get here??");
            }

            // Add colors to payload
            foreach (Color color in fadeThroughColors(numLeds, colors.ToArray()))
            {
                payload.Add(color.R);
                payload.Add(color.G);
                payload.Add(color.B);
            }

            // Write data to Pico
            result = WriteToSerialPort(displayMode, serialPort, payload);

            // Receive any print statements
            Debug(serialPort);

        }
        finally
        {
            // Close serialPort
            serialPort?.Close();
        }

        return result;
    }

    private static string Configure(Dictionary<string, string> parameters)
    {
        AssertParameters(parameters, new string[] { "numleds", "id", "com" });

        SerialPort? serialPort = null;
        string result;
        ushort numLeds;
        List<byte> payload;

        try
        {
            serialPort = GetSerialPort(port: parameters["com"]);
            numLeds = ParseUShort(parameters["numleds"]);
            payload = new List<byte>
            {
                (byte)numLeds,
                (byte)(numLeds >> 8),
            };

            if (numLeds < 1)
            {
                throw new Exception("numleds must beat least 1.");
            }

            foreach (char c in parameters["id"])
            {
                payload.Add((byte)c);
            }

            payload.Add(0x00);

            result = WriteToSerialPort(Constants.DisplayMode.Config, serialPort, payload);

            Debug(serialPort);
        }
        finally
        {
            // Close serialPort
            serialPort?.Close();
        }

        return result;
    }


    /**************************************************************************/
    // SERIAL
    /**************************************************************************/
    private static string WriteToSerialPort(Constants.DisplayMode displayMode, SerialPort serialPort, List<byte> payload, int retries = 0)
    {
        LinkedList<byte> rawPacket;
        List<byte> packet;
        int count, index;
        byte hash;
        string response = "E";
        ushort length;

        payload.Insert(0, (byte)displayMode);

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
                        return WriteToSerialPort(displayMode, serialPort, payload, retries + 1);
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

    private static SerialPort? GetSerialPort(string? id = null, string? port = null)
    {
        string deviceId;
        SerialPort? serialPort = null;

        foreach (string portName in ComPortNames(VID, PID))
        {
            serialPort = new SerialPort(portName);
            serialPort.ReadTimeout = 100;
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
                deviceId = GetHostConfig(serialPort)[Constants.Config.DeviceId];

                // If neither id not port have been specified, list ports and ids.
                if (id == null && port == null)
                {
                    serialPort.Close();
                    serialPort = null;
                    Console.WriteLine(portName + ":" + deviceId);
                }

                // If port or id match, return the serialPort.
                else if ((id != null && id == deviceId) ||
                         (port != null && port == portName))
                {
                    break;
                }

                // Try the next serialPort.
                else
                {
                    serialPort.Close();
                    serialPort = null;
                }
            }
            catch (TimeoutException)
            {
                serialPort?.Close();
                continue;
            }
        }

        // Make sure serialPort was successfully found
        if (id != null && (serialPort == null || !serialPort.IsOpen))
        {
            throw new Exception("PicoUsbLedStripHost not found.");
        }

        return serialPort;
    }


    /**************************************************************************/
    // ANIMATION
    /**************************************************************************/
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
                     + (color2.R * ((Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.G * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2)))
                     + (color2.G * ((Math.Cos(Math.PI * brightness) + 1) / -2))),

                (byte)((color1.B * (1 - ((Math.Cos(Math.PI * brightness) + 1) / -2)))
                     + (color2.B * ((Math.Cos(Math.PI * brightness) + 1) / -2)))
            );
        }

        return fade;
    }


    /**************************************************************************/
    // PARSING
    /**************************************************************************/
    private static (List<string>, Dictionary<string, string>) parseArgs(string[] args, string[][] options)
    {
        List<string> positionalArgs = new();
        Dictionary<string, string> parameters = new();
        int index = 0;

        // Get all command line arguments
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

        return (positionalArgs, parameters);
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
   
    private static Dictionary<Constants.Config, string> GetHostConfig(SerialPort serialPort)
    {
        Dictionary<Constants.Config, string> hostConfig = new();
        Dictionary<string, Constants.Config> configMap = new();

        foreach (Constants.Config mode in Enum.GetValues<Constants.Config>())
        {
            configMap.Add(((byte)mode).ToString(), mode);
        }

        string result = WriteToSerialPort(Constants.DisplayMode.GetConfig, serialPort, new List<byte>());

        if (result == Constants.SUCCESS)
        {
            foreach (string token in serialPort.ReadLine().TrimEnd(',', '\r').Split(','))
            {
                string[] keyval = token.Split('=');

                if (keyval.Length >= 2)
                {
                    hostConfig.Add(configMap[keyval[0]], keyval[1]);
                }
                else
                {
                    hostConfig.Add(configMap[keyval[0]], "");
                }
            }
        }

        return hostConfig;
    }


    /**************************************************************************/
    // HELPER METHODS
    /**************************************************************************/
    private static void PrintResult(string? result)
    {
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

    private static ushort ParseUShort(string toParse)
    {
        ushort loopTime;
        if (toParse.StartsWith("0x"))
        {
            loopTime = Convert.ToUInt16(toParse, 16);
        }
        else
        {
            loopTime = Convert.ToUInt16(toParse);
        }

        return loopTime;
    }

    private static List<Color> GetColors(string colorString)
    {
        List<Color> colors = new List<Color>();

        foreach (string color in colorString.Split(','))
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

        return colors;
    }

    private static void Debug(SerialPort serialPort)
    {
        try
        {
            while (true) Console.WriteLine("Debug: " + serialPort.ReadLine());
        }
        catch (TimeoutException e) { }
    }

    private static void AssertParameters(Dictionary<string, string> parameters, string[] assertList)
    {
        foreach (string parameter in assertList)
        {
            if (!parameters.ContainsKey(parameter))
            {
                throw new Exception("Missing parameter: " + parameter + ".");
            }
        }
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

    //    for (int i = 0; i < Int32.Parse(hostConfig["LED_STRIP_LENGTH"]); i++)
    //    {
    //        colors.Add(GetColorAt(100, 100));

    //    }

    //    return colors;
    //}
}