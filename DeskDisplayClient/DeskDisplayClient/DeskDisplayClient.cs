// See https://aka.ms/new-console-template for more information
using Microsoft.Win32;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;

class DeskDisplay
{
    const string VID = "2E8A";
    const string PID = "000A";

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;
        List<byte> leds;
        string result = "";
        Stopwatch stopWatch;
        ushort loopTime;

        try
        {
            // Get the deskDisplay serialPort
            serialPort = GetSerialPort();

            // Make sure serialPort was successfully found
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }

            Constants.DisplayMode displayMode = Constants.DisplayMode.SpectrumAnalyzer;
            Color color1 = Color.FromArgb(0x80, 0x00, 0xFF);
            Color color2 = Color.FromArgb(0x00, 0xFF, 0x00);

            Color[] evaColors = 
            {
                color1,
                color1,
                color2,
                color2
            };

            Color[] rainbow = {
                Color.FromArgb(0xFF, 0x00, 0x00),
                Color.FromArgb(0xFF, 0xFF, 0x00),
                Color.FromArgb(0x00, 0xFF, 0x00),
                Color.FromArgb(0x00, 0xFF, 0xFF),
                Color.FromArgb(0x00, 0x00, 0xFF),
                Color.FromArgb(0xFF, 0x00, 0xFF),
            };

            Color[] landon =
            {
                Color.Red,
                Color.Red,
                Color.Yellow,
                Color.Yellow,
                Color.Purple,
                Color.Purple
            };

            switch (displayMode)
            {
                case Constants.DisplayMode.SpectrumAnalyzer:
                    loopTime = 0x8000;

                    leds = new List<byte>
                    {
                        (byte)displayMode,
                        (byte)loopTime, // Pulse Speed Low
                        (byte)(loopTime >> 8), // Pulse Speed High
                    };

                    foreach (Color color in fadeThroughColors(Constants.LED_STRIP_LENGTH, evaColors))
                    {
                        leds.Add(color.R);
                        leds.Add(color.G);
                        leds.Add(color.B);
                    }

                    result = WriteToSerialPort(serialPort, leds);
                    break;

                case Constants.DisplayMode.Solid:
                    leds = new List<byte>
                    {
                        (byte)displayMode
                    };

                    for (int i = 0; i < Constants.LED_STRIP_LENGTH; i++)
                    {
                        leds.Add(color1.R);
                        leds.Add(color1.G);
                        leds.Add(color1.B);
                    }

                    result = WriteToSerialPort(serialPort, leds);
                    break;

                case Constants.DisplayMode.Pulse:
                    loopTime = 0x1000;

                    leds = new List<byte>
                    {
                        (byte)displayMode,
                        (byte)loopTime, // Pulse Speed Low
                        (byte)(loopTime >> 8), // Pulse Speed High
                    };

                    for (int i = 0; i < Constants.LED_STRIP_LENGTH; i++)
                    {
                        leds.Add(color1.R);
                        leds.Add(color1.G);
                        leds.Add(color1.B);
                    }

                    result = WriteToSerialPort(serialPort, leds);
                    break;


                case Constants.DisplayMode.Scroll:
                    loopTime = 0x8000;

                    leds = new List<byte>
                    {
                        (byte)displayMode,
                        (byte)loopTime, // Pulse Speed Low
                        (byte)(loopTime >> 8), // Pulse Speed High
                    };


                    foreach (Color color in fadeThroughColors(Constants.LED_STRIP_LENGTH, evaColors))
                    {
                        leds.Add(color.R);
                        leds.Add(color.G);
                        leds.Add(color.B);
                    }

                    result = WriteToSerialPort(serialPort, leds);
                    break;

                case Constants.DisplayMode.Stream:
                    //https://stackoverflow.com/questions/1483928/how-to-read-the-color-of-a-screen-pixel
                    stopWatch = new Stopwatch();
                    stopWatch.Start();

                    for (byte i = 0x00; i < 0xFF; i++)
                    {
                        leds = new List<byte>
                        {
                            (byte)Constants.DisplayMode.Stream
                        };

                        for (int j = 0; j < Constants.LED_STRIP_LENGTH; j++)
                        {
                            leds.Add((byte)(color1.R * ((float) i / 0xFF)));
                            leds.Add((byte)(color1.G * ((float) i / 0xFF)));
                            leds.Add((byte)(color1.B * ((float) i / 0xFF)));
                        }

                        result = WriteToSerialPort(serialPort, leds);
                    }

                    stopWatch.Stop();
                    Console.WriteLine(1f / (stopWatch.Elapsed.TotalSeconds / 0xFF) + " FPS");
                    break;
            }

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

    private static string WriteToSerialPort(SerialPort serialPort, IEnumerable<byte> payload)
    {

        // Discard any garbage
        serialPort.DiscardInBuffer();

        // Ask to transmit
        serialPort.Write(new byte[] { (byte)Constants.START_CODE }, 0, 1);

        // Receive ready response
        serialPort.ReadTo(Constants.ROM_ID + "\r\n");

        // Build payload
        LinkedList<byte> leds = new(payload);

        // Prepend length
        ushort length = (ushort) payload.Count();

        leds.AddFirst((byte) (length >> 8));
        leds.AddFirst((byte) length);

        // Build hash
        byte hash = 0;

        foreach (byte b in leds)
        {
            hash += b;
        }

        // Append hash
        leds.AddLast(hash);

        // Write payload
        serialPort.Write(leds.ToArray(), 0, leds.Count);

        // Get response
        string response = serialPort.ReadTo("!\r\n");
        //Console.WriteLine("Response: " + response);

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
}