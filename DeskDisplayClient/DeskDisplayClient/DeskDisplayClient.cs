﻿// See https://aka.ms/new-console-template for more information
using Microsoft.Win32;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;

class DeskDisplay
{
    const string VID = "2E8A";
    const string PID = "000A";

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;
        byte[] ledValues;
        List<byte> leds = new List<byte>();
        byte hash = 0;

        Console.WriteLine(Constants.DisplayMode.Solid);

        try
        {
            // Get the deskDisplay serialPort
            serialPort = GetSerialPort();

            // Make sure serialPort was successfully found
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }

            //ushort length = 0x0401;

            //// Should be 0x0102
            //serialPort.Write(new byte[] { (byte) (length >> 8), (byte) length }, 0, 2);

            //// Send 0x0102 empty bytes
            //serialPort.Write(new byte[length], 0, length);

            //// Recieve 0x00 back
            //Console.WriteLine("Hash: " + serialPort.ReadByte().ToString("X2"));

            //// Respond success
            //serialPort.Write(new byte[] { 0x01 }, 0, 1);


            leds.Add((byte)(Constants.LED_STRIP_LENGTH * 3));
            leds.Add((byte) ((Constants.LED_STRIP_LENGTH * 3) >> 8));

            for (int i = 0; i < Constants.LED_STRIP_LENGTH; i++)
            {//9e34eb
                leds.Add(0x80);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
                leds.Add(0x00);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
                leds.Add(0xFF);// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            }

            for (int i = 2; i < leds.Count; i++)
            {
                hash += leds[i];
            }

            leds.Add(0x00);

            Console.WriteLine(
                "Low:  0x" + leds[0].ToString("X2") + "\n" +
                "High: 0x" + leds[1].ToString("X2") + "\n" +
                "Hash: 0x" + hash.ToString("X2") + "\n"
            );

            serialPort.Write(leds.ToArray(), 0, leds.Count);

            //try
            //{
            //    while (true)
            //    {
            //        Console.Write(serialPort.ReadByte().ToString("X2") + " ");
            //    }
            //}
            //catch (TimeoutException)
            //{

            //}

            Console.WriteLine("Hash: 0x" + serialPort.ReadByte().ToString("X2"));

            serialPort.Write(new byte[] { 0x00 }, 0, 1);



            //// Send displayMode
            //WriteToSerialPort(serialPort, new byte[] { (byte)Constants.DisplayMode.Solid });

            //// Prepare data to be written
            //ledValues = new byte[Constants.DATA_LENGTH];

            //for (int i = 0; i < Constants.LED_STRIP_LENGTH; i++)
            //{//9e34eb
            //    ledValues[(3 * i) + 0] = 0x80;// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //    ledValues[(3 * i) + 1] = 0x00;// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //    ledValues[(3 * i) + 2] = 0xFF;// (byte)((0xFF * i) / LED_STRIP_LENGTH);
            //}


            //// Write data
            //WriteToSerialPort(serialPort, ledValues);
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

    private static void WriteToSerialPort(SerialPort serialPort, byte[] ledValues)
    {
        int offset = 0;
        int count = Constants.SERIAL_PAGE_SIZE; // If there are problems transmitting bytes, lower this.
        DateTime dateTime ;

        // Write data and confirm it was received
        while (offset + count < ledValues.Length)
        {
            serialPort.Write(ledValues, offset, count);
            Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, count)); // TODO throw exception on false
            offset += count;
        }

        // Write the final chunk of data and confirm it was received
        serialPort.Write(ledValues, offset, ledValues.Length - offset);
        dateTime = DateTime.Now;
        Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, ledValues.Length - offset)); // TODO throw exception on false
        Console.WriteLine("Confirmation time: " + (DateTime.Now - dateTime).ToString());
    }

    static bool Confirm(SerialPort serialPort, List<byte> data)
    {
        bool thisResult;
        bool result = true;
        byte received;
        byte prev = 0;

        // Get and confirm data until the read operation times out
        foreach (byte b in data)
        {
            try
            {
                // If the next byte was already received, use it
                if (prev != 0)
                {
                    received = prev;
                }
                // If the next byte has not been received, get it
                else
                {
                    received = (byte)serialPort.ReadByte();
                }

                // Determine whether the byte received was the byte expected
                thisResult = b == received;

                // If the byte received could have been an LF replaced with CRLF, check
                if (received == 0x0D && b == 0x0A)
                {
                    prev = received;
                    received = (byte)serialPort.ReadByte();

                    thisResult = b == received;
                }

                // Log the result
                if (thisResult)
                {
                    prev = 0;
                    //Console.WriteLine("Received: " + b.ToString("X2"));
                }
                else
                {
                    result = false;
                    Console.WriteLine("Received: " + received.ToString("X2") + 
                        " Expected: " + b.ToString("X2"));
                }
            }
            catch (TimeoutException)
            {
                Console.WriteLine("No input");
                break;
            }
        }

        return result;
    }

    // FIXME Windows only
    static List<string> ComPortNames(String VID, String PID)
    {
        String pattern = String.Format("^VID_{0}.PID_{1}", VID, PID);
        Regex _rx = new Regex(pattern, RegexOptions.IgnoreCase);
        List<string> comports = new List<string>();

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
                //serialPort.Write(new byte[] { 0x01 }, 0, 1);
                deviceName = serialPort.ReadLine();
                Console.WriteLine("Device Name: " + deviceName);

                if (deviceName == Constants.ROM_ID)  
                {
                    break;
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

    static List<byte> Receive(SerialPort serialPort)
    {
        List<byte> data = new List<byte>();
        bool timeout = false;

        while (!timeout)
        {
            try
            {
                data.Add((byte)serialPort.ReadByte());
            }
            catch (TimeoutException)
            {
                timeout = true;
            }
        }

        return data;
    }

}