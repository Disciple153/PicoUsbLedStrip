// See https://aka.ms/new-console-template for more information
using System.IO.Ports;
using System.Text;

class DeskDisplay
{

    const int LED_STRIP_LENGTH = 96;
    const int DATA_LENGTH = (LED_STRIP_LENGTH * 3);

    static void Main(string[] args)
    {
        SerialPort? serialPort = null;

        try
        {
            // Get the deskDisplay serialPort
            serialPort = GetSerialPort();

            // Make sure serialPort was successfully found
            if (serialPort == null || !serialPort.IsOpen)
            {
                throw new Exception("DeskDisplay not found.");
            }

            // Prepare data to be written
            byte[] ledValues = new byte[DATA_LENGTH];

            for (int i = 0; i < LED_STRIP_LENGTH; i++)
            {
                ledValues[(3 * i) + 0] = (byte)(i + 1);
                ledValues[(3 * i) + 1] = (byte)(i + 1);
                ledValues[(3 * i) + 2] = (byte)(i + 1);
            }

            // Write data
            WriteToSerialPort(serialPort, ledValues);
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
        int count = 8;

        // Write data and confirm it was received
        while (offset + count < ledValues.Length)
        {
            serialPort.Write(ledValues, offset, count);
            Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, count)); // TODO throw exception on false
            offset += count;
        }

        // Write the final chunk of data and confirm it was received
        serialPort.Write(ledValues, offset, ledValues.Length - offset);
        Confirm(serialPort, new List<byte>(ledValues).GetRange(offset, ledValues.Length - offset)); // TODO throw exception on false
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
                    Console.WriteLine("Recieved: " + b.ToString("X"));
                }
                else
                {
                    result = false;
                    Console.WriteLine("Recieved: " + received.ToString("X") + 
                        " Expected: " + b.ToString("X"));
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

    private static SerialPort? GetSerialPort()
    {
        string deviceName;
        SerialPort? serialPort = null;

        foreach (String portName in SerialPort.GetPortNames())
        {
            Console.WriteLine(portName);

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
                deviceName = serialPort.ReadLine();
                Console.WriteLine("Device Name: " + deviceName);
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