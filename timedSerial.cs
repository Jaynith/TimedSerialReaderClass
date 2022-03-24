using System;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace fastSerialReaderPrototype
{
    public class DataStreamEventArgs : EventArgs
    {
        private byte[] _byteArray;

        public DataStreamEventArgs(byte[] byteArray)
        {
            _byteArray = byteArray;
        }

        public byte[] Response
        {
            get { return _byteArray; }
        }
    }

    public class SerialClient : IDisposable
    {

        private String _port;
        private UInt32 _baudRate;
        private SerialPort _serialPort;
        private Thread serThread;
        private Double _packetRate;
        private DateTime _lastReceive;

        private const int freqCriticalLimit = 285;

        public SerialClient(String port)
        {
            _port = port;
            _baudRate = 921600;
            _lastReceive = DateTime.MinValue;

            serThread = new Thread(new ThreadStart(SerialReceiving));
            serThread.Priority = ThreadPriority.AboveNormal;
            serThread.Name = "SerialHandler" + serThread.ManagedThreadId;
        }

        public SerialClient (String port, UInt32 baudRate) 
            : this(port)
        {
            _baudRate = baudRate;
        }

        private void SerialReceiving()
        {

        }

        public void Dispose()
        {
            
        }
    }
}
