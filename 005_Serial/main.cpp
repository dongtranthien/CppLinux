#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <iostream>
#include <string> 


int main()
{
    using LibSerial::SerialPort ;
    using LibSerial::SerialStream ;

    // You can instantiate a Serial Port or a Serial Stream object, whichever you'd prefer to work with.
    // For this example, we will demonstrate by using both types of objects.
    SerialPort serial_port ;

    // Open hardware serial ports using the Open() method.
    serial_port.Open( "/dev/ttyUSB0" ) ;

    // Set the baud rates.
    using LibSerial::BaudRate ;
    serial_port.SetBaudRate( BaudRate::BAUD_115200 ) ;

    // Create a few variables with data we can send.
    char write_byte_1 = 'a' ;
    char write_byte_2 = 'b' ;

    unsigned char read_byte_1 = 'A' ;
    char read_byte_2 = 'B' ;

    // Read a byte to the serial port using SerialPort Write() methods.
    serial_port.WriteByte(write_byte_1) ;

    // Specify a timeout value (in milliseconds).
    size_t timeout_milliseconds = 100000 ;

    using LibSerial::ReadTimeout ;
    while(read_byte_1 == 'A'){
      try
      {
          // Read a byte from the serial port using SerialPort Read() methods.
          serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
      }
      catch (const ReadTimeout&)
      {
          std::cerr << "The Read() call has timed out." << std::endl ;
      }

    }
    std::cout << "serial_port read:   " << std::to_string(read_byte_1) << std::endl ;

    read_byte_1 = 'A';
    while(read_byte_1 == 'A'){
      try
      {
          // Read a byte from the serial port using SerialPort Read() methods.
          serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
      }
      catch (const ReadTimeout&)
      {
          std::cerr << "The Read() call has timed out." << std::endl ;
      }

    }
    std::cout << "serial_port read:   " << std::to_string(read_byte_1) << std::endl ;

    read_byte_1 = 'A';
    while(read_byte_1 == 'A'){
      try
      {
          // Read a byte from the serial port using SerialPort Read() methods.
          serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
      }
      catch (const ReadTimeout&)
      {
          std::cerr << "The Read() call has timed out." << std::endl ;
      }

    }
    std::cout << "serial_port read:   " << std::to_string(read_byte_1) << std::endl ;

    read_byte_1 = 'A';
    while(read_byte_1 == 'A'){
      try
      {
          // Read a byte from the serial port using SerialPort Read() methods.
          serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
      }
      catch (const ReadTimeout&)
      {
          std::cerr << "The Read() call has timed out." << std::endl ;
      }

    }
    std::cout << "serial_port read:   " << std::to_string(read_byte_1) << std::endl ;
    std::cout << "serial_port read:   " << (int)read_byte_1 << std::endl ;

    read_byte_1 = 'A';
    while(read_byte_1 == 'A'){
      try
      {
          // Read a byte from the serial port using SerialPort Read() methods.
          serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
      }
      catch (const ReadTimeout&)
      {
          std::cerr << "The Read() call has timed out." << std::endl ;
      }

    }
    std::cout << "serial_port read:   " << std::to_string(read_byte_1) << std::endl ;

    // Close the Serial Port and Serial Stream.
    serial_port.Close() ;
}