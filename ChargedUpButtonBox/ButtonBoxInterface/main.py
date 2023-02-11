from ntcore import NetworkTableInstance
from serial import Serial, SerialTimeoutException, SerialException
from serial.tools import list_ports
from traceback import print_exc

SERIAL_PORT = ""  # Leave empty to try to find port automatically
ARDUINO_NAME = "Arduino Mega 2560"  # The arduino to search for when scanning ports. Can leave blank if not scanning
NT_TABLE_NAME = "ButtonBox"  # Name of the subtable on NT
NT_TOPIC_NAME = "IsRioBusy"  # Name of the topic on NT
TEAM_NUMBER = 263
NT_NAME = "DS Laptop"
ROBORIO_BUSY = "b"
ROBORIO_READY = "r"


def send_serial(serial: Serial, value: bool) -> bool:
    """Sends true or false value over serial to arduino

    Converts the true or false into rio busy or rio ready,
    which are defined by constants set at the top of the file.
    True means busy, and false means ready.

    Paramaters
    ----------
    serial : Serial
        the serial object to send over
    value : bool
        the true or false value to send

    Returns
    --------
    Whether or not it was successful (returns true if timed out)
    """

    try:
        serial.write(
            ROBORIO_BUSY.encode("utf-8") if value else ROBORIO_READY.encode("utf-8")
        )
        print(f"Rio is {'Busy' if value else 'Ready'}")
    except SerialTimeoutException:
        print("Serial Write Timed Out")
    except TypeError:
        print_exc()
        print()
        print("Bad data type sent over serial")
        return False
    except:
        print_exc()
        print()
        print("Error occured while attempting to write to serial")
        return False

    serial.flush()
    return True


def main():
    print("Button Box Interface v1")

    inst = NetworkTableInstance.getDefault()
    buttonbox_table = inst.getTable(NT_TABLE_NAME)  # get NT instance and desired topic

    ledTopic = buttonbox_table.getBooleanTopic(NT_TOPIC_NAME)
    ledSub = ledTopic.subscribe(None)  # get the desired topic and subscribe to it

    # start the client
    inst.startClient4(NT_NAME)
    inst.setServerTeam(TEAM_NUMBER)
    inst.startDSClient()

    print("Client started")

    serial_port = SERIAL_PORT  # assign local variable to constant beacuse python inerpreter is weird

    # find the correct serial port using the arduino name
    if not serial_port:
        ports = list(list_ports.comports())
        for port in ports:
            if ARDUINO_NAME in port.description:
                serial_port = port.device

    try:
        arduino = Serial(serial_port)
    except SerialException:
        print_exc()
        print()
        print(
            "Error occured while attempting to open serial port. Make sure no other program is attmepting to access it"
        )
        return
    except ValueError:
        print_exc()
        print()
        print(
            "Make sure your serial port is a String or None, not an int or other data type"
        )
        print('\t ex. Serial("COM5")')
        return
    except:
        print_exc()
        print()
        print("Soemthing went wrong while trying to open serial")
        return

    while True:
        changes = (
            ledSub.readQueue()
        )  # check if there have been any changes since last check

        if not changes:
            continue

        if changes[0] is None:
            continue

        val = changes[
            0
        ].value  # changes is an array. since only one led would be changed, get the first index

        if not send_serial(arduino, val):  # if sending failed, quit program gracefully
            return


if __name__ == "__main__":
    main()
