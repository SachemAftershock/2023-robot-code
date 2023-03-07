from ntcore import NetworkTableInstance, StringSubscriber
from serial import Serial, SerialTimeoutException, SerialException
from serial.tools import list_ports
from traceback import print_exc

SERIAL_PORT = ""  # Leave empty to try to find port automatically
ARDUINO_NAME = "Arduino Mega 2560"  # The arduino to search for when scanning ports. Can leave blank if not scanning
TEAM_NUMBER = 263
NT_NAME = "DS Laptop"
ROBORIO_BUSY = "b"
ROBORIO_READY = "r"

NT_WAYPOINT_PATH = "/ButtonBox/Waypoint"
NT_INTAKE_PATH = "/ButtonBox/Intake"
NT_SUPERSTRUCTURE_PATH = "/ButtonBox/Superstructure"
NT_TOGGLE_PATH = "/ButtonBox/Toggle"
NT_JOYSTICK_PATH = "/ButtonBox/Joystick"
NT_MESSAGE_PATH = "/ButtonBox/Message"


def send_serial(serial: Serial, command: str) -> bool:
    """Sends command over serial to arduino

    Paramaters
    ----------
    serial : Serial
        the serial object to send over
    command : str
        the command to send

    Returns
    --------
    Whether or not it was successful (returns true if timed out)
    """

    try:
        serial.write(command.encode("UTF-8"))
        print(command)
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

    waypointSub = inst.getStringTopic(NT_WAYPOINT_PATH).subscribe()
    intakeSub = inst.getStringTopic(NT_INTAKE_PATH).subscribe()
    superstructureSub = inst.getStringTopic(NT_SUPERSTRUCTURE_PATH)
    toggleSub = inst.getStringTopic(NT_TOGGLE_PATH).subscribe()
    joystickSub = inst.getStringTopic(NT_JOYSTICK_PATH).subscribe()
    messageSub = inst.getStringTopic(NT_MESSAGE_PATH).subscribe()

    subList: list[StringSubscriber] = [
        waypointSub,
        intakeSub,
        superstructureSub,
        toggleSub,
        joystickSub,
        messageSub,
    ]

    # start the client
    inst.startClient4(NT_NAME)
    inst.setServerTeam(TEAM_NUMBER)
    inst.startDSClient()

    print("Client started")

    serial_port = (
        SERIAL_PORT  # assign local variable beacuse python inerpreter is weird
    )

    # find the correct serial port using the arduino name
    if not serial_port:
        ports = list(list_ports.comports())
        for port in ports:
            if ARDUINO_NAME in port.description:
                serial_port = port.device

    try:
        arduino = Serial(serial_port, 9600, timeout=5, dsrdtr=True, rtscts=True)
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
        for sub in subList:
            queue = sub.readQueue()

            if not queue:
                continue

            for message in queue:
                if not message:
                    continue

                if not send_serial(arduino, message.value):
                    return


if __name__ == "__main__":
    main()
