from ntcore import NetworkTableInstance
from serial import Serial, SerialTimeoutException, SerialException
from traceback import print_exc

SERIAL_PORT = "COM5"
TEAM_NUMBER = 263
NT_NAME = "DS Laptop"
ROBORIO_BUSY = "b"
ROBORIO_READY = "r"


def send_serial(serial: Serial, value: bool) -> bool:
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
    buttonbox_table = inst.getTable("ButtonBox")

    ledTopic = buttonbox_table.getBooleanTopic("IsRioBusy")
    ledSub = ledTopic.subscribe(None)

    inst.startClient4(NT_NAME)
    inst.setServerTeam(TEAM_NUMBER)
    inst.startDSClient()

    print("Client started")

    try:
        arduino = Serial(SERIAL_PORT)
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
        changes = ledSub.readQueue()
        if not changes:
            continue

        if changes[0] is None:
            continue

        val = changes[0].value
        if not send_serial(arduino, val):
            return


if __name__ == "__main__":
    main()
