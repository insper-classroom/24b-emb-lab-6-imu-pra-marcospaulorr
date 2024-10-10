import serial
import uinput
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

# Create new mouse device
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
    uinput.REL_X,
    uinput.REL_Y,
])


def parse_data(data):
    axis = data[0]  # 0 for X, 1 for Y
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    print(f"Received data: {data}")
    print(f"axis: {axis}, value: {value}")
    return axis, value


def move_mouse(data):
    if data['type'] == 'movement':
        x = data['x'] // 2  # Aplica fator de escala adicional
        y = data['y'] // 2
        device.emit(uinput.REL_X, x)
        device.emit(uinput.REL_Y, y)
    elif data['type'] == 'click':
        # Simula um clique do mouse
        device.emit(uinput.BTN_LEFT, 1)  # Pressiona o botão
        time.sleep(0.05)
        device.emit(uinput.BTN_LEFT, 0)  # Solta o botão


try:
    # sync package
    while True:
        print('Waiting for sync package...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break
            else:
                print(f"Received error: {data}")

        # Read 4 bytes from UART
        data = ser.read(3)
        axis, value = parse_data(data)
        move_mouse(axis, value)

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()
