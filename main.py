from pymodbus.client import ModbusTcpClient
import json
import time

client = ModbusTcpClient("127.0.0.1")

with open("actions.json", "r") as file:
    actions = json.load(file)

if client.connect():
    print("Connected!")

    for action in actions:

        if action["action"] == "move":
            x = action["x"]
            y = action["y"]

            print(f"Moving to X={x}, Y={y}")

            client.write_register(1, x)  # setX
            client.write_register(2, y)  # setY

            time.sleep(2)

        elif action["action"] == "vacuum":
            value = action["value"]

            print(f"Vacuum = {value}")

            client.write_register(3, value)

            time.sleep(1)

    print("Sequence completed!")

    client.close()

else:
    print("Could not connect.")