from pymodbus.client import ModbusTcpClient
import json
import time

client = ModbusTcpClient("127.0.0.1:502")

def read_register(address):
    result = client.read_holding_registers(address=address, count=1)
    return result.registers[0]

 
with open("actions.json", "r") as file:
    actions = json.load(file)
    

if client.connect():
    print("You are Connected!")

    # Grade D - keep the system running
    while True:

        # Wait until Source 1 has a part
        while True:
            source_1 = read_register(17)

            if source_1 == 1:
                print("Source 1 part detected")
                break
            else:
                print("Waiting for part at Source 1...")
                time.sleep(2)

        # Betyg E
        # Run sequence from JSON
        for action in actions:

            if action["action"] == "move":
                x = action["x"]
                y = action["y"]

                print(f"Moving to X={x}, Y={y}")

                client.write_register(1, x)
                client.write_register(2, y)

                time.sleep(4)

            elif action["action"] == "vacuum":
                value = action["value"]

                print(f"Vacuum = {value}")

                client.write_register(3, value)

                time.sleep(1)

            # Betyg D " lägger till action till process1"
            elif action["action"] == "process1":
                print("Starting Process 1")

                client.write_register(4, 1)

                # Wait until Process 1 starts
                while read_register(19) == 0:
                    time.sleep(0.5)

                print("Process 1 is running")

                # Wait until Process 1 finishes
                while read_register(19) == 1:
                    time.sleep(0.5)

                print("Process 1 is finished")


        print("Part completed!")
        print("Waiting for next part...")

else:
    print("Could not connect.")
