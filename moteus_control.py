#!/usr/bin/python3
#
# Description: Applies a constant q-axis voltage to a Moteus controller
#              and prints the resulting state.
#

import asyncio
import moteus
import time

async def main():
    """Main function to connect and control the motor."""
    print("Connecting to Moteus controller...")
    
    # Use 'sudo' if you get a permissions error running this script.
    # The --can-if argument should match your CAN interface name.
    # You can find it by running `ip link show`.
    transport = moteus.Fdcanusb(can_if='can0')
    c = moteus.Controller(id=1, transport=transport) # Target motor ID 1

    # Stop the controller to clear any previous state.
    await c.make_stop()

    # The q-axis voltage to apply (in Volts).
    # Start low and increase carefully.
    # A positive value should spin it one way, negative the other.
    VOLTAGE_Q = 2.0 
    
    print(f"Applying {VOLTAGE_Q}V to q-axis. Press Ctrl+C to stop.")

    try:
        while True:
            # The make_voltage command directly sets the d and q axis voltages.
            # For a standard BLDC, we want Vd=0 and Vq=our target voltage.
            state = await c.set_voltage(vd=0, vq=VOLTAGE_Q, query=True)

            if state:
                print(
                    f"Velocity: {state.values[moteus.Register.VELOCITY]:.2f} rps | "
                    f"Torque: {state.values[moteus.Register.TORQUE]:.2f} Nm | "
                    f"Voltage Q: {state.values[moteus.Register.V_Q]:.2f} V",
                    end='\r'
                )

            # A small delay to not flood the CAN bus. 20-50Hz is plenty.
            await asyncio.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping motor.")
    finally:
        # This is critical. Always ensure the motor is stopped on exit.
        await c.make_stop()
        print("Motor stopped. Script terminated.")


if __name__ == '__main__':
    asyncio.run(main())