"""K230 IO33 trigger + UART result unit test.

Signal directions:
- Arduino Nano D11 -> voltage divider/level shifter -> K230 IO33
- K230 IO9 UART1_TXD -> Arduino Nano D12 RX
- Common GND

Arduino raises the trigger line briefly. K230 detects the rising edge and sends
TRIGGER_OK over its one-way UART result link.
"""

from machine import FPIOA
from machine import Pin
from machine import UART
import time


UART_TX_PIN = 9
UART_RX_PIN = 10  # Must be configured for UART1 construction; leave physically unconnected.
TRIGGER_PIN = 33
UART_BAUDRATE = 57600


def send_line(uart, text):
    uart.write((text + "\n").encode())
    print("TX ->", text)


def main():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    fpioa.set_function(TRIGGER_PIN, FPIOA.GPIO33)

    trigger = Pin(
        TRIGGER_PIN,
        Pin.IN,
        pull=Pin.PULL_NONE,
        drive=7,
    )

    uart = UART(
        UART.UART1,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )

    trigger_armed = trigger.value() == 0

    print("K230 GPIO trigger test started")
    print("Trigger input: IO%d" % TRIGGER_PIN)
    print("UART1 TX: IO%d baud=%d" % (
        UART_TX_PIN,
        UART_BAUDRATE,
    ))
    send_line(uart, "TRIGGER_READY")

    try:
        while True:
            trigger_level = trigger.value()

            if trigger_level and trigger_armed:
                trigger_armed = False
                print("IO33 rising edge detected")
                send_line(uart, "TRIGGER_OK")
            elif not trigger_level:
                trigger_armed = True

            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("K230 GPIO trigger test stopped")
    except Exception as error:
        print("K230 GPIO trigger test error:", error)
        raise
    finally:
        uart.deinit()


main()
