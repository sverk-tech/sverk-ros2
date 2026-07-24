#!/usr/bin/env python3
import argparse
import socket
import threading
import time

import serial


def socket_to_serial(sock, ser, stop_event):
    while not stop_event.is_set():
        try:
            data = sock.recv(4096)
        except OSError:
            break
        if not data:
            break
        try:
            ser.write(data)
        except serial.SerialException:
            break
    stop_event.set()


def serial_to_socket(ser, sock, stop_event):
    while not stop_event.is_set():
        try:
            data = ser.read(4096)
        except serial.SerialException:
            break
        if not data:
            continue
        try:
            sock.sendall(data)
        except OSError:
            break
    stop_event.set()


def open_serial(port, baud, exclusive):
    try:
        return serial.Serial(port, baud, timeout=0.05, exclusive=exclusive)
    except TypeError:
        return serial.Serial(port, baud, timeout=0.05)


def reset_arduino(ser):
    ser.dtr = False
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.dtr = True
    time.sleep(0.3)


def handle_client(client, address, port, baud, reset, exclusive):
    print(f"client connected: {address}", flush=True)
    stop_event = threading.Event()
    try:
        try:
            ser = open_serial(port, baud, exclusive)
        except serial.SerialException as exc:
            print(f"failed to open {port}: {exc}", flush=True)
            return

        with ser:
            if reset:
                reset_arduino(ser)
            else:
                ser.reset_input_buffer()

            to_serial = threading.Thread(
                target=socket_to_serial,
                args=(client, ser, stop_event),
                daemon=True,
            )
            to_socket = threading.Thread(
                target=serial_to_socket,
                args=(ser, client, stop_event),
                daemon=True,
            )
            to_serial.start()
            to_socket.start()

            while not stop_event.is_set():
                time.sleep(0.1)
    finally:
        stop_event.set()
        try:
            client.close()
        except OSError:
            pass
        print(f"client disconnected: {address}", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Expose a Linux serial device as a raw TCP stream."
    )
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=7777)
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not toggle DTR when a TCP client connects.",
    )
    parser.add_argument(
        "--no-exclusive",
        action="store_true",
        help="Do not request exclusive access to the serial device.",
    )
    args = parser.parse_args()

    reset = not args.no_reset
    exclusive = not args.no_exclusive

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.listen_host, args.listen_port))
        server.listen(1)
        server.settimeout(0.2)
        print(
            f"forwarding {args.serial_port} @ {args.baud} on "
            f"{args.listen_host}:{args.listen_port}",
            flush=True,
        )
        try:
            while True:
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                handle_client(
                    client,
                    address,
                    args.serial_port,
                    args.baud,
                    reset,
                    exclusive,
                )
        except KeyboardInterrupt:
            print("stopping relay", flush=True)


if __name__ == "__main__":
    main()
