#!/usr/bin/env python3
import argparse
import select
import socket
import threading
import time

import serial


def pipe(src, dst, stop_event):
    while not stop_event.is_set():
        try:
            ready, _, _ = select.select([src], [], [], 0.05)
        except (OSError, ValueError):
            break
        if not ready:
            continue
        try:
            data = src.recv(4096)
        except OSError:
            break
        if not data:
            break
        try:
            dst.write(data)
        except serial.SerialException:
            break
    stop_event.set()


def serial_pipe(ser, sock, stop_event):
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


def handle_client(client, address, port, baud):
    print(f"client connected: {address}", flush=True)
    stop_event = threading.Event()
    try:
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            print(f"failed to open {port}: {exc}", flush=True)
            return

        with ser:
            ser.dtr = False
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.dtr = True
            time.sleep(0.1)

            to_serial = threading.Thread(target=pipe, args=(client, ser, stop_event), daemon=True)
            to_socket = threading.Thread(target=serial_pipe, args=(ser, client, stop_event), daemon=True)
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
    parser = argparse.ArgumentParser(description="Expose a Windows COM port as a raw TCP stream.")
    parser.add_argument("--serial-port", default="COM4")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=7777)
    args = parser.parse_args()

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
                handle_client(client, address, args.serial_port, args.baud)
        except KeyboardInterrupt:
            print("stopping relay", flush=True)


if __name__ == "__main__":
    main()
