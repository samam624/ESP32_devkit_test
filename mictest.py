import argparse
import sys
import time
import wave

import serial
from serial.tools import list_ports


DEFAULT_PORT = "COM13"
BAUD = 921600
SAMPLE_RATE = 16000
CHANNELS = 2
SAMPLE_WIDTH = 2  # int16 samples from the ESP32
OUTPUT_FILE = "esp32_pdm_test.wav"
START_MARKER = b"REC_START\n"
STOP_MARKER = b"REC_STOP\n"


def available_ports():
    return [port.device for port in list_ports.comports()]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record raw int16 stereo audio from an ESP32 button-triggered PDM stream."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Serial baud rate, default {BAUD}")
    parser.add_argument("--sample-rate", type=int, default=SAMPLE_RATE, help=f"WAV sample rate, default {SAMPLE_RATE}")
    parser.add_argument("--channels", type=int, default=CHANNELS, help=f"Audio channels, default {CHANNELS}")
    parser.add_argument(
        "--sample-width",
        type=int,
        default=SAMPLE_WIDTH,
        choices=(2, 3, 4),
        help="Bytes per sample in the incoming stream: 2=16-bit, 3=24-bit packed, 4=32-bit",
    )
    parser.add_argument("--seconds", type=float, help="Optional max seconds to record after the button starts capture")
    parser.add_argument("--output", default=OUTPUT_FILE, help=f"WAV output path, default {OUTPUT_FILE}")
    parser.add_argument("--list-ports", action="store_true", help="Show detected serial ports and exit")
    return parser.parse_args()


def wait_for_marker(ser, marker):
    print("Waiting for ESP32 button press...")
    buffer = bytearray()

    while True:
        chunk = ser.read(256)
        if not chunk:
            continue

        buffer.extend(chunk)
        if marker in buffer:
            return

        # Keep enough bytes to catch a marker split across serial reads.
        if len(buffer) > len(marker):
            del buffer[: -len(marker)]


def read_audio_until_stop(ser, sample_rate, channels, sample_width, max_bytes=None):
    audio = bytearray()
    pending = bytearray()
    last_update = time.monotonic()
    no_data_since = time.monotonic()
    marker_tail = len(STOP_MARKER) - 1

    print("Recording... press the ESP32 button again to stop.")

    while max_bytes is None or len(audio) < max_bytes:
        read_size = 4096
        if max_bytes is not None:
            read_size = min(read_size, max_bytes - len(audio))

        chunk = ser.read(read_size)

        if chunk:
            pending.extend(chunk)
            no_data_since = time.monotonic()
        elif time.monotonic() - no_data_since > 3:
            raise TimeoutError(
                "No audio bytes received for 3 seconds. Check the ESP32 sketch, port, "
                "baud rate, and microphone wiring."
            )

        stop_at = pending.find(STOP_MARKER)
        if stop_at >= 0:
            audio.extend(pending[:stop_at])
            print()
            break

        if len(pending) > marker_tail:
            audio.extend(pending[:-marker_tail])
            del pending[:-marker_tail]

        now = time.monotonic()
        if now - last_update > 0.25:
            seconds = len(audio) / (sample_rate * channels * sample_width)
            print(f"\rReceived {len(audio)} bytes ({seconds:0.1f} s)", end="")
            last_update = now

    frame_size = channels * sample_width
    usable_bytes = len(audio) - (len(audio) % frame_size)
    return bytes(audio[:usable_bytes])


def main():
    args = parse_args()
    ports = available_ports()

    if args.list_ports:
        print("Detected serial ports:")
        for port in ports:
            print(f"  {port}")
        if not ports:
            print("  none")
        return 0

    if args.port not in ports:
        print(f"Warning: {args.port} is not in the detected serial ports: {', '.join(ports) or 'none'}")

    max_bytes = None
    if args.seconds is not None:
        bytes_per_second = args.sample_rate * args.channels * args.sample_width
        max_bytes = int(bytes_per_second * args.seconds)
        frame_size = args.channels * args.sample_width
        max_bytes -= max_bytes % frame_size

    bytes_per_second = args.sample_rate * args.channels * args.sample_width
    serial_bytes_per_second = args.baud / 10
    if bytes_per_second > serial_bytes_per_second:
        print(
            "Warning: this audio format needs about "
            f"{bytes_per_second:,} bytes/s, but {args.baud} baud serial carries only "
            f"about {serial_bytes_per_second:,.0f} bytes/s."
        )

    print(f"Opening {args.port} at {args.baud} baud...")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            # Opening a serial port usually resets an ESP32. Let setup() run, then
            # discard boot chatter or stale bytes so the WAV starts on fresh audio.
            time.sleep(3)
            ser.reset_input_buffer()

            wait_for_marker(ser, START_MARKER)
            audio = read_audio_until_stop(
                ser,
                sample_rate=args.sample_rate,
                channels=args.channels,
                sample_width=args.sample_width,
                max_bytes=max_bytes,
            )
    except serial.SerialException as exc:
        print(f"Serial error opening {args.port}: {exc}", file=sys.stderr)
        if "Access is denied" in str(exc) or "PermissionError" in str(exc):
            print(
                "Close anything else using the port, especially Arduino Serial Monitor, "
                "Serial Plotter, VS Code serial tools, PuTTY, or another Python run. "
                "If it still fails, unplug and reconnect the ESP32.",
                file=sys.stderr,
            )
        return 1
    except TimeoutError as exc:
        print(f"\n{exc}", file=sys.stderr)
        return 1

    with wave.open(args.output, "wb") as wav:
        wav.setnchannels(args.channels)
        wav.setsampwidth(args.sample_width)
        wav.setframerate(args.sample_rate)
        wav.writeframes(audio)

    print(f"Saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
