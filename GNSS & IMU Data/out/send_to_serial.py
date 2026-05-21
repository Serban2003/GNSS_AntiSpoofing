# send_csv_to_uart_rates.py
import csv
import struct
import time
import serial


# ================= CONFIG =================
NOVATEL_CSV = "2026_05_17_10_31_26_tlm_spoofed\\novatel_oem615.csv"
IMU_CSV = "2026_05_17_10_31_26_tlm_spoofed\\generic_imu.csv"

#NOVATEL_CSV = "2026_05_17_10_13_07_tlm\\novatel_oem615.csv"
#IMU_CSV = "2026_05_17_10_13_07_tlm\\generic_imu.csv"

SERIAL_PORT = "COM5"
BAUDRATE = 115200

# NOVATEL / GNSS is faster
NOVATEL_RATE_HZ = 4

# IMU is slower
IMU_RATE_HZ = 1

NOVATEL_ENDIAN = ">"
IMU_ENDIAN = ">"

PRINT_HEX = True
LOOP_DATA = True
# ==========================================


def u16(v):
    return int(float(v)) & 0xFFFF


def u32(v):
    return int(float(v)) & 0xFFFFFFFF


def f32(v):
    return float(v)


def f64(v):
    return float(v)


def build_ccsds_header(row, spare_endian=">"):
    pkt = b""

    pkt += struct.pack(">H", u16(row["CCSDS_STREAMID"]))
    pkt += struct.pack(">H", u16(row["CCSDS_SEQUENCE"]))
    pkt += struct.pack(">H", u16(row["CCSDS_LENGTH"]))
    pkt += struct.pack(">I", u32(row["CCSDS_SECONDS"]))
    pkt += struct.pack(">H", u16(row["CCSDS_SUBSECS"]))
    pkt += struct.pack(spare_endian + "I", u32(row["CCSDS_SPARE"]))

    return pkt


def build_novatel_packet(row):
    pkt = build_ccsds_header(row, spare_endian=">")

    pkt += struct.pack(
        NOVATEL_ENDIAN + "HIdddddddfff",
        u16(row["GPS_WEEKS"]),
        u32(row["GPS_SECONDS"]),
        f64(row["GPS_FRAC_SECS"]),
        f64(row["ECEF_X"]),
        f64(row["ECEF_Y"]),
        f64(row["ECEF_Z"]),
        f64(row["VEL_X"]),
        f64(row["VEL_Y"]),
        f64(row["VEL_Z"]),
        f32(row["LAT"]),
        f32(row["LON"]),
        f32(row["ALT"]),
    )

    return pkt


def build_imu_packet(row):
    pkt = build_ccsds_header(row, spare_endian="<")

    pkt += struct.pack(
        IMU_ENDIAN + "ffffff",
        f32(row["X_LINEAR_ACCELERATION"]),
        f32(row["X_ANGULAR_RATE"]),
        f32(row["Y_LINEAR_ACCELERATION"]),
        f32(row["Y_ANGULAR_RATE"]),
        f32(row["Z_LINEAR_ACCELERATION"]),
        f32(row["Z_ANGULAR_RATE"]),
    )

    return pkt


def read_csv_rows(file_path):
    with open(file_path, "r", newline="") as f:
        return list(csv.DictReader(f))


def send_packet(ser, pkt, name):
    if PRINT_HEX:
        print(name, pkt.hex(" ").upper())

    ser.write(pkt)
    ser.flush()


def main():
    novatel_rows = read_csv_rows(NOVATEL_CSV)
    imu_rows = read_csv_rows(IMU_CSV)

    print("NOVATEL rows:", len(novatel_rows))
    print("IMU rows:", len(imu_rows))

    novatel_period = 1.0 / NOVATEL_RATE_HZ
    imu_period = 1.0 / IMU_RATE_HZ

    next_novatel_time = time.monotonic()
    next_imu_time = time.monotonic()

    novatel_index = 0
    imu_index = 0

    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
    )

    print("UART opened:", SERIAL_PORT)
    print("NOVATEL rate:", NOVATEL_RATE_HZ, "Hz")
    print("IMU rate:", IMU_RATE_HZ, "Hz")

    try:
        while True:
            now = time.monotonic()

            if now >= next_novatel_time:
                row = novatel_rows[novatel_index]
                pkt = build_novatel_packet(row)
                send_packet(ser, pkt, "NOVATEL")

                novatel_index += 1
                if novatel_index >= len(novatel_rows):
                    if LOOP_DATA:
                        novatel_index = 0
                    else:
                        break

                next_novatel_time += novatel_period

            if now >= next_imu_time:
                row = imu_rows[imu_index]
                pkt = build_imu_packet(row)
                send_packet(ser, pkt, "IMU")

                imu_index += 1
                if imu_index >= len(imu_rows):
                    if LOOP_DATA:
                        imu_index = 0
                    else:
                        break

                next_imu_time += imu_period

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        ser.close()
        print("UART closed")


if __name__ == "__main__":
    main()