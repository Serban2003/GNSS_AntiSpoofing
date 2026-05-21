# extract_tlm.py
import struct
import csv
from pathlib import Path


# ================= CONFIG =================
BIN_FILE = "2026_05_17_10_31_26_tlm.bin"
OUT_DIR = "out"

# Use "<" for LITTLE_ENDIAN, ">" for BIG_ENDIAN
NOVATEL_ENDIAN = "<"
# ==========================================


NOVATEL_STREAM_ID = 0x0871
IMU_STREAM_ID = 0x0926

NOVATEL_PACKET_SIZE = 90
IMU_PACKET_SIZE = 40


def read_ccsds_header(data, off, spare_endian=">"):
    stream_id = struct.unpack_from(">H", data, off + 0)[0]
    seq = struct.unpack_from(">H", data, off + 2)[0]
    length = struct.unpack_from(">H", data, off + 4)[0]
    seconds = struct.unpack_from(">I", data, off + 6)[0]
    subsecs = struct.unpack_from(">H", data, off + 10)[0]
    spare = struct.unpack_from(spare_endian + "I", data, off + 12)[0]

    return {
        "CCSDS_STREAMID": stream_id,
        "CCSDS_SEQUENCE": seq,
        "CCSDS_LENGTH": length,
        "CCSDS_SECONDS": seconds,
        "CCSDS_SUBSECS": subsecs,
        "CCSDS_SPARE": spare,
    }


def parse_novatel(data, off):
    if off + NOVATEL_PACKET_SIZE > len(data):
        return None

    h = read_ccsds_header(data, off, spare_endian=">")

    p = off + 16

    values = struct.unpack_from(
        NOVATEL_ENDIAN + "HIdddddddfff",
        data,
        p
    )

    h.update({
        "GPS_WEEKS": values[0],
        "GPS_SECONDS": values[1],
        "GPS_FRAC_SECS": values[2],
        "ECEF_X": values[3],
        "ECEF_Y": values[4],
        "ECEF_Z": values[5],
        "VEL_X": values[6],
        "VEL_Y": values[7],
        "VEL_Z": values[8],
        "LAT": values[9],
        "LON": values[10],
        "ALT": values[11],
    })

    return h


def parse_imu(data, off):
    if off + IMU_PACKET_SIZE > len(data):
        return None

    h = read_ccsds_header(data, off, spare_endian="<")

    p = off + 16

    values = struct.unpack_from("<ffffff", data, p)

    h.update({
        "X_LINEAR_ACCELERATION": values[0],
        "X_ANGULAR_RATE": values[1],
        "Y_LINEAR_ACCELERATION": values[2],
        "Y_ANGULAR_RATE": values[3],
        "Z_LINEAR_ACCELERATION": values[4],
        "Z_ANGULAR_RATE": values[5],
    })

    return h


def write_csv(file_path, rows):
    if not rows:
        return

    with open(file_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main():
    data = Path(BIN_FILE).read_bytes()

    novatel_rows = []
    imu_rows = []

    i = 0

    while i < len(data) - 2:
        stream_id = struct.unpack_from(">H", data, i)[0]

        if stream_id == NOVATEL_STREAM_ID:
            row = parse_novatel(data, i)

            if row is not None:
                novatel_rows.append(row)
                i += NOVATEL_PACKET_SIZE
                continue

        elif stream_id == IMU_STREAM_ID:
            row = parse_imu(data, i)

            if row is not None:
                imu_rows.append(row)
                i += IMU_PACKET_SIZE
                continue

        i += 1

    out = Path(OUT_DIR)
    out.mkdir(exist_ok=True)

    write_csv(out / "novatel_oem615.csv", novatel_rows)
    write_csv(out / "generic_imu.csv", imu_rows)

    print("Done")
    print("NOVATEL packets:", len(novatel_rows))
    print("IMU packets:", len(imu_rows))
    print("Output folder:", OUT_DIR)


if __name__ == "__main__":
    main()