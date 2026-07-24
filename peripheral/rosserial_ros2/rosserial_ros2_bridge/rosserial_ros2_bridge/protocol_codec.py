import struct
import time

PROTOCOL_VER = 0xFE
ID_PUBLISHER = 0
ID_SUBSCRIBER = 1
ID_SERVICE_SERVER = 2
ID_SERVICE_CLIENT = 4
ID_PARAMETER_REQUEST = 6
ID_LOG = 7
ID_TIME = 10


def checksum(items):
    return 255 - (sum(items) % 256)


def build_packet(topic_id, payload=b""):
    size = len(payload)
    if size > 0xFFFF:
        raise ValueError(f"rosserial payload is too large: {size} bytes")
    size_l = size & 0xFF
    size_h = (size >> 8) & 0xFF
    topic_l = topic_id & 0xFF
    topic_h = (topic_id >> 8) & 0xFF
    msg_checksum = checksum([topic_l, topic_h, *payload])
    return bytes([0xFF, PROTOCOL_VER, size_l, size_h, checksum([size_l, size_h]), topic_l, topic_h]) + payload + bytes([msg_checksum])


def increment_stat(stats, key):
    if stats is not None:
        stats[key] = stats.get(key, 0) + 1


def read_exact(ser, count, deadline):
    out = bytearray()
    while len(out) < count and time.monotonic() < deadline:
        chunk = ser.read(count - len(out))
        if chunk:
            out.extend(chunk)
    return bytes(out) if len(out) == count else None


def read_packet(ser, timeout_s, max_payload_size=65535, stats=None):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        first = ser.read(1)
        if not first:
            continue
        if first[0] != 0xFF:
            increment_stat(stats, "bytes_discarded")
            continue
        second = read_exact(ser, 1, deadline)
        if not second:
            increment_stat(stats, "incomplete_packets")
            return None
        if second[0] != PROTOCOL_VER:
            increment_stat(stats, "bad_protocol")
            continue
        header = read_exact(ser, 3, deadline)
        if not header:
            increment_stat(stats, "incomplete_packets")
            return None
        size_l, size_h, size_chk = header
        if ((size_l + size_h + size_chk) & 0xFF) != 0xFF:
            increment_stat(stats, "bad_size_checksum")
            continue
        size = size_l | (size_h << 8)
        if size > max_payload_size:
            increment_stat(stats, "oversized_payload")
            read_exact(ser, size + 3, deadline)
            continue
        topic_bytes = read_exact(ser, 2, deadline)
        payload = read_exact(ser, size, deadline)
        msg_chk = read_exact(ser, 1, deadline)
        if topic_bytes is None or payload is None or msg_chk is None:
            increment_stat(stats, "incomplete_packets")
            return None
        if ((sum(topic_bytes) + sum(payload) + msg_chk[0]) & 0xFF) != 0xFF:
            increment_stat(stats, "bad_message_checksum")
            continue
        topic_id = topic_bytes[0] | (topic_bytes[1] << 8)
        increment_stat(stats, "packets_ok")
        return topic_id, payload
    return None


def take_u16(payload, offset):
    return struct.unpack_from("<H", payload, offset)[0], offset + 2


def take_u32(payload, offset):
    return struct.unpack_from("<I", payload, offset)[0], offset + 4


def take_i16(payload, offset):
    return struct.unpack_from("<h", payload, offset)[0], offset + 2


def take_u8(payload, offset):
    return payload[offset], offset + 1


def take_i8(payload, offset):
    return struct.unpack_from("<b", payload, offset)[0], offset + 1


def take_bool(payload, offset):
    return payload[offset] != 0, offset + 1


def take_i32(payload, offset):
    return struct.unpack_from("<i", payload, offset)[0], offset + 4


def take_float32(payload, offset):
    return struct.unpack_from("<f", payload, offset)[0], offset + 4


def take_float64(payload, offset):
    return struct.unpack_from("<d", payload, offset)[0], offset + 8


def take_string(payload, offset):
    length = struct.unpack_from("<I", payload, offset)[0]
    offset += 4
    if offset + length > len(payload):
        raise ValueError(f"string field length {length} exceeds remaining payload")
    value = payload[offset:offset + length].decode("utf-8", errors="replace")
    return value, offset + length


def take_float32_array(payload, offset):
    length, offset = take_u32(payload, offset)
    values = []
    for _ in range(length):
        value, offset = take_float32(payload, offset)
        values.append(value)
    return values, offset


def take_float64_array(payload, offset, length):
    values = []
    for _ in range(length):
        value, offset = take_float64(payload, offset)
        values.append(value)
    return values, offset


def decode_topic_info(payload):
    offset = 0
    topic_id, offset = take_u16(payload, offset)
    topic_name, offset = take_string(payload, offset)
    message_type, offset = take_string(payload, offset)
    md5sum, offset = take_string(payload, offset)
    buffer_size, offset = take_i32(payload, offset)
    return {
        "topic_id": topic_id,
        "topic_name": topic_name,
        "message_type": message_type,
        "md5sum": md5sum,
        "buffer_size": buffer_size,
    }


def decode_std_msgs_string(payload):
    value, _ = take_string(payload, 0)
    return value


def decode_std_msgs_bool(payload):
    return bool(payload[0]) if payload else False


def decode_std_msgs_int8(payload):
    value, _ = take_i8(payload, 0)
    return value


def decode_std_msgs_uint8(payload):
    value, _ = take_u8(payload, 0)
    return value


def decode_std_msgs_uint16(payload):
    value, _ = take_u16(payload, 0)
    return value


def decode_std_msgs_int16(payload):
    value, _ = take_i16(payload, 0)
    return value


def decode_std_msgs_int32(payload):
    value, _ = take_i32(payload, 0)
    return value


def decode_std_msgs_uint32(payload):
    value, _ = take_u32(payload, 0)
    return value


def decode_std_msgs_float32(payload):
    value, _ = take_float32(payload, 0)
    return value


def decode_std_msgs_float64(payload):
    value, _ = take_float64(payload, 0)
    return value


def decode_std_msgs_empty(payload):
    return None


def encode_std_msgs_bool(value):
    return bytes([1 if value else 0])


def encode_std_msgs_int8(value):
    return struct.pack("<b", int(value))


def encode_std_msgs_uint8(value):
    return int(value).to_bytes(1, "little")


def encode_std_msgs_uint16(value):
    return int(value).to_bytes(2, "little")


def encode_std_msgs_int16(value):
    return int(value).to_bytes(2, "little", signed=True)


def encode_std_msgs_int32(value):
    return int(value).to_bytes(4, "little", signed=True)


def encode_std_msgs_uint32(value):
    return int(value).to_bytes(4, "little")


def encode_std_msgs_float32(value):
    return struct.pack("<f", float(value))


def encode_std_msgs_float64(value):
    return struct.pack("<d", float(value))


def encode_std_msgs_empty(_value=None):
    return b""


def encode_std_msgs_string(value):
    encoded = value.encode("utf-8")
    return len(encoded).to_bytes(4, "little") + encoded


def encode_int32_array(values):
    out = bytearray()
    out.extend(len(values).to_bytes(4, "little"))
    for value in values:
        out.extend(encode_std_msgs_int32(value))
    return bytes(out)


def encode_float32_array(values):
    out = bytearray()
    out.extend(len(values).to_bytes(4, "little"))
    for value in values:
        out.extend(encode_std_msgs_float32(value))
    return bytes(out)


def encode_string_array(values):
    out = bytearray()
    out.extend(len(values).to_bytes(4, "little"))
    for value in values:
        out.extend(encode_std_msgs_string(str(value)))
    return bytes(out)


def decode_std_msgs_time(payload):
    offset = 0
    sec, offset = take_u32(payload, offset)
    nanosec, _ = take_u32(payload, offset)
    return {"sec": sec, "nanosec": nanosec}


def encode_std_msgs_time(sec, nanosec):
    return int(sec).to_bytes(4, "little") + int(nanosec).to_bytes(4, "little")


def decode_rosserial_log(payload):
    offset = 0
    level, offset = take_u8(payload, offset)
    msg, _ = take_string(payload, offset)
    return {"level": level, "msg": msg}


def decode_request_param_request(payload):
    name, _ = take_string(payload, 0)
    return {"name": name}


def encode_request_param_response(ints=None, floats=None, strings=None):
    return (
        encode_int32_array(list(ints or [])) +
        encode_float32_array(list(floats or [])) +
        encode_string_array(list(strings or []))
    )


def decode_std_srvs_set_bool_request(payload):
    return {"data": decode_std_msgs_bool(payload)}


def encode_std_srvs_set_bool_request(value):
    return encode_std_msgs_bool(value.data)


def decode_std_srvs_set_bool_response(payload):
    offset = 0
    success, offset = take_bool(payload, offset)
    message, offset = take_string(payload, offset)
    return {"success": success, "message": message}


def encode_std_srvs_set_bool_response(value):
    return encode_std_msgs_bool(value.success) + encode_std_msgs_string(value.message)


def decode_std_srvs_trigger_request(payload):
    return {}


def encode_std_srvs_trigger_request(_value):
    return b""


def decode_std_srvs_trigger_response(payload):
    offset = 0
    success, offset = take_bool(payload, offset)
    message, offset = take_string(payload, offset)
    return {"success": success, "message": message}


def encode_std_srvs_trigger_response(value):
    return encode_std_msgs_bool(value.success) + encode_std_msgs_string(value.message)


def decode_std_msgs_header(payload, offset):
    seq, offset = take_u32(payload, offset)
    stamp_sec, offset = take_u32(payload, offset)
    stamp_nanosec, offset = take_u32(payload, offset)
    frame_id, offset = take_string(payload, offset)
    return {
        "seq": seq,
        "stamp_sec": stamp_sec,
        "stamp_nanosec": stamp_nanosec,
        "frame_id": frame_id,
    }, offset


def decode_sensor_msgs_range(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    radiation_type, offset = take_u8(payload, offset)
    field_of_view, offset = take_float32(payload, offset)
    min_range, offset = take_float32(payload, offset)
    max_range, offset = take_float32(payload, offset)
    range_value, offset = take_float32(payload, offset)
    return {
        "header": header,
        "radiation_type": radiation_type,
        "field_of_view": field_of_view,
        "min_range": min_range,
        "max_range": max_range,
        "range": range_value,
    }


def encode_float64(value):
    return struct.pack("<d", float(value))


def decode_std_msgs_color_rgba(payload):
    offset = 0
    r, offset = take_float32(payload, offset)
    g, offset = take_float32(payload, offset)
    b, offset = take_float32(payload, offset)
    a, offset = take_float32(payload, offset)
    return {"r": r, "g": g, "b": b, "a": a}


def encode_std_msgs_color_rgba(value):
    return (
        encode_std_msgs_float32(value.r) +
        encode_std_msgs_float32(value.g) +
        encode_std_msgs_float32(value.b) +
        encode_std_msgs_float32(value.a)
    )


def decode_geometry_msgs_point(payload, offset=0):
    x, offset = take_float64(payload, offset)
    y, offset = take_float64(payload, offset)
    z, offset = take_float64(payload, offset)
    return {"x": x, "y": y, "z": z}, offset


def encode_geometry_msgs_point(value):
    return (
        encode_float64(value.x) +
        encode_float64(value.y) +
        encode_float64(value.z)
    )


def decode_geometry_msgs_vector3(payload, offset=0):
    x, offset = take_float64(payload, offset)
    y, offset = take_float64(payload, offset)
    z, offset = take_float64(payload, offset)
    return {"x": x, "y": y, "z": z}, offset


def encode_geometry_msgs_vector3(value):
    return (
        encode_float64(value.x) +
        encode_float64(value.y) +
        encode_float64(value.z)
    )


def decode_geometry_msgs_quaternion(payload, offset=0):
    x, offset = take_float64(payload, offset)
    y, offset = take_float64(payload, offset)
    z, offset = take_float64(payload, offset)
    w, offset = take_float64(payload, offset)
    return {"x": x, "y": y, "z": z, "w": w}, offset


def encode_geometry_msgs_quaternion(value):
    return (
        encode_float64(value.x) +
        encode_float64(value.y) +
        encode_float64(value.z) +
        encode_float64(value.w)
    )


def decode_geometry_msgs_twist(payload):
    linear, offset = decode_geometry_msgs_vector3(payload, 0)
    angular, offset = decode_geometry_msgs_vector3(payload, offset)
    return {"linear": linear, "angular": angular}


def encode_geometry_msgs_twist(value):
    return encode_geometry_msgs_vector3(value.linear) + encode_geometry_msgs_vector3(value.angular)


def decode_geometry_msgs_pose(payload, offset=0):
    position, offset = decode_geometry_msgs_point(payload, offset)
    orientation, offset = decode_geometry_msgs_quaternion(payload, offset)
    return {"position": position, "orientation": orientation}, offset


def encode_geometry_msgs_pose(value):
    return encode_geometry_msgs_point(value.position) + encode_geometry_msgs_quaternion(value.orientation)


def decode_geometry_msgs_pose_stamped(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    pose, offset = decode_geometry_msgs_pose(payload, offset)
    return {"header": header, "pose": pose}


def encode_geometry_msgs_pose_stamped(value):
    return encode_std_msgs_header(value.header) + encode_geometry_msgs_pose(value.pose)


def decode_geometry_msgs_twist_stamped(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    twist = decode_geometry_msgs_twist(payload[offset:])
    return {"header": header, "twist": twist}


def encode_geometry_msgs_twist_stamped(value):
    return encode_std_msgs_header(value.header) + encode_geometry_msgs_twist(value.twist)


def decode_geometry_msgs_vector3_stamped(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    vector, offset = decode_geometry_msgs_vector3(payload, offset)
    return {"header": header, "vector": vector}


def encode_geometry_msgs_vector3_stamped(value):
    return encode_std_msgs_header(value.header) + encode_geometry_msgs_vector3(value.vector)


def decode_geometry_msgs_quaternion_stamped(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    quaternion, offset = decode_geometry_msgs_quaternion(payload, offset)
    return {"header": header, "quaternion": quaternion}


def encode_geometry_msgs_quaternion_stamped(value):
    return encode_std_msgs_header(value.header) + encode_geometry_msgs_quaternion(value.quaternion)


def encode_std_msgs_header(value):
    frame_id = value.frame_id.encode("utf-8")
    return (
        int(0).to_bytes(4, "little") +
        int(value.stamp.sec).to_bytes(4, "little") +
        int(value.stamp.nanosec).to_bytes(4, "little") +
        len(frame_id).to_bytes(4, "little") +
        frame_id
    )


def decode_covariance_9(payload, offset):
    return take_float64_array(payload, offset, 9)


def decode_sensor_msgs_imu(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    orientation, offset = decode_geometry_msgs_quaternion(payload, offset)
    orientation_covariance, offset = decode_covariance_9(payload, offset)
    angular_velocity, offset = decode_geometry_msgs_vector3(payload, offset)
    angular_velocity_covariance, offset = decode_covariance_9(payload, offset)
    linear_acceleration, offset = decode_geometry_msgs_vector3(payload, offset)
    linear_acceleration_covariance, offset = decode_covariance_9(payload, offset)
    return {
        "header": header,
        "orientation": orientation,
        "orientation_covariance": orientation_covariance,
        "angular_velocity": angular_velocity,
        "angular_velocity_covariance": angular_velocity_covariance,
        "linear_acceleration": linear_acceleration,
        "linear_acceleration_covariance": linear_acceleration_covariance,
    }


def decode_sensor_msgs_magnetic_field(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    magnetic_field, offset = decode_geometry_msgs_vector3(payload, offset)
    magnetic_field_covariance, offset = decode_covariance_9(payload, offset)
    return {
        "header": header,
        "magnetic_field": magnetic_field,
        "magnetic_field_covariance": magnetic_field_covariance,
    }


def decode_sensor_msgs_battery_state(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    voltage, offset = take_float32(payload, offset)
    temperature, offset = take_float32(payload, offset)
    current, offset = take_float32(payload, offset)
    charge, offset = take_float32(payload, offset)
    capacity, offset = take_float32(payload, offset)
    design_capacity, offset = take_float32(payload, offset)
    percentage, offset = take_float32(payload, offset)
    power_supply_status, offset = take_u8(payload, offset)
    power_supply_health, offset = take_u8(payload, offset)
    power_supply_technology, offset = take_u8(payload, offset)
    present, offset = take_bool(payload, offset)
    cell_voltage, offset = take_float32_array(payload, offset)
    cell_temperature, offset = take_float32_array(payload, offset)
    location, offset = take_string(payload, offset)
    serial_number, offset = take_string(payload, offset)
    return {
        "header": header,
        "voltage": voltage,
        "temperature": temperature,
        "current": current,
        "charge": charge,
        "capacity": capacity,
        "design_capacity": design_capacity,
        "percentage": percentage,
        "power_supply_status": power_supply_status,
        "power_supply_health": power_supply_health,
        "power_supply_technology": power_supply_technology,
        "present": present,
        "cell_voltage": cell_voltage,
        "cell_temperature": cell_temperature,
        "location": location,
        "serial_number": serial_number,
    }


def decode_sensor_msgs_laser_scan(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    angle_min, offset = take_float32(payload, offset)
    angle_max, offset = take_float32(payload, offset)
    angle_increment, offset = take_float32(payload, offset)
    time_increment, offset = take_float32(payload, offset)
    scan_time, offset = take_float32(payload, offset)
    range_min, offset = take_float32(payload, offset)
    range_max, offset = take_float32(payload, offset)
    ranges, offset = take_float32_array(payload, offset)
    intensities, offset = take_float32_array(payload, offset)
    return {
        "header": header,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "time_increment": time_increment,
        "scan_time": scan_time,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
        "intensities": intensities,
    }


def decode_sensor_msgs_temperature(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    temperature, offset = take_float64(payload, offset)
    variance, offset = take_float64(payload, offset)
    return {"header": header, "temperature": temperature, "variance": variance}


def decode_sensor_msgs_relative_humidity(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    relative_humidity, offset = take_float64(payload, offset)
    variance, offset = take_float64(payload, offset)
    return {"header": header, "relative_humidity": relative_humidity, "variance": variance}


def decode_sensor_msgs_fluid_pressure(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    fluid_pressure, offset = take_float64(payload, offset)
    variance, offset = take_float64(payload, offset)
    return {"header": header, "fluid_pressure": fluid_pressure, "variance": variance}


def decode_sensor_msgs_illuminance(payload):
    offset = 0
    header, offset = decode_std_msgs_header(payload, offset)
    illuminance, offset = take_float64(payload, offset)
    variance, offset = take_float64(payload, offset)
    return {"header": header, "illuminance": illuminance, "variance": variance}
