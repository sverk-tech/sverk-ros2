import importlib
import json
from pathlib import Path
import struct


BUILTIN_FORMATS = {
    "bool": "<?",
    "int8": "<b",
    "uint8": "<B",
    "byte": "<B",
    "char": "<b",
    "int16": "<h",
    "uint16": "<H",
    "int32": "<i",
    "uint32": "<I",
    "int64": "<q",
    "uint64": "<Q",
    "float32": "<f",
    "float64": "<d",
}


def load_generated_schemas(path=None):
    if path is None:
        path = Path(__file__).with_name("generated_interfaces.json")
    path = Path(path)
    if not path.exists():
        return {"messages": {}, "services": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def ros2_message_class(rosserial_type):
    package, name = rosserial_type.split("/", 1)
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def ros2_service_class(rosserial_type):
    package, name = rosserial_type.split("/", 1)
    module = importlib.import_module(f"{package}.srv")
    return getattr(module, name)


def read_value(payload, offset, field, schemas):
    if field["array"]:
        if field["array_len"] is None:
            length = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
        else:
            length = field["array_len"]
        values = []
        scalar_field = dict(field)
        scalar_field["array"] = False
        scalar_field["array_len"] = None
        for _ in range(length):
            value, offset = read_value(payload, offset, scalar_field, schemas)
            values.append(value)
        return values, offset

    type_name = field["type"]
    if type_name == "string":
        length = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        value = payload[offset:offset + length].decode("utf-8", errors="replace")
        return value, offset + length
    if type_name in ("time", "builtin_interfaces/Time", "builtin_interfaces/msg/Time"):
        sec = struct.unpack_from("<I", payload, offset)[0]
        nanosec = struct.unpack_from("<I", payload, offset + 4)[0]
        return {"sec": sec, "nanosec": nanosec}, offset + 8
    if type_name in ("duration", "builtin_interfaces/Duration", "builtin_interfaces/msg/Duration"):
        sec = struct.unpack_from("<i", payload, offset)[0]
        nanosec = struct.unpack_from("<i", payload, offset + 4)[0]
        return {"sec": sec, "nanosec": nanosec}, offset + 8
    if type_name in BUILTIN_FORMATS:
        fmt = BUILTIN_FORMATS[type_name]
        return struct.unpack_from(fmt, payload, offset)[0], offset + struct.calcsize(fmt)

    nested_schema = schemas["messages"][type_name]
    decoded, offset = decode_fields(payload, offset, nested_schema["fields"], schemas)
    msg = ros2_message_class(type_name)()
    return populate_message(msg, decoded), offset


def write_value(value, field, schemas):
    if field["array"]:
        values = list(value)
        out = bytearray()
        if field["array_len"] is None:
            out.extend(len(values).to_bytes(4, "little"))
        scalar_field = dict(field)
        scalar_field["array"] = False
        scalar_field["array_len"] = None
        for item in values:
            out.extend(write_value(item, scalar_field, schemas))
        return bytes(out)

    type_name = field["type"]
    if type_name == "string":
        encoded = str(value).encode("utf-8")
        return len(encoded).to_bytes(4, "little") + encoded
    if type_name in ("time", "builtin_interfaces/Time", "builtin_interfaces/msg/Time"):
        sec = getattr(value, "sec", value.get("sec", 0))
        nanosec = getattr(value, "nanosec", value.get("nanosec", 0))
        return int(sec).to_bytes(4, "little") + int(nanosec).to_bytes(4, "little")
    if type_name in ("duration", "builtin_interfaces/Duration", "builtin_interfaces/msg/Duration"):
        sec = getattr(value, "sec", value.get("sec", 0))
        nanosec = getattr(value, "nanosec", value.get("nanosec", 0))
        return int(sec).to_bytes(4, "little", signed=True) + int(nanosec).to_bytes(4, "little", signed=True)
    if type_name in BUILTIN_FORMATS:
        return struct.pack(BUILTIN_FORMATS[type_name], value)

    nested_schema = schemas["messages"][type_name]
    return encode_fields(value, nested_schema["fields"], schemas)


def decode_fields(payload, offset, fields, schemas):
    decoded = {}
    for field in fields:
        value, offset = read_value(payload, offset, field, schemas)
        decoded[field["name"]] = value
    return decoded, offset


def encode_fields(msg, fields, schemas):
    out = bytearray()
    for field in fields:
        if hasattr(msg, field["name"]):
            value = getattr(msg, field["name"])
        elif field["name"] == "seq":
            value = 0
        else:
            raise AttributeError(f"{type(msg).__name__} has no field {field['name']}")
        out.extend(write_value(value, field, schemas))
    return bytes(out)


def assign_value(target, name, value):
    current = getattr(target, name)
    if hasattr(current, "sec") and isinstance(value, dict):
        current.sec = int(value["sec"])
        current.nanosec = int(value["nanosec"])
    else:
        setattr(target, name, value)


def populate_message(msg, decoded):
    for name, value in decoded.items():
        if not hasattr(msg, name):
            continue
        current = getattr(msg, name)
        if hasattr(current, "get_fields_and_field_types") and isinstance(value, dict):
            populate_message(current, value)
        else:
            assign_value(msg, name, value)
    return msg


def make_message_decoder(ros_msg_type, schema):
    def decode(_node, payload):
        decoded, _ = decode_fields(payload, 0, schema["fields"], load_generated_schemas())
        msg = ros_msg_type()
        return populate_message(msg, decoded)

    return decode


def make_message_encoder(schema):
    def encode(_node, msg):
        return encode_fields(msg, schema["fields"], load_generated_schemas())

    return encode


def make_service_request_decoder(ros_srv_type, schema):
    def decode(_node, payload):
        decoded, _ = decode_fields(payload, 0, schema["request"]["fields"], load_generated_schemas())
        request = ros_srv_type.Request()
        return populate_message(request, decoded)

    return decode


def make_service_request_encoder(schema):
    def encode(_node, request):
        return encode_fields(request, schema["request"]["fields"], load_generated_schemas())

    return encode


def make_service_response_decoder(ros_srv_type, schema):
    def decode(_node, payload):
        decoded, _ = decode_fields(payload, 0, schema["response"]["fields"], load_generated_schemas())
        response = ros_srv_type.Response()
        return populate_message(response, decoded)

    return decode


def make_service_response_encoder(schema):
    def encode(_node, response):
        return encode_fields(response, schema["response"]["fields"], load_generated_schemas())

    return encode
