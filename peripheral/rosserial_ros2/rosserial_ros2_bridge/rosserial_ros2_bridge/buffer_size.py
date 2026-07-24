#!/usr/bin/env python3
import argparse
from pathlib import Path

from rosserial_ros2_bridge.generic_codec import load_generated_schemas


FRAME_OVERHEAD = 8

BUILTIN_SIZES = {
    "bool": 1,
    "byte": 1,
    "char": 1,
    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "float32": 4,
    "int64": 8,
    "uint64": 8,
    "float64": 8,
    "time": 8,
    "duration": 8,
    "builtin_interfaces/Time": 8,
    "builtin_interfaces/msg/Time": 8,
    "builtin_interfaces/Duration": 8,
    "builtin_interfaces/msg/Duration": 8,
}


def parse_overrides(items):
    overrides = {}
    for item in items:
        if ":=" in item:
            key, value = item.split(":=", 1)
        elif "=" in item:
            key, value = item.split("=", 1)
        else:
            raise ValueError(f"override must use field:=value syntax: {item}")
        key = key.strip()
        if not key:
            raise ValueError(f"empty override field in: {item}")
        overrides[key] = value
    return overrides


def int_override(value, path):
    try:
        parsed = int(value, 10)
    except ValueError as exc:
        raise ValueError(f"{path} must be an integer length/count, got {value!r}") from exc
    if parsed < 0:
        raise ValueError(f"{path} must be >= 0, got {parsed}")
    return parsed


def string_length(value):
    try:
        parsed = int(value, 10)
    except ValueError:
        return len(value.encode("utf-8"))
    return parsed


def round_up(value, align):
    if align <= 1:
        return value
    return ((value + align - 1) // align) * align


def field_path(prefix, name):
    return f"{prefix}.{name}" if prefix else name


def scalar_size(field, schemas, overrides, prefix, notes):
    type_name = field["type"]
    path = field_path(prefix, field["name"])

    if type_name == "string":
        if path in overrides:
            length = string_length(overrides[path])
            notes.append(f"{path}: string length {length}")
        else:
            length = 0
            notes.append(f"{path}: string length 0 default")
        return 4 + length

    if type_name in BUILTIN_SIZES:
        return BUILTIN_SIZES[type_name]

    nested = schemas["messages"].get(type_name)
    if nested is None:
        raise KeyError(f"missing schema for nested message type {type_name}")
    return fields_size(nested["fields"], schemas, overrides, path, notes)


def field_size(field, schemas, overrides, prefix, notes):
    path = field_path(prefix, field["name"])
    if not field["array"]:
        return scalar_size(field, schemas, overrides, prefix, notes)

    if field["array_len"] is None:
        if path in overrides:
            length = int_override(overrides[path], path)
            notes.append(f"{path}: array length {length}")
        else:
            length = 0
            notes.append(f"{path}: array length 0 default")
        total = 4
    else:
        length = int(field["array_len"])
        total = 0

    scalar_field = dict(field)
    scalar_field["array"] = False
    scalar_field["array_len"] = None

    if field["type"] == "string":
        item_path = f"{path}.item"
        item_length = string_length(overrides.get(item_path, "0"))
        if item_path in overrides:
            notes.append(f"{item_path}: string length {item_length}")
        elif length:
            notes.append(f"{item_path}: string length 0 default")
        return total + length * (4 + item_length)

    if field["type"] in BUILTIN_SIZES:
        return total + length * BUILTIN_SIZES[field["type"]]

    item_notes = []
    item_size = scalar_size(scalar_field, schemas, overrides, prefix, item_notes)
    notes.extend(item_notes)
    return total + length * item_size


def fields_size(fields, schemas, overrides, prefix="", notes=None):
    if notes is None:
        notes = []
    return sum(field_size(field, schemas, overrides, prefix, notes) for field in fields)


def select_fields(type_name, schemas, part):
    if type_name in schemas["messages"]:
        return "message", schemas["messages"][type_name]["fields"]
    if type_name in schemas["services"]:
        service = schemas["services"][type_name]
        if part not in ("request", "response"):
            raise ValueError("--part must be request or response for service types")
        return f"service {part}", service[part]["fields"]
    raise KeyError(f"unknown message/service type {type_name}")


def unused_overrides(overrides, notes):
    used_roots = set()
    for note in notes:
        used_roots.add(note.split(":", 1)[0])
    return sorted(key for key in overrides if key not in used_roots)


def main():
    parser = argparse.ArgumentParser(
        description="Estimate rosserial payload/frame sizes for generated ROS 2 message schemas.",
        epilog="Dynamic field hints may be placed anywhere after the type, for example data:=32 or ranges:=30.",
    )
    parser.add_argument("type", help="Message or service type, for example std_msgs/String or std_srvs/Trigger.")
    parser.add_argument("--part", choices=["request", "response"], default="request", help="Service part to estimate.")
    parser.add_argument("--schema", type=Path, default=None, help="Path to generated_interfaces.json.")
    parser.add_argument("--margin", type=int, default=16, help="Extra bytes added to recommended buffer.")
    parser.add_argument("--align", type=int, default=16, help="Round recommended buffer up to this boundary.")
    parser.add_argument("--buffer", type=int, default=None, help="Compare against an existing Arduino buffer size.")
    parser.add_argument("--details", action="store_true", help="Print dynamic-field assumptions.")
    args, overrides_arg = parser.parse_known_args()

    try:
        overrides = parse_overrides(overrides_arg)
        schemas = load_generated_schemas(args.schema)
        kind, fields = select_fields(args.type, schemas, args.part)
        notes = []
        payload = fields_size(fields, schemas, overrides, notes=notes)
    except (KeyError, ValueError) as exc:
        parser.error(str(exc))

    frame = payload + FRAME_OVERHEAD
    recommended = round_up(frame + max(args.margin, 0), args.align)

    print(f"type: {args.type} ({kind})")
    print(f"payload: {payload} bytes")
    print(f"frame: {frame} bytes")
    print(f"recommended buffer: {recommended} bytes")

    if args.buffer is not None:
        status = "OK" if frame <= args.buffer else "TOO SMALL"
        print(f"buffer {args.buffer}: {status}")

    if args.details and notes:
        print("assumptions:")
        for note in notes:
            print(f"  {note}")

    unused = unused_overrides(overrides, notes)
    if unused:
        print("unused overrides:")
        for key in unused:
            print(f"  {key}")


if __name__ == "__main__":
    main()
