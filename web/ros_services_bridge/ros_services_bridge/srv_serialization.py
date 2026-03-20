"""
Convert ROS2 service Request/Response to/from JSON-serializable dict.
"""

import os
import re
import importlib
from typing import Any, Dict, Type


def get_srv_class(service_type: str):
    parts = service_type.split("/")
    if len(parts) != 3:
        return None, None, None
    module_name = f"{parts[0]}.{parts[1]}"
    try:
        mod = importlib.import_module(module_name)
        srv_class = getattr(mod, parts[2], None)
        if srv_class is None:
            return None, None, None
        return srv_class, srv_class.Request, srv_class.Response
    except Exception:
        return None, None, None


def _get_fields_and_types(msg_or_class) -> Dict[str, str]:
    if hasattr(msg_or_class, "get_fields_and_field_types"):
        return msg_or_class.get_fields_and_field_types()
    return {}


def _is_ros_message_instance(obj) -> bool:
    return hasattr(obj, "get_fields_and_field_types")


def response_to_dict(msg) -> Dict[str, Any]:
    if msg is None:
        return {}
    if type(msg) in (str, bool, int, float):
        return msg
    if type(msg) is bytes:
        import base64
        return base64.b64encode(msg).decode()
    if type(msg) in (list, tuple):
        return [response_to_dict(x) for x in msg]
    out = {}
    for field in _get_fields_and_types(msg):
        value = getattr(msg, field, None)
        if value is None:
            out[field] = None
        elif type(value) in (str, bool, int, float):
            out[field] = value
        elif type(value) is bytes:
            import base64
            out[field] = base64.b64encode(value).decode()
        elif type(value) in (list, tuple):
            out[field] = [response_to_dict(x) for x in value]
        elif _is_ros_message_instance(value):
            out[field] = response_to_dict(value)
        else:
            out[field] = value
    return out


def dict_to_request(RequestClass: Type, data: Dict[str, Any]) -> Any:
    req = RequestClass()
    if not data:
        return req
    fields = _get_fields_and_types(RequestClass)
    for key, value in data.items():
        if key not in fields or value is None:
            continue
        field_type = fields[key]
        try:
            setattr(req, key, _value_to_ros_field(value, field_type))
        except TypeError as e:
            raise TypeError(f"The '{key}' field must be of type '{field_type}'") from e
        except ValueError as e:
            raise TypeError(f"The '{key}' field must be of type '{field_type}'") from e
    return req


def _value_to_ros_field(value: Any, field_type: str) -> Any:
    if field_type in ("float32", "float64", "float", "double"):
        if value is None:
            return 0.0
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value.strip())
            except ValueError:
                raise TypeError("expected float")
        raise TypeError("expected float")
    if field_type in ("int8", "uint8", "int16", "uint16", "int32", "uint32",
                       "int64", "uint64", "byte", "char"):
        return int(value) if value is not None else 0
    if field_type == "bool":
        return bool(value) if value is not None else False
    if field_type == "string":
        return str(value) if value is not None else ""
    if "sequence" in field_type:
        return value if isinstance(value, list) else value
    if isinstance(value, dict) and "/" in field_type:
        try:
            msg_class = _load_message_class(field_type)
            if msg_class is None:
                return value
            obj = msg_class()
            for k, v in value.items():
                if hasattr(obj, k):
                    setattr(obj, k, v)
            return obj
        except Exception:
            return value
    return value


def _load_message_class(type_str: str):
    parts = type_str.split("/")
    if len(parts) != 3:
        return None
    module_name = f"{parts[0]}.{parts[1]}"
    try:
        mod = importlib.import_module(module_name)
        return getattr(mod, parts[2], None)
    except Exception:
        return None

_INT_TYPES = {
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "byte", "char", "octet",
}
_FLOAT_TYPES = {"float32", "float64", "float", "double"}
_BOOL_TYPES = {"bool", "boolean"}
_STRING_TYPES = {"string", "wstring"}


def message_schema(msg_or_class):
    return _get_fields_and_types(msg_or_class)


def message_template(msg_or_class):
    fields = _get_fields_and_types(msg_or_class)
    out = {}
    for name, type_str in fields.items():
        out[name] = _template_value_for_type(type_str)
    return out


def _template_value_for_type(type_str: str):
    if not type_str:
        return None

    # sequence<T> or sequence<T, N>
    if type_str.startswith("sequence<") and type_str.endswith(">"):
        return []

    # T[N]
    m = re.match(r"(.+)\[(\d+)\]$", type_str)
    if m:
        base = m.group(1).strip()
        size = int(m.group(2))
        if size <= 16:
            return [_template_value_for_type(base) for _ in range(size)]
        return []

    # bounded string like string<=10
    if type_str.startswith("string") or type_str.startswith("wstring"):
        return ""

    if type_str in _BOOL_TYPES:
        return False
    if type_str in _FLOAT_TYPES:
        return 0.0
    if type_str in _INT_TYPES:
        return 0

    # nested message type: pkg/msg/Type
    if "/" in type_str:
        cls = _load_message_class(type_str)
        if cls is None:
            return {}
        return message_template(cls)

    return None


def get_srv_definition_text(service_type: str):
    parts = service_type.split("/")
    if len(parts) != 3:
        return None
    pkg, _, srv_name = parts

    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(pkg)
    except Exception:
        return None

    candidate = os.path.join(share_dir, "srv", f"{srv_name}.srv")
    if not os.path.exists(candidate):
        return None

    try:
        with open(candidate, "r", encoding="utf-8") as f:
            return f.read()
    except Exception:
        return None