#!/usr/bin/env python3
"""
ROS 2 node: HTTP API to list services and call them.
Default port 9090 (parameter 'port').
Access-Control-Allow-Origin: * on responses.
"""

import asyncio
import json
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import tornado.ioloop
import tornado.web
import tornado.escape

from ros_services_bridge.srv_serialization import (
    get_srv_class,
    response_to_dict,
    dict_to_request,
    message_schema,
    message_template,
    get_srv_definition_text,
)

class ServicesBridgeNode(Node):
    def __init__(self):
        super().__init__("ros_services_bridge_node")
        self.port = self.declare_parameter("port", 9090).value
        self._lock = threading.Lock()
        self._app = None
        self._ioloop = None

    def get_services_list(self):
        names_and_types = self.get_service_names_and_types()
        return [
            {"name": name, "type": types[0] if types else ""}
            for name, types in names_and_types
        ]

    def _resolve_service_name(self, service_name: str):
        names_and_types = self.get_service_names_and_types()
        for name, types in names_and_types:
            if name == service_name and types:
                return name, types[0]
        alt = ("/" + service_name) if not service_name.startswith("/") else service_name[1:]
        for name, types in names_and_types:
            if name == alt and types:
                return name, types[0]
        return None, None

    def call_service(self, service_name: str, request_dict: dict):
        resolved_name, service_type = self._resolve_service_name(service_name)
        if not service_type:
            return False, f"Service '{service_name}' not found"

        SrvClass, RequestClass, _ = get_srv_class(service_type)
        if SrvClass is None:
            return False, f"Could not load service type '{service_type}'"

        try:
            request_msg = dict_to_request(RequestClass, request_dict or {})
        except Exception as e:
            return False, f"Invalid request: {e!s}"

        try:
            client = self.create_client(SrvClass, resolved_name)
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=3.0)
            if not client.service_is_ready():
                return False, "Service not available (timeout)"

            future = client.call_async(request_msg)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=10.0,
                executor=getattr(self, "_executor", None),
            )
            if not future.done():
                return False, "Service call timeout"
            result = future.result()
            if result is None:
                return False, "Service call failed (no result)"
            return True, response_to_dict(result)
        except Exception as e:
            return False, str(e)


class _CorsHandler(tornado.web.RequestHandler):
    """Base handler with permissive CORS headers."""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.set_header("Content-Type", "application/json")

    def options(self, *_args):
        self.set_status(204)
        self.finish()


def _make_handlers(node: ServicesBridgeNode):
    class ListServicesHandler(_CorsHandler):
        def get(self):
            try:
                self.write(json.dumps({"services": node.get_services_list()}))
            except Exception as e:
                self.set_status(500)
                self.write(json.dumps({"error": str(e)}))

    class CallServiceHandler(_CorsHandler):
        def post(self, service_name):
            service_name = tornado.escape.url_unescape(service_name)
            try:
                body = self.request.body.decode("utf-8") if self.request.body else "{}"
                request_dict = json.loads(body) if body.strip() else {}
            except json.JSONDecodeError as e:
                self.set_status(400)
                self.write(json.dumps({"success": False, "error": f"Invalid JSON: {e}"}))
                return
            try:
                success, result = node.call_service(service_name, request_dict)
                if success:
                    self.write(json.dumps({"success": True, "response": result}))
                else:
                    self.set_status(400)
                    self.write(json.dumps({"success": False, "error": result}))
            except Exception as e:
                self.set_status(500)
                self.write(json.dumps({"success": False, "error": str(e)}))

    class ServiceSchemaHandler(_CorsHandler):
        def get(self, service_name):
            service_name = tornado.escape.url_unescape(service_name)

            resolved_name, service_type = node._resolve_service_name(service_name)
            if not service_type:
                self.set_status(404)
                self.write(json.dumps({"error": "Not Found"}))
                return

            SrvClass, RequestClass, ResponseClass = get_srv_class(service_type)
            if SrvClass is None:
                self.set_status(500)
                self.write(json.dumps({"error": f"Could not load service type '{service_type}'"}))
                return

            body = {
                "service": resolved_name,
                "type": service_type,
                "request": {
                    "fields": message_schema(RequestClass),
                    "template": message_template(RequestClass),
                },
                "response": {
                    "fields": message_schema(ResponseClass),
                    "template": message_template(ResponseClass),
                },
            }

            definition = get_srv_definition_text(service_type)
            if definition is not None:
                body["definition"] = definition

            self.write(json.dumps(body))

    class HealthHandler(_CorsHandler):
        def get(self):
            self.write("ok")

    return [
        (r"/api/services", ListServicesHandler),
        (r"/api/services/(.+)/schema", ServiceSchemaHandler),
        (r"/api/services/(.+)/call", CallServiceHandler),
        (r"/", HealthHandler),
    ]


def _run_tornado(node: ServicesBridgeNode):
    asyncio.set_event_loop(asyncio.new_event_loop())
    app = tornado.web.Application(_make_handlers(node))
    app.listen(node.port)
    node._app = app
    node._ioloop = tornado.ioloop.IOLoop.current()
    node._ioloop.start()


def main(args=None):
    rclpy.init(args=args)
    node = ServicesBridgeNode()
    node.get_logger().info("ROS Services Bridge starting on port %d" % node.port)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node._executor = executor
    threading.Thread(target=_run_tornado, args=(node,), daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
