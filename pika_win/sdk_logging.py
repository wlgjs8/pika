"""Logging controls for third-party Pika SDK modules."""
import logging
import time


class _PikaSdkLogFilter(logging.Filter):
    """Keep third-party SDK logs readable in this app."""

    _TRANSLATIONS = (
        ("成功连接到串口设备: ", "[sdk] serial connected: "),
        ("连接串口设备失败: ", "[sdk] serial connect failed: "),
        ("启动串口读取线程", "[sdk] serial read thread started"),
        ("串口读取线程已停止", "[sdk] serial read thread stopped"),
        ("读取线程已停止", "[sdk] read thread stopped"),
        ("已断开串口设备连接: ", "[sdk] serial disconnected: "),
        ("成功连接到Pika Sense设备: ", "[sdk] Pika Sense connected: "),
        ("已断开Pika Sense设备连接: ", "[sdk] Pika Sense disconnected: "),
        ("成功连接到Pika Gripper设备: ", "[sdk] Pika Gripper connected: "),
        ("已断开Pika Gripper设备连接: ", "[sdk] Pika Gripper disconnected: "),
        ("设备已经连接", "[sdk] device already connected"),
        ("串口未连接，无法发送数据", "[sdk] serial is not connected; cannot send"),
        ("串口未连接，无法读取数据", "[sdk] serial is not connected; cannot read"),
        ("发送数据失败: ", "[sdk] serial send failed: "),
        ("读取数据失败: ", "[sdk] serial read failed: "),
        ("读取线程异常: ", "[sdk] serial read thread error: "),
        ("通信Json异常: ", "[sdk] serial JSON error: "),
    )

    def __init__(self, show_parse_errors=False, parse_period_sec=10.0):
        super().__init__()
        self.show_parse_errors = bool(show_parse_errors)
        self.parse_period_sec = float(parse_period_sec)
        self._last_parse_log = 0.0

    def filter(self, record):
        if not record.name.startswith("pika."):
            return True

        msg = record.getMessage()
        if msg.startswith("JSON解析错误: "):
            if not self.show_parse_errors:
                return False
            now = time.monotonic()
            if now - self._last_parse_log < self.parse_period_sec:
                return False
            self._last_parse_log = now
            detail = msg.split(": ", 1)[1] if ": " in msg else msg
            record.msg = "[sdk] malformed serial JSON frame ignored: %s" % detail
            record.args = ()
            record.levelno = logging.WARNING
            record.levelname = "WARNING"
            return True

        for src, dst in self._TRANSLATIONS:
            if msg.startswith(src):
                record.msg = dst + msg[len(src):]
                record.args = ()
                return True

        return True


def quiet_pika_sdk_info(show_parse_errors=False):
    """Hide noisy SDK info logs while keeping warnings and errors visible."""
    sdk_filter = _PikaSdkLogFilter(show_parse_errors=show_parse_errors)
    for name in ("pika.sense", "pika.serial_comm", "pika.gripper"):
        logger = logging.getLogger(name)
        logger.setLevel(logging.WARNING)
        if not any(isinstance(f, _PikaSdkLogFilter) for f in logger.filters):
            logger.addFilter(sdk_filter)
