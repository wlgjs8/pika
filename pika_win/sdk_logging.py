"""Logging controls for third-party Pika SDK modules."""
import logging


def quiet_pika_sdk_info():
    """Hide noisy SDK info logs while keeping warnings and errors visible."""
    for name in ("pika.sense", "pika.serial_comm", "pika.gripper"):
        logging.getLogger(name).setLevel(logging.WARNING)
