"""Miscellaneous helper functions for the elliptec package."""
from .errcodes import error_codes
from enum import Enum

def is_null_or_empty(msg):
    """Checks if message is empty or null."""
    if not msg.endswith(b"\r\n") or (len(msg) == 0):
        return True
    else:
        return False

def is_metric(num):
    """Checks if thread is metric or imperial."""
    if num == "0":
        thread_type = "Metric"
    elif num == "1":
        thread_type = "Imperial"
    else:
        thread_type = None

    return thread_type


def error_check(status):
    """Checks if there is an error."""
    if not status:
        print("Status is None")
    elif isinstance(status, dict):
        print("Status is a dictionary.")
    elif status[1] == "GS":
        if status[2] != "0":  # is there an error?
            err = error_codes[status[2]]
            print(f"ERROR: {err}")
        else:
            print("Status OK")
    elif status[1] == "PO":
        print("Status OK (position)")
    else:
        print("Other status:", status)


def move_check(status):
    """Checks if the move was successful."""
    if not status:
        print("Status is None")
    elif status[1] == "GS":
        error_check(status)
    elif (status[1] == "PO") or (status[1] == "MO"):
        print("Move Successful.")
    else:
        print(f"Unknown response code {status[1]}")
