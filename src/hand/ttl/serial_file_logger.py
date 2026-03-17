"""serial_file_logger.py — File logger for real-hardware serial traffic.

Logs TX/RX packets in the same format HandSimulator uses, so
sim_log_analyze.py works identically on real-hardware captures.

Log location: logs/real/real_YYYYMMDD_HHMMSS.log

Usage::

    from hand.serial_file_logger import SerialFileLogger
    logger = SerialFileLogger()
    hand._protocol._logger = logger
    ...
    logger.close()   # on exit
"""

import datetime
import os


class SerialFileLogger:
    """Write complete Feetech packets to a timestamped log file.

    The log format is identical to HandSimulator:

        HH:MM:SS.mmm  TX       FF FF FE 0B 82 38 0F 00 01 02 03 04 05 06 4B
        HH:MM:SS.mmm  RX       FF FF 00 11 00 [data bytes] CK

    Attach to a ServoProtocol instance via its ``_logger`` attribute::

        hand._protocol._logger = SerialFileLogger()
    """

    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(log_dir, f"real_{ts}.log")
        self._file = open(self._path, "w", buffering=1)  # line-buffered

    @property
    def path(self) -> str:
        return self._path

    def _write(self, direction: str, data: bytes):
        if not self._file:
            return
        ts   = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_ = " ".join(f"{b:02X}" for b in data)
        self._file.write(f"{ts}  {direction:<7}  {hex_}\n")

    def log_tx(self, data: bytes):
        """Log a complete outgoing packet."""
        self._write("TX", data)

    def log_rx(self, data: bytes):
        """Log a complete incoming response packet."""
        self._write("RX", data)

    def close(self):
        """Flush and close the log file."""
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None
