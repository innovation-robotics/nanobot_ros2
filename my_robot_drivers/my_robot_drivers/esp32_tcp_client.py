#!/usr/bin/env python3
import socket
import threading
from typing import Optional


class Esp32TcpClient:
    """Persistent TCP client supporting line writes, line reads, auto-reconnect, and thread safety."""

    def __init__(
        self,
        logger,
        ip: str,
        port: int,
        name: str = "TCP",
        connect_timeout: float = 1.5,
        io_timeout: float = 0.2,
    ):
        self._logger = logger
        self.ip = ip
        self.port = int(port)
        self.name = name
        self.connect_timeout = float(connect_timeout)
        self.io_timeout = float(io_timeout)

        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._rx_buf = bytearray()

    def _create_socket(self, log_reconnect: bool = False) -> Optional[socket.socket]:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(self.connect_timeout)
            s.connect((self.ip, self.port))
            s.settimeout(self.io_timeout)
            if log_reconnect:
                self._logger.info(f"[{self.name}] Reconnected to ESP32 at {self.ip}:{self.port}")
            else:
                self._logger.info(f"[{self.name}] Connected to ESP32 at {self.ip}:{self.port}")
            return s
        except Exception as e:
            self._logger.warn(
                f"[{self.name}] Connect failed ({self.ip}:{self.port}): {e}",
                throttle_duration_sec=2.0,
            )
            return None

    def _close_locked(self, target_sock: Optional[socket.socket] = None):
        sock = target_sock if target_sock is not None else self._sock
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        if target_sock is None or self._sock is target_sock:
            self._sock = None
            self._rx_buf.clear()

    def connect(self) -> bool:
        with self._lock:
            self._close_locked()
            self._sock = self._create_socket(log_reconnect=False)
            return self._sock is not None

    def ensure_connected(self) -> bool:
        with self._lock:
            if self._sock is not None:
                return True
            self._sock = self._create_socket(log_reconnect=False)
            return self._sock is not None

    def send_line(self, line: str) -> bool:
        payload = line if line.endswith("\n") else f"{line}\n"
        encoded = payload.encode("utf-8")

        with self._lock:
            if self._sock is None:
                self._sock = self._create_socket(log_reconnect=True)
            if self._sock is None:
                return False

            try:
                self._sock.sendall(encoded)
                return True
            except (socket.error, socket.timeout, OSError) as e:
                self._logger.warn(
                    f"[{self.name}] Send error: {e}. Reconnecting...",
                    throttle_duration_sec=2.0,
                )
                self._close_locked()
                self._sock = self._create_socket(log_reconnect=True)
                if self._sock is not None:
                    try:
                        self._sock.sendall(encoded)
                        return True
                    except Exception:
                        self._close_locked()
                return False

    def recv_line(self) -> Optional[str]:
        """Read a single newline-delimited line, or return None on timeout/disconnect/error."""
        with self._lock:
            newline_idx = self._rx_buf.find(b"\n")
            if newline_idx != -1:
                raw_line = bytes(self._rx_buf[:newline_idx])
                del self._rx_buf[: newline_idx + 1]
                return raw_line.decode("utf-8", errors="ignore").strip()

            sock = self._sock
            if sock is None:
                return None

        try:
            chunk = sock.recv(256)
        except socket.timeout:
            return None
        except (socket.error, OSError) as e:
            self._logger.warn(f"[{self.name}] Read error: {e}", throttle_duration_sec=2.0)
            with self._lock:
                self._close_locked(sock)
            return None

        with self._lock:
            if not chunk:
                self._close_locked(sock)
                return None

            self._rx_buf.extend(chunk)
            newline_idx = self._rx_buf.find(b"\n")
            if newline_idx == -1:
                return None

            raw_line = bytes(self._rx_buf[:newline_idx])
            del self._rx_buf[: newline_idx + 1]
            return raw_line.decode("utf-8", errors="ignore").strip()

    def close(self):
        with self._lock:
            self._close_locked()