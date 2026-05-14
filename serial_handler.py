import threading
import time
import queue
from typing import Any, Callable, Iterable, List

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None


class SerialHandler:
    def __init__(self):
        self.port = None
        self.baud = 115200
        self._ser: Any | None = None
        self._queue: queue.Queue[bytes] = queue.Queue()
        self._read_buffer = bytearray()
        self._ready = False
        self._line_callback: Callable[[str], None] | None = None
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def set_line_callback(self, callback: Callable[[str], None] | None):
        with self._state_lock:
            self._line_callback = callback

    def list_ports(self) -> List[str]:
        if serial is None:
            return []
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports

    def connect(self, port: str, baud: int = 115200) -> None:
        with self._state_lock:
            self.port = port
            self.baud = int(baud)
            self._ready = False
            self._read_buffer.clear()
            self._vaciar_cola()

    def set_ready(self, ready: bool) -> None:
        with self._state_lock:
            self._ready = bool(ready)

    def is_ready(self) -> bool:
        with self._state_lock:
            return bool(self._ready and self._ser and getattr(self._ser, 'is_open', False))

    def disconnect(self) -> None:
        # clearing port will cause worker to close underlying serial
        with self._state_lock:
            self.port = None
            self._ready = False
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._read_buffer.clear()
        self._vaciar_cola()

    def get_status(self) -> dict[str, object]:
        return {
            'connected': bool(self._ser and getattr(self._ser, 'is_open', False)),
            'ready': self.is_ready(),
            'port': self.port,
            'baud': self.baud,
            'queue_size': self._queue.qsize(),
        }

    def enqueue_line(self, line: str) -> None:
        if not line.endswith('\n'):
            line = f'{line}\n'
        self._queue.put(line.encode('utf-8'))

    def enqueue_lines(self, lines: Iterable[str]) -> None:
        for line in lines:
            self.enqueue_line(line)

    def _vaciar_cola(self) -> None:
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            return

    def clear_output_queue(self) -> None:
        self._vaciar_cola()

    def send_detection(self, track_id: int, figura: str, color: str, x_cm: float | None, y_cm: float | None, z_cm: float | None) -> bool:
        if not self.is_ready():
            return False

        # Format CSV: OBJ,x_cm,y_cm,z_cm,container_index\n
        try:
            x = '' if x_cm is None else f"{float(x_cm):.2f}"
        except Exception:
            x = ''
        try:
            y = '' if y_cm is None else f"{float(y_cm):.2f}"
        except Exception:
            y = ''
        try:
            z = '' if z_cm is None else f"{float(z_cm):.2f}"
        except Exception:
            z = ''

        linea = f"OBJ,{x},{y},{z},{track_id}"
        self._queue.put(f"{linea}\n".encode('utf-8'))
        return True

    def _worker(self) -> None:
        while not self._stop_event.is_set():
            if serial is None:
                time.sleep(1.0)
                continue

            if self.port and (self._ser is None or not getattr(self._ser, 'is_open', False)):
                try:
                    self._ser = serial.Serial(self.port, self.baud, timeout=1)
                except Exception:
                    # failed to open, retry after short sleep
                    try:
                        if self._ser is not None:
                            self._ser.close()
                    except Exception:
                        pass
                    self._ser = None
                    time.sleep(1.0)
                    continue

            # if connected, drain queue
            if self._ser is not None and getattr(self._ser, 'is_open', False):
                self._leer_mensajes_serial()

                try:
                    data = self._queue.get(timeout=0.5)
                    print(f"[TO ARDUINO] {data}", flush=True)
                except queue.Empty:
                    continue
                try:
                    self._ser.write(data)
                    self._ser.flush()
                except Exception:
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser = None
                    # push back to queue to retry later
                    try:
                        self._queue.put_nowait(data)
                    except queue.Full:
                        pass
            else:
                # not connected, sleep briefly
                time.sleep(0.5)

    def _leer_mensajes_serial(self) -> None:
        if self._ser is None or not getattr(self._ser, 'is_open', False):
            return

        try:
            bytes_pendientes = self._ser.in_waiting
        except Exception:
            return

        if bytes_pendientes <= 0:
            return

        try:
            data = self._ser.read(bytes_pendientes)
        except Exception:
            return

        if not data:
            return

        self._read_buffer.extend(data)

        while True:
            try:
                indice_nueva_linea = self._read_buffer.index(10)  # \n
            except ValueError:
                break

            linea = self._read_buffer[:indice_nueva_linea + 1]
            del self._read_buffer[:indice_nueva_linea + 1]

            texto = linea.decode('utf-8', errors='replace').rstrip('\r\n')
            if texto:
                print(f"[ARDUINO] {texto}", flush=True)
                callback = None
                with self._state_lock:
                    callback = self._line_callback
                if callback is not None:
                    try:
                        callback(texto)
                    except Exception:
                        pass

    def stop(self) -> None:
        self._stop_event.set()
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
