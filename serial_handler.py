import threading
import time
import queue
from typing import List

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None


class SerialHandler:
    def __init__(self):
        self.port = None
        self.baud = 115200
        self._ser = None
        self._queue = queue.Queue()
        self._read_buffer = bytearray()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def list_ports(self) -> List[str]:
        if serial is None:
            return []
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports

    def connect(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = int(baud)
        self._read_buffer.clear()

    def disconnect(self):
        # clearing port will cause worker to close underlying serial
        self.port = None
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._read_buffer.clear()

    def get_status(self):
        return {
            'connected': bool(self._ser and getattr(self._ser, 'is_open', False)),
            'port': self.port,
            'baud': self.baud,
            'queue_size': self._queue.qsize(),
        }

    def send_detection(self, track_id, figura, color, x_cm, y_cm):
        # Format CSV: track_id,figura,color,x_cm,y_cm\n
        try:
            x = '' if x_cm is None else f"{float(x_cm):.2f}"
        except Exception:
            x = ''
        try:
            y = '' if y_cm is None else f"{float(y_cm):.2f}"
        except Exception:
            y = ''

        linea = f"{track_id},{figura},{color},{x},{y}\n"
        print(f"[TO ARDUINO] {linea.strip()}", flush=True)
        self._queue.put(linea.encode('utf-8'))

    def _worker(self):
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

    def _leer_mensajes_serial(self):
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

    def stop(self):
        self._stop_event.set()
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
