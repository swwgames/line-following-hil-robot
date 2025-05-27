import socket

class Communicator:
    def __init__(self, esp32_ip='192.168.4.1', esp32_port=8888, header=b'S'):
        """Initializes communication with an ESP32 device.

        Args:
            esp32_ip (str): IP address of the ESP32 device. Defaults to '192.168.4.1'.
            esp32_port (int): Port used for communication. Defaults to 8888.
            header (bytes): Communication header. Defaults to b'S'.

        Raises:
            Exception: If connection to ESP32 fails (if you call connect_to_esp32).
        """
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.header = header
        self.client_socket = None
        self.connect_to_esp32()

    def connect_to_esp32(self):
        """Connect to the ESP32 socket."""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"Attempting to connect to ESP32 at {self.esp32_ip}:{self.esp32_port}...")
            self.client_socket.connect((self.esp32_ip, self.esp32_port))
            print("Connected to ESP32.")
            self.client_socket.settimeout(0.1)
        except socket.error as e:
            print(f"Socket connection failed: {e}")
            print("Is the ESP32 running and did you connect your PC to its Wi-Fi AP?")
            return

    def read_bytes_from_socket(self, num_bytes, timeout_sec: float = 1.0):
        """Read amount of bytes from a socket

        Args:
            num_bytes (int): number of bytes to read
            timeout_sec (float): timeout in milliseconds

        Returns:
            data (bytes | None): bytes read, or None if no bytes
        """

        data = b''
        self.client_socket.settimeout(timeout_sec)
        try:
            while len(data) < num_bytes:
                chunk = self.client_socket.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
        except socket.timeout:
            print(f"Socket recv timed out waiting for {num_bytes} bytes.")
            return None
        except socket.error as e:
            print(f"read_bytes_from_socket error: {e}")
            return None
        finally:
            self.client_socket.settimeout(None)
        return data

    def receive_packet_from_socket(self, timeout_sec: float = 1.0) -> tuple[bytes, bytes] | None:
        """Read packet from socket

        Args:
            timeout_sec (float): timeout in milliseconds. Defaults to 1000.

        Returns:
            packet (tuple[bytes, bytes | None] | None): packet read, or None if no packet or invalid packet
        """

        header_byte = self.read_bytes_from_socket(1, timeout_sec)
        if header_byte != self.header and header_byte is not None:
            print(f"Bad header received: {header_byte}")
            return None

        packet_type_byte = self.read_bytes_from_socket(1, timeout_sec)
        if not packet_type_byte:
            return None

        size_byte = self.read_bytes_from_socket(1, timeout_sec)
        if not size_byte:
            return None
        payload_size = size_byte[0]

        payload_data = self.read_bytes_from_socket(payload_size, timeout_sec)
        if not payload_data or len(payload_data) != payload_size:
            return None

        return packet_type_byte, payload_data

    def send_packet_to_socket(self, packet_type: bytes, payload: bytes):
        """Send packet to socket

        Args:
            packet_type (bytes): packet type byte
            payload (bytes): payload bytes
        """
        if not self.client_socket:
            print("No socket to send to.")
            return
        try:
            payload_size = len(payload)
            packet = self.header + packet_type + bytes([payload_size]) + payload
            self.client_socket.sendall(packet)
        except socket.error as e:
            print(f"Socket send error: {e}")
            raise