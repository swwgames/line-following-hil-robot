import network
import socket
import time

class Communicator:
    def __init__(self):
        self.wifi_ssid = "ESP32_LineFollower"
        self.wifi_password = "password123"
        self.server_ip = '192.168.4.1'
        self.server_port = 8888

        self.client_socket = None
        self.client_addr = None
        self.setup_wifi()
        self.setup_server()
        
        self.header = b'S'

    def setup_wifi(self):
        """Configure AP for the simulation to connect to."""
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=self.wifi_ssid, password=self.wifi_password)

        while not ap.active():
            time.sleep_ms(100)

        print('ESP32 Access Point configured.')
        print(f'SSID: {self.wifi_ssid}')
        print(f'IP: {ap.ifconfig()[0]}')

    def setup_server(self):
        """Open a socket for the simulation to connect to."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.server_ip, self.server_port))
        s.listen(1)

        print("Waiting for Webots to connect...")
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        self.client_socket = conn
        self.client_addr = addr
        self.client_socket.settimeout(1.0)

    def read_bytes_from_socket(self, num_bytes: int, timeout_ms: int) -> bytes | None:
        """Read amount of bytes from a socket, return None if no bytes."""
        data = b''
        start_time = time.ticks_ms()
        self.client_socket.settimeout(timeout_ms / 1000)
        try:
            while len(data) < num_bytes:
                if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                    return None
                chunk = self.client_socket.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
        except OSError as e:
            #print(f"read_bytes_from_socket error: {e}")
            return None
        finally:
            self.client_socket.settimeout(1.0)
        return data

    def read_packet_from_socket(self, timeout_ms=1000) -> tuple[bytes, bytes | None] | None:
        """Read packet from socket, return None if no packet or invalid packet"""
        header_byte = self.read_bytes_from_socket(1, timeout_ms)
        if header_byte != self.header:
            if header_byte is not None:
                print(f"Bad header: {header_byte}")
            return None

        packet_type_byte = self.read_bytes_from_socket(1, timeout_ms)
        if not packet_type_byte:
            return None

        size_byte = self.read_bytes_from_socket(1, timeout_ms)
        if not size_byte:
            return None
        payload_size = size_byte[0]

        payload_data = self.read_bytes_from_socket(payload_size, timeout_ms)
        if not payload_data or len(payload_data) != payload_size:
            return None

        return packet_type_byte, payload_data

    def send_packet_to_socket(self, packet_type: bytes, payload: bytes):
        """Send packet to socket."""
        if not self.client_socket:
            print("No socket to send to.")
            return
        try:
            size = len(payload)
            packet = self.header + packet_type + bytes([size]) + payload
            self.client_socket.sendall(packet)
        except OSError as e:
            print(f"Socket send error: {e}")
            raise
