import streamlit as st
from PIL import Image, ImageDraw, ImageFont, ImageEnhance
import threading
import socket
import json
import queue
from streamlit_autorefresh import st_autorefresh

MAP_IMAGE_PATH = "map.png"
ROBOT_ICON_PATH = "robot.png"
FONT_PATH = "arial.ttf"
DEFAULT_FONT_SIZE = 40
SERVER_ADDRESS = ('localhost', 7000)

NODE_POSITIONS = {
    "A1": (86, 130), "A2": (175, 130), "A3": (260, 130), "A4": (350, 130),
    "A5": (510, 130), "A6": (935, 130), "B1": (510, 250), "B2": (935, 250),
    "C1": (86, 340), "C2": (510, 340), "C3": (935, 340), "D1": (86, 425),
    "D2": (510, 425), "E1": (86, 550), "E2": (510, 550), "E3": (675, 550),
    "E4": (761, 550), "E5": (849, 550), "E6": (935, 550), "P1": (86, 60),
    "P2": (175, 60), "P3": (260, 60), "P4": (350, 60), "P5": (675, 620),
    "P6": (761, 620), "P7": (849, 620), "P8": (935, 60),
}

@st.cache_data
def load_image(image_path: str) -> Image.Image:
    """Loads an image from the specified path and converts it to RGBA.

    Args:
        image_path (str): The path to the image file.

    Returns:
        Image.Image: The loaded PIL Image object in RGBA format.
    """
    return Image.open(image_path).convert("RGBA")

@st.cache_resource # Caches the font object itself
def load_font(font_path: str, size: int) -> ImageFont.FreeTypeFont:
    """Loads a font from the specified path and size.
    Falls back to a default font if the specified font is not found.

    Args:
        font_path (str): The path to the .ttf font file.
        size (int): The desired font size.

    Returns:
        ImageFont.FreeTypeFont: The loaded PIL ImageFont object.
    """
    try:
        return ImageFont.truetype(font_path, size)
    except IOError:
        st.warning(f"Font '{font_path}' not found. Using default font. Text may not appear as expected.")
        return FreeTypeFont.load_default()


def initialize_session_state():
    """Initializes required keys in Streamlit's session state if they don't exist.
    This function is called on every rerun but only sets initial values once per session.
    """
    if "data_queue" not in st.session_state:
        st.session_state.data_queue = queue.Queue()
    if "current_node" not in st.session_state:
        st.session_state.current_node = None
    if "current_path" not in st.session_state:
        st.session_state.current_path = []
    if "server_started" not in st.session_state:
        st.session_state.server_started = False
    if "new_data_received_flag" not in st.session_state:
        st.session_state.new_data_received_flag = False



def socket_server_logic(data_q: queue.Queue):
    """Listens for incoming socket connections and puts received data into a queue.

    Args:
        data_q (queue.Queue): The queue to which received data payloads are added.
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(SERVER_ADDRESS)
        server.listen(5)
        print(f"Socket server listening on port {SERVER_ADDRESS[1]}")
        while True:
            try:
                conn, addr = server.accept()
                print(f"Connection from {addr}")
                with conn:
                    data = conn.recv(1024)
                    if data:
                        try:
                            payload = json.loads(data.decode())
                            data_q.put(payload)
                            print(f"Data received and queued: {payload}")
                        except json.JSONDecodeError:
                            print("Invalid JSON received")
                        except Exception as e:
                            print(f"Error processing received data: {e}")
                    else:
                        print(f"Connection from {addr} closed with no data.")
            except socket.error as e:
                print(f"Socket accept/receive error: {e}")
            except Exception as e:
                print(f"Critical error in server accept loop: {e}")
                break
    except OSError as e:
        print(f"Error binding socket: {e}. Port {SERVER_ADDRESS[1]} might be in use.")
    finally:
        print("Socket server shutting down.")
        server.close()

def start_server_thread(data_q: queue.Queue):
    """Starts the socket server in a separate thread if it hasn't started yet.

    Args:
        data_q (queue.Queue): The data queue to be passed to the server thread.
    """
    if not st.session_state.server_started:
        thread = threading.Thread(target=socket_server_logic, args=(data_q,), daemon=True)
        thread.start()
        st.session_state.server_started = True
        print("Socket server thread started.")


def process_data_queue(data_q: queue.Queue):
    """Processes all messages currently in the data queue, updating session state.

    Args:
        data_q (queue.Queue): The queue containing data payloads from the socket server.
    """
    st.session_state.new_data_received_flag = False
    while not data_q.empty():
        try:
            payload = data_q.get_nowait()
            node = payload.get("node")
            path = payload.get("path", []) # Default to empty list if path is not in payload

            # Specific condition to clear current_node and path (e.g., robot is lost/re-localizing)
            if node == "" and path == []: # Explicitly empty strings/lists
                st.session_state.current_node = "" # Use empty string to signify this state
                st.session_state.current_path = []
            else: # Handle normal updates
                if node is not None: # Allow node to be updated even if empty string (but not None)
                    st.session_state.current_node = node
                if path: # Only update path if it's a non-empty list
                    st.session_state.current_path = path

            st.session_state.new_data_received_flag = True
            print(f"Data processed from queue: node={st.session_state.current_node}, path={st.session_state.current_path}")
        except queue.Empty:
            break
        except Exception as e:
            print(f"Error processing item from queue: {e}")


def generate_map_visualization(
    base_map_img: Image.Image,
    robot_icon_img: Image.Image,
    font: ImageFont.FreeTypeFont,
    current_node: str | None,
    current_path: list,
    node_positions: dict
) -> Image.Image:
    """Generates the map visualization with robot position, path, and status text.

    Args:
        base_map_img (Image.Image): The base map image.
        robot_icon_img (Image.Image): The image for the robot icon.
        font (ImageFont.FreeTypeFont): The font to use for text rendering.
        current_node (str | None): The current node the robot is at.
                                   An empty string "" can signify a "localizing" state without a known node.
        current_path (list): A list of node names representing the robot's path.
        node_positions (dict): A dictionary mapping node names to (x, y) coordinates.

    Returns:
        Image.Image: The final composed image with all visualizations.
    """
    img_to_draw_on = base_map_img.copy()
    draw = ImageDraw.Draw(img_to_draw_on)

    # Determine if we are in the "Localizing position" state
    # This state is active if current_node is an empty string AND current_path is empty,
    # OR if current_node is None (initial state before any data).
    is_localizing = (current_node == "" and not current_path) or current_node is None

    if is_localizing:
        enhancer = ImageEnhance.Brightness(img_to_draw_on)
        img_to_draw_on = enhancer.enhance(0.3) # Darken the image
        draw = ImageDraw.Draw(img_to_draw_on) # Re-create draw object for the new darkened image

        text = "Localizing position"
        text_bbox = draw.textbbox((0, 0), text, font=font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        text_x = (img_to_draw_on.width - text_width) // 2
        text_y = (img_to_draw_on.height - text_height) // 2
        draw.text((text_x, text_y), text, font=font, fill="white")
    else:
        # Draw path
        if len(current_path) > 1:
            for i in range(len(current_path) - 1):
                start_node_name = current_path[i]
                end_node_name = current_path[i+1]
                if start_node_name in node_positions and end_node_name in node_positions:
                    start_pos = node_positions[start_node_name]
                    end_pos = node_positions[end_node_name]
                    draw.line([start_pos, end_pos], fill="red", width=5)
                else:
                    print(f"Warning: Node in path not in node_positions: {start_node_name} or {end_node_name}")

        # Draw robot icon at current_node
        if current_node and current_node in node_positions:
            robot_resized = robot_icon_img.resize((40, 40))
            pos = node_positions[current_node]
            top_left = (pos[0] - robot_resized.width // 2, pos[1] - robot_resized.height // 2)
            img_to_draw_on.paste(robot_resized, top_left, robot_resized if robot_resized.mode == 'RGBA' else None)
        elif current_node: # Node name exists but not in positions
             print(f"Warning: Current node '{current_node}' not found in node_positions dictionary.")

    return img_to_draw_on


def main():
    """Main function to run the Streamlit application."""
    st.set_page_config(page_title="Robot Live Map", layout="wide")
    st.title("Robot Live View")

    # Initialize session state variables (safe to call on each rerun)
    initialize_session_state()

    # Load resources (cached, so only loads once effectively)
    map_img = load_image(MAP_IMAGE_PATH)
    robot_icon = load_image(ROBOT_ICON_PATH)
    ui_font = load_font(FONT_PATH, DEFAULT_FONT_SIZE)

    # Start the socket server thread (only if not already started)
    start_server_thread(st.session_state.data_queue)

    # Setup auto-refresh for the page
    st_autorefresh(interval=1000, key="maprefresh")

    # Process any new data from the socket server
    process_data_queue(st.session_state.data_queue)

    # Get current state for display
    node_to_display = st.session_state.current_node
    path_to_display = st.session_state.current_path

    # Generate the map image
    display_img = generate_map_visualization(
        map_img, robot_icon, ui_font,
        node_to_display, path_to_display, NODE_POSITIONS
    )

    # Display the map and status
    st.image(display_img, caption="Robot Live View", width=800)

    is_currently_localizing = (node_to_display == "" and not path_to_display) or node_to_display is None
    node_display_text = 'Localizing...' if is_currently_localizing else (node_to_display if node_to_display else 'N/A')

    st.write(f"Current Node: `{node_display_text}`")
    st.write(f"Path: `{path_to_display if path_to_display else 'N/A'}`")

    if st.session_state.new_data_received_flag:
        st.success("Map updated with new data!")
        st.session_state.new_data_received_flag = False # Reset flag

if __name__ == "__main__":
    main()