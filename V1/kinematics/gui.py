import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time

# Configure a clean, minimal aesthetic 
ctk.set_appearance_mode("System")  # Adapts to macOS/Windows dark or light mode
ctk.set_default_color_theme("blue")

class SerialTerminalApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("Serial Monitor")
        self.geometry("900x600")
        self.minsize(700, 500)
        
        # State variables
        self.serial_port = None
        self.is_connected = False
        self.read_thread = None
        
        # Layout configuration
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        self._build_sidebar()
        self._build_main_area()
        self.refresh_ports()

    def _build_sidebar(self):
        """Creates the left sidebar for connection controls."""
        self.sidebar_frame = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(5, weight=1)
        
        self.logo_label = ctk.CTkLabel(self.sidebar_frame, text="Connection", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        
        # COM Port Selection
        self.port_label = ctk.CTkLabel(self.sidebar_frame, text="COM Port:")
        self.port_label.grid(row=1, column=0, padx=20, pady=(10, 0), sticky="w")
        
        self.port_combobox = ctk.CTkComboBox(self.sidebar_frame, values=["Loading..."])
        self.port_combobox.grid(row=2, column=0, padx=20, pady=(5, 10), sticky="ew")
        
        self.refresh_btn = ctk.CTkButton(self.sidebar_frame, text="Refresh Ports", command=self.refresh_ports, 
                                         fg_color="transparent", border_width=1, text_color=("gray10", "#DCE4EE"))
        self.refresh_btn.grid(row=3, column=0, padx=20, pady=(0, 10), sticky="ew")
        
        # Baud Rate Selection
        self.baud_label = ctk.CTkLabel(self.sidebar_frame, text="Baud Rate:")
        self.baud_label.grid(row=4, column=0, padx=20, pady=(10, 0), sticky="w")
        
        self.baud_combobox = ctk.CTkComboBox(self.sidebar_frame, 
                                             values=["9600", "19200", "38400", "57600", "115200", "250000"])
        self.baud_combobox.set("115200")
        self.baud_combobox.grid(row=5, column=0, padx=20, pady=(5, 10), sticky="ewn")
        
        # Connect/Disconnect Button
        self.connect_btn = ctk.CTkButton(self.sidebar_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=6, column=0, padx=20, pady=(10, 20), sticky="ew")

    def _build_main_area(self):
        """Creates the main terminal output and input area."""
        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)
        
        # Read-only text box for incoming data
        self.textbox = ctk.CTkTextbox(self.main_frame, font=ctk.CTkFont(family="Consolas", size=13))
        self.textbox.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 20))
        self.textbox.configure(state="disabled")
        
        # Input entry for sending data
        self.input_entry = ctk.CTkEntry(self.main_frame, placeholder_text="Type command here and press Enter...")
        self.input_entry.grid(row=1, column=0, sticky="ew", padx=(0, 10))
        self.input_entry.bind("<Return>", lambda event: self.send_data())
        
        self.send_btn = ctk.CTkButton(self.main_frame, text="Send", width=100, command=self.send_data)
        self.send_btn.grid(row=1, column=1, sticky="e")

    def get_system_ports(self):
        """Fetches available serial ports across Windows, macOS, and Linux."""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["No Ports Found"]

    def refresh_ports(self):
        """Updates the dropdown with currently available ports."""
        ports = self.get_system_ports()
        self.port_combobox.configure(values=ports)
        if ports and ports[0] != "No Ports Found":
            self.port_combobox.set(ports[0])
        else:
            self.port_combobox.set("No Ports Found")

    def toggle_connection(self):
        """Handles connecting and disconnecting from the selected serial port."""
        if not self.is_connected:
            port = self.port_combobox.get()
            baud = self.baud_combobox.get()
            
            if port == "No Ports Found" or not port:
                self.log_to_terminal("Error: No valid port selected.\n")
                return
                
            try:
                self.serial_port = serial.Serial(port, int(baud), timeout=1)
                self.is_connected = True
                
                # Update UI for connected state
                self.connect_btn.configure(text="Disconnect", fg_color="#C93434", hover_color="#992828")
                self.port_combobox.configure(state="disabled")
                self.baud_combobox.configure(state="disabled")
                
                self.log_to_terminal(f"--- Connected to {port} at {baud} baud ---\n")
                
                # Start a daemon thread to read incoming data without freezing the GUI
                self.read_thread = threading.Thread(target=self.read_from_port, daemon=True)
                self.read_thread.start()
                
            except Exception as e:
                self.log_to_terminal(f"Failed to connect: {e}\n")
        else:
            self.is_connected = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            # Restore UI for disconnected state
            self.connect_btn.configure(text="Connect", fg_color=["#3B8ED0", "#1F6AA5"], hover_color=["#36719F", "#144870"])
            self.port_combobox.configure(state="normal")
            self.baud_combobox.configure(state="normal")
            self.log_to_terminal("--- Disconnected ---\n")

    def read_from_port(self):
        """Continuously reads data from the serial port while connected."""
        while self.is_connected and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='replace')
                    if data:
                        self.log_to_terminal(data)
            except Exception as e:
                if self.is_connected:
                    self.log_to_terminal(f"\nConnection lost: {e}\n")
                    self.after(0, self.toggle_connection)  # Safely update GUI from thread
                break
            time.sleep(0.01)

    def send_data(self):
        """Writes data from the entry field to the serial port."""
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            data = self.input_entry.get()
            if data:
                try:
                    # Appending \r\n as it is standard for most serial parsers
                    self.serial_port.write((data + '\r\n').encode('utf-8'))
                    self.log_to_terminal(f"> {data}\n")
                    self.input_entry.delete(0, 'end')
                except Exception as e:
                    self.log_to_terminal(f"Failed to send: {e}\n")
        else:
            self.log_to_terminal("Error: Not connected to a port.\n")

    def log_to_terminal(self, message):
        """Safely inserts text into the read-only textbox and auto-scrolls."""
        self.textbox.configure(state="normal")
        self.textbox.insert("end", message)
        self.textbox.see("end")
        self.textbox.configure(state="disabled")

if __name__ == "__main__":
    app = SerialTerminalApp()
    app.mainloop()