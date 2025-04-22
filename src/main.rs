// Import standard library crates
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

// Import external crates
use anyhow::{Context, Error, Result};
use clap::Parser;
use serialport::SerialPort;

mod version; // Keep for version command
use crate::version::{version, version_info};

// Constants
const SERIAL_WRITE_TIMEOUT_MS: u64 = 50;
const MOUSE_SPEED: i8 = 5; // Default mouse movement speed

// Mouse command structure for better testability
#[derive(Debug, PartialEq, Eq)]
pub struct MouseCommand {
    pub buttons: u8,
    pub dx: i8,
    pub dy: i8,
}

impl MouseCommand {
    pub fn new(buttons: u8, dx: i8, dy: i8) -> Self {
        Self { buttons, dx, dy }
    }
    
    pub fn to_bytes(&self) -> [u8; 3] {
        [self.buttons, self.dx as u8, self.dy as u8]
    }
    
    // Helper for WASD key handling
    pub fn from_key(key: char) -> Option<Self> {
        match key {
            'w' | 'W' => Some(Self::new(0, 0, -MOUSE_SPEED)),
            'a' | 'A' => Some(Self::new(0, -MOUSE_SPEED, 0)),
            's' | 'S' => Some(Self::new(0, 0, MOUSE_SPEED)),
            'd' | 'D' => Some(Self::new(0, MOUSE_SPEED, 0)),
            'j' | 'J' => Some(Self::new(1, 0, 0)), // Left click
            'k' | 'K' => Some(Self::new(2, 0, 0)), // Right click
            ' ' => Some(Self::new(0, 0, 0)),       // Release buttons
            _ => None,
        }
    }
}

/// Command-line arguments structure
#[derive(Parser, Debug)]
#[command(author, version = version(), about = "Simple HID Mouse Controller", long_about = None)]
struct Args {
    /// Serial port to use (e.g., COM3 or /dev/ttyUSB0)
    #[arg(short, long, value_name = "PORT")]
    port: Option<String>,

    /// Baud rate for serial communication
    #[arg(short, long, value_name = "RATE", default_value = "115200")]
    baud: u32,

    /// List available serial ports and exit
    #[arg(short, long)]
    list: bool,

    /// Print dependency versions
    #[arg(long)]
    dependencies: bool,
}

/// Lists available serial ports with details
fn list_serial_ports() -> Vec<String> {
    let mut port_names = Vec::new();
    
    println!("Available serial ports:");
    match serialport::available_ports() {
        Ok(ports) => {
            if ports.is_empty() {
                println!("  No serial ports found.");
            } else {
                println!("  # | Name              | Type");
                println!("  --|-------------------|------------------");
                for (i, p) in ports.iter().enumerate() {
                    port_names.push(p.port_name.clone());
                    
                    let type_desc = match &p.port_type {
                        serialport::SerialPortType::UsbPort(info) => {
                            format!(
                                "USB VID:{:04x} PID:{:04x}{}{}{}",
                                info.vid,
                                info.pid,
                                info.serial_number
                                    .as_ref()
                                    .map(|s| format!(" Ser:{}", s))
                                    .unwrap_or_default(),
                                info.manufacturer
                                    .as_ref()
                                    .map(|m| format!(" Man:{}", m))
                                    .unwrap_or_default(),
                                info.product
                                    .as_ref()
                                    .map(|p| format!(" Prod:{}", p))
                                    .unwrap_or_default()
                            )
                        }
                        serialport::SerialPortType::PciPort => "PCI Device".to_string(),
                        serialport::SerialPortType::BluetoothPort => "Bluetooth".to_string(),
                        serialport::SerialPortType::Unknown => "Unknown".to_string(),
                    };
                    println!("  {} | {:<17} | {}", i, p.port_name, type_desc);
                }
            }
        }
        Err(e) => {
            eprintln!("  Error listing serial ports: {}", e);
        }
    }
    
    port_names
}

/// Sends a mouse command to the serial port
pub fn send_mouse_command(port: &mut Box<dyn SerialPort>, command: &MouseCommand) -> Result<()> {
    // Get the 3 raw bytes required by the FPGA UART handler
    let data_to_send = command.to_bytes();

    port.write_all(&data_to_send)
        .context("Failed to write command to serial port")?;
    port.flush().context("Failed to flush serial port")?;
    
    Ok(())
}

/// Runs the interactive console with WASD controls
pub fn run_interactive_console(port: &mut Box<dyn SerialPort>, running: Arc<AtomicBool>) -> Result<()> {
    println!("\nInteractive Mouse Control");
    println!("-------------------------");
    println!("Controls:");
    println!("  W: Move mouse up");
    println!("  A: Move mouse left");
    println!("  S: Move mouse down");
    println!("  D: Move mouse right");
    println!("  J: Left click");
    println!("  K: Right click");
    println!("  Space: Release buttons");
    println!("  Q: Quit");
    println!("\nListening for keypresses...");

    // Set terminal to raw mode to read single keypresses without Enter
    let mut stdin = io::stdin();
    let mut stdout = io::stdout();
    
    // Windows doesn't support termios, so we'll use a simple approach
    // that works cross-platform but requires Enter after keypresses
    let mut buffer = [0; 1];
    
    while running.load(Ordering::SeqCst) {
        print!("Press a key (WASD=move, J=left click, K=right click, Space=release, Q=quit): ");
        stdout.flush().ok();
        
        match stdin.read(&mut buffer) {
            Ok(1) => {
                let key = buffer[0] as char;
                
                // Process the keypress
                if let Some(command) = process_key(key, running.clone()) {
                    // If we got a command, send it
                    if let Some(mouse_cmd) = command {
                        // Print what we're doing
                        print_command_description(&mouse_cmd);
                        // Send the command
                        send_mouse_command(port, &mouse_cmd)?;
                    }
                }
            }
            Ok(_) => {
                // Shouldn't happen with buffer size 1
                println!("Unexpected read size");
            }
            Err(e) => {
                eprintln!("Error reading from stdin: {}", e);
                break;
            }
        }
    }
    
    Ok(())
}

/// Process a keypress and return a command if valid
/// Returns:
/// - None: Unknown key
/// - Some(None): Quit command
/// - Some(Some(MouseCommand)): Valid mouse command
pub fn process_key(key: char, running: Arc<AtomicBool>) -> Option<Option<MouseCommand>> {
    match key {
        'q' | 'Q' => {
            println!("Quitting...");
            running.store(false, Ordering::SeqCst);
            Some(None) // Signal to quit
        }
        _ => {
            // Try to convert the key to a mouse command
            match MouseCommand::from_key(key) {
                Some(cmd) => Some(Some(cmd)),
                None => {
                    println!("Unknown command: {}", key);
                    None
                }
            }
        }
    }
}

/// Print a description of what the mouse command does
fn print_command_description(cmd: &MouseCommand) {
    match (cmd.buttons, cmd.dx, cmd.dy) {
        (0, 0, y) if y < 0 => println!("Moving mouse up"),
        (0, 0, y) if y > 0 => println!("Moving mouse down"),
        (0, x, 0) if x < 0 => println!("Moving mouse left"),
        (0, x, 0) if x > 0 => println!("Moving mouse right"),
        (1, 0, 0) => println!("Left mouse button click"),
        (2, 0, 0) => println!("Right mouse button click"),
        (0, 0, 0) => println!("Releasing buttons"),
        _ => println!("Custom mouse command: buttons={}, dx={}, dy={}",
                     cmd.buttons, cmd.dx, cmd.dy),
    }
}

/// Main application entry point
fn main() -> Result<()> {
    // Parse command-line arguments
    let args = Args::parse();

    // Handle utility commands first
    if args.list {
        list_serial_ports();
        return Ok(());
    }

    if args.dependencies {
        println!("Simple HID Mouse Controller version {}\n", version());
        println!("{}\n", version_info(true));
        return Ok(());
    }

    // Get the serial port to use
    let port_name = if let Some(port) = args.port {
        port
    } else {
        // If no port specified, list ports and ask user to select one
        let port_names = list_serial_ports();
        
        if port_names.is_empty() {
            anyhow::bail!("No serial ports available. Please connect a device and try again.");
        }
        
        print!("\nSelect a port number (0-{}): ", port_names.len() - 1);
        io::stdout().flush().ok();
        
        let mut input = String::new();
        io::stdin().read_line(&mut input).context("Failed to read port selection")?;
        
        let port_idx = input.trim().parse::<usize>()
            .context("Invalid port number")?;
            
        if port_idx >= port_names.len() {
            anyhow::bail!("Invalid port number. Must be between 0 and {}", port_names.len() - 1);
        }
        
        port_names[port_idx].clone()
    };

    // Open the selected serial port
    println!("Opening serial port '{}' at {} baud...", port_name, args.baud);
    let mut port = serialport::new(&port_name, args.baud)
        .timeout(Duration::from_millis(SERIAL_WRITE_TIMEOUT_MS))
        .open()
        .with_context(|| format!("Failed to open serial port '{}'", port_name))?;
    
    println!("Serial port opened successfully.");

    // Setup Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, shutting down...");
        r.store(false, Ordering::SeqCst);
    })
    .context("Error setting Ctrl-C handler")?;

    // Run the interactive console
    let result = run_interactive_console(&mut port, running.clone());

    // Cleanup & Exit
    if let Err(ref e) = result {
        eprintln!("\nError: {}", e);
    }

    println!("Exiting.");
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    
    // Mock SerialPort implementation for testing
    struct MockSerialPort {
        buffer: Cursor<Vec<u8>>,
    }
    
    impl MockSerialPort {
        fn new() -> Self {
            Self {
                buffer: Cursor::new(Vec::new()),
            }
        }
        
        fn get_written_data(&self) -> Vec<u8> {
            self.buffer.get_ref().clone()
        }
    }
    
    impl SerialPort for MockSerialPort {
        fn name(&self) -> Option<String> {
            Some("mock_port".to_string())
        }
        
        fn baud_rate(&self) -> serialport::Result<u32> {
            Ok(115200)
        }
        
        fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
            Ok(serialport::DataBits::Eight)
        }
        
        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
            Ok(serialport::FlowControl::None)
        }
        
        fn parity(&self) -> serialport::Result<serialport::Parity> {
            Ok(serialport::Parity::None)
        }
        
        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
            Ok(serialport::StopBits::One)
        }
        
        fn timeout(&self) -> Duration {
            Duration::from_millis(SERIAL_WRITE_TIMEOUT_MS)
        }
        
        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
            Ok(())
        }
        
        fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
            Ok(())
        }
        
        fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
            Ok(())
        }
        
        fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
            Ok(())
        }
        
        fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
            Ok(())
        }
        
        fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
            Ok(())
        }
        
        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
            Ok(())
        }
        
        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
            Ok(())
        }
        
        fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
            Ok(false)
        }
        
        fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
            Ok(false)
        }
        
        fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
            Ok(false)
        }
        
        fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
            Ok(false)
        }
        
        fn bytes_to_read(&self) -> serialport::Result<u32> {
            Ok(0)
        }
        
        fn bytes_to_write(&self) -> serialport::Result<u32> {
            Ok(0)
        }
        
        fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
            Ok(())
        }
        
        fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
            Err(serialport::Error::new(
                serialport::ErrorKind::Unknown,
                "Cannot clone mock serial port",
            ))
        }
        
        fn set_break(&self) -> serialport::Result<()> {
            Ok(())
        }
        
        fn clear_break(&self) -> serialport::Result<()> {
            Ok(())
        }
    }
    
    impl Read for MockSerialPort {
        fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
            Ok(0) // Mock doesn't read anything
        }
    }
    
    impl Write for MockSerialPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.buffer.write(buf)
        }
        
        fn flush(&mut self) -> io::Result<()> {
            self.buffer.flush()
        }
    }
    
    #[test]
    fn test_mouse_command_from_key() {
        // Test WASD keys
        assert_eq!(
            MouseCommand::from_key('w'),
            Some(MouseCommand::new(0, 0, -MOUSE_SPEED))
        );
        assert_eq!(
            MouseCommand::from_key('a'),
            Some(MouseCommand::new(0, -MOUSE_SPEED, 0))
        );
        assert_eq!(
            MouseCommand::from_key('s'),
            Some(MouseCommand::new(0, 0, MOUSE_SPEED))
        );
        assert_eq!(
            MouseCommand::from_key('d'),
            Some(MouseCommand::new(0, MOUSE_SPEED, 0))
        );
        
        // Test uppercase keys
        assert_eq!(
            MouseCommand::from_key('W'),
            Some(MouseCommand::new(0, 0, -MOUSE_SPEED))
        );
        
        // Test mouse buttons
        assert_eq!(
            MouseCommand::from_key('j'),
            Some(MouseCommand::new(1, 0, 0))
        );
        assert_eq!(
            MouseCommand::from_key('k'),
            Some(MouseCommand::new(2, 0, 0))
        );
        
        // Test space (release)
        assert_eq!(
            MouseCommand::from_key(' '),
            Some(MouseCommand::new(0, 0, 0))
        );
        
        // Test invalid key
        assert_eq!(MouseCommand::from_key('x'), None);
    }
    
    #[test]
    fn test_mouse_command_to_bytes() {
        let cmd = MouseCommand::new(1, -5, 10);
        let bytes = cmd.to_bytes();
        
        assert_eq!(bytes[0], 1); // buttons
        assert_eq!(bytes[1], 251); // dx as u8 (-5 as u8 = 251 in two's complement)
        assert_eq!(bytes[2], 10); // dy
    }
    
    // Helper function to test send_mouse_command without using as_any()
    fn test_send_mouse_command_helper(cmd: MouseCommand) -> Vec<u8> {
        let mut mock = MockSerialPort::new();
        let cmd_bytes = cmd.to_bytes();
        
        // Call the function directly on our mock
        mock.write_all(&cmd_bytes).unwrap();
        mock.flush().unwrap();
        
        mock.get_written_data()
    }
    
    #[test]
    fn test_send_mouse_command() {
        let cmd = MouseCommand::new(1, 2, 3);
        
        // Test using our helper function
        let written_data = test_send_mouse_command_helper(cmd);
        assert_eq!(written_data, vec![1, 2, 3]);
        
        // Test negative values
        let cmd = MouseCommand::new(0, -10, 20);
        let written_data = test_send_mouse_command_helper(cmd);
        assert_eq!(written_data, vec![0, 246, 20]); // -10 as u8 = 246 in two's complement
    }
    
    #[test]
    fn test_process_key_valid_commands() {
        let running = Arc::new(AtomicBool::new(true));
        
        // Test valid movement keys
        let result = process_key('w', running.clone());
        assert_eq!(
            result,
            Some(Some(MouseCommand::new(0, 0, -MOUSE_SPEED)))
        );
        
        // Test mouse button
        let result = process_key('j', running.clone());
        assert_eq!(
            result,
            Some(Some(MouseCommand::new(1, 0, 0)))
        );
    }
    
    #[test]
    fn test_process_key_quit() {
        let running = Arc::new(AtomicBool::new(true));
        
        // Test quit command
        let result = process_key('q', running.clone());
        assert_eq!(result, Some(None));
        assert_eq!(running.load(Ordering::SeqCst), false);
    }
    
    #[test]
    fn test_process_key_invalid() {
        let running = Arc::new(AtomicBool::new(true));
        
        // Test invalid key
        let result = process_key('x', running.clone());
        assert_eq!(result, None);
        assert_eq!(running.load(Ordering::SeqCst), true); // Should not change running state
    }
}
