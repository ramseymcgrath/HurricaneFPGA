// src/main.rs

// Import standard library crates
use std::io::{self, BufRead, BufReader, Write}; // Added Write
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

// Import external crates
use anyhow::{bail, Context, Error};
use clap::Parser;
use serialport::SerialPort; // Import the trait

mod version; // Keep for version command

// Import necessary items from local modules
use crate::version::{version, version_info}; // Adjust if version module structure changed

// Constants
const UDP_READ_TIMEOUT_MS: u64 = 100; // Prevent blocking forever on UDP socket
// const SERIAL_READ_TIMEOUT_MS: u64 = 100; // For optional serial listener
const MAX_UDP_PACKET_SIZE: usize = 1024; // Max size for incoming UDP commands
const CONTROL_SERIAL_WRITE_TIMEOUT_MS: u64 = 50; // Short timeout for writing to FPGA
const CONTROL_SERIAL_READ_TIMEOUT_MS: u64 = 1; // Very short read timeout if needed for control port

/// Command-line arguments structure
#[derive(Parser, Debug)]
#[command(author, version = version(), about = "KMBox HID Command Gateway for UART-Controlled Cynthion", long_about = None)]
struct Args {
    /// Run in UDP server mode, listening on <IP:PORT> (e.g., 127.0.0.1:9001)
    /// Requires --control-serial to specify the Cynthion's command port.
    #[arg(long, value_name = "IP:PORT")]
    udp: Option<String>,

    /// **REQUIRED**: Serial port connected to the Cynthion PMOD for sending injection commands
    /// (e.g., COM3 or /dev/ttyUSB0).
    #[arg(long = "control-serial", value_name = "PORT")]
    control_serial_port: String, // Make this required

    /// Baud rate for the Cynthion control serial communication. MUST match FPGA.
    #[arg(long = "control-baud", value_name = "RATE", default_value = "115200")]
    control_baud: u32,

    /// List available serial ports and exit (useful for finding the control port)
    #[arg(long)]
    list: bool,

    /// Print dependency versions (use with --version)
    #[arg(long)]
    dependencies: bool,

    /* // --- TODO: kmbox b+ serial ---
    /// Run in Serial server mode, *listening* for commands on the specified port
    #[arg(long = "listen-serial", value_name = "PORT", conflicts_with = "udp")]
    listen_serial_port: Option<String>,

    /// Baud rate for the serial *listener* port
    #[arg(long = "listen-baud", value_name = "RATE", default_value = "115200")]
    listen_baud: u32,
    */
}

/// Lists available serial ports with more detail.
fn list_serial_ports() {
    println!("Available serial ports:");
    match serialport::available_ports() {
        Ok(ports) => {
            if ports.is_empty() {
                println!("  No serial ports found.");
            } else {
                println!("  Name              | Type");
                println!("  ------------------|------------------");
                for p in ports {
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
                    println!("  {:<17} | {}", p.port_name, type_desc);
                }
            }
        }
        Err(e) => {
            eprintln!("  Error listing serial ports: {}", e);
        }
    }
}

/// Handles the --list command-line argument.
fn handle_list_command() -> Result<(), Error> {
    println!("--- Serial Ports ---");
    list_serial_ports();
    println!(
        "\nNote: Cynthion injection control now uses the serial port specified via --control-serial."
    );
    Ok(())
}

/// Parses the command string "buttons,dx,dy" into (u8, i8, i8)
fn parse_command(cmd_str: &str) -> Result<(u8, i8, i8), Error> {
    let parts: Vec<&str> = cmd_str.trim().split(',').collect();
    if parts.len() != 3 {
        bail!(
            "Invalid command format. Expected 'buttons,dx,dy', got '{}'",
            cmd_str
        );
    }

    // Trim whitespace from each part before parsing
    let buttons_str = parts[0].trim();
    let dx_str = parts[1].trim();
    let dy_str = parts[2].trim();

    let buttons = buttons_str
        .parse::<u8>()
        .with_context(|| format!("Failed to parse buttons value: '{}'", buttons_str))?;
    let dx = dx_str
        .parse::<i8>()
        .with_context(|| format!("Failed to parse dx value: '{}'", dx_str))?;
    let dy = dy_str
        .parse::<i8>()
        .with_context(|| format!("Failed to parse dy value: '{}'", dy_str))?;

    Ok((buttons, dx, dy))
}

/// Sends the 3-byte command to the control serial port.
fn send_control_command(
    port: &mut Box<dyn SerialPort>,
    buttons: u8,
    dx: i8,
    dy: i8,
) -> Result<(), Error> {
    // Format the 3 raw bytes required by the FPGA UART handler
    // Casting i8 to u8 preserves the bit pattern, which is what we want for raw transmission.
    let data_to_send: [u8; 3] = [buttons, dx as u8, dy as u8];

    print!(
        " -> Sending to Cynthion: {:02X} {:02X} {:02X} ... ",
        data_to_send[0], data_to_send[1], data_to_send[2]
    );
    // Ensure message appears before potentially blocking write/flush
    io::stdout().flush().ok();

    port.write_all(&data_to_send)
        .context("Failed to write command to control serial port")?;
    port.flush().context("Failed to flush control serial port")?;
    println!("OK"); // Confirmation after successful send + flush
    Ok(())
}

/// Runs the UDP server, listening for commands and forwarding them.
fn run_udp_server(
    addr: &str,
    control_port: &mut Box<dyn SerialPort>, // Accepts the opened control port
    running: Arc<AtomicBool>,
) -> Result<(), Error> {
    let socket = UdpSocket::bind(addr).context(format!("Failed to bind UDP socket to {}", addr))?;
    socket
        .set_read_timeout(Some(Duration::from_millis(UDP_READ_TIMEOUT_MS)))
        .context("Failed to set UDP read timeout")?;
    println!("Listening for HID commands on UDP {}...", addr);
    println!(
        "Forwarding commands to control port {}...",
        control_port.name().unwrap_or_else(|| "Unknown".to_string())
    );

    let mut buf = [0u8; MAX_UDP_PACKET_SIZE]; // Reusable buffer

    while running.load(Ordering::SeqCst) {
        match socket.recv_from(&mut buf) {
            Ok((num_bytes, src_addr)) => {
                // Process received data
                let received_data = &buf[..num_bytes];
                match std::str::from_utf8(received_data) {
                    Ok(cmd_str) => {
                        print!("\rUDP <- {}: {}", src_addr, cmd_str.trim()); // Log received command, \r overprints dots
                        io::stdout().flush().ok(); // Ensure log appears

                        match parse_command(cmd_str) {
                            Ok((buttons, dx, dy)) => {
                                // --- Forward Command to Control Port ---
                                if let Err(e) = send_control_command(control_port, buttons, dx, dy)
                                {
                                    // Use eprint! for errors so they stand out and aren't overwritten
                                    eprintln!(
                                        "\nError sending command to control port: {}",
                                        e
                                    );
                                    // Consider if errors should halt the program or just be logged
                                }
                                // --- End Forward ---
                            }
                            Err(e) => {
                                // Use eprint! for parsing errors
                                eprintln!("\nError parsing command from {}: {}", src_addr, e);
                            }
                        }
                    }
                    Err(_) => {
                        eprintln!("\nReceived non-UTF8 data from {}", src_addr);
                        //eprintln!(" Bytes: {:?}", received_data);
                    }
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // Timeout is expected, allows checking the 'running' flag
                // Print a dot to show activity without flooding logs
                print!(".");
                io::stdout().flush().ok();
                // Short sleep to prevent tight loop consuming CPU on timeout
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                // Other UDP read error
                eprintln!("\nUDP receive error: {}. Retrying in 1s...", e);
                // Avoid busy-looping on persistent errors
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
    Ok(())
}

/* // --- KMBOX B+ logic ---
fn run_serial_listener(
    listen_port_name: &str,
    listen_baud_rate: u32,
    control_port: &mut Box<dyn SerialPort>, // Accepts the opened control port
    running: Arc<AtomicBool>,
) -> Result<(), Error> {
     let listen_port = serialport::new(listen_port_name, listen_baud_rate)
        .timeout(Duration::from_millis(SERIAL_READ_TIMEOUT_MS)) // Use constant
        .open()
        .context(format!("Failed to open listen serial port '{}'", listen_port_name))?;

    // Use BufReader for potentially easier line-based reading (if commands are newline terminated)
    let mut reader = BufReader::new(listen_port);
    let mut line_buf = String::new();

    println!(
        "Listening for HID commands on Serial {} ({} baud)...",
        listen_port_name, listen_baud_rate
    );
     println!(
         "Forwarding commands to control port {}...",
         control_port.name().unwrap_or_else(|| "Unknown".to_string())
    );

    while running.load(Ordering::SeqCst) {
        line_buf.clear();
        match reader.read_line(&mut line_buf) {
             Ok(0) => {
                 // End Of File/Stream - Port likely closed or disconnected
                 if !running.load(Ordering::SeqCst) { break; } // Exit if Ctrl+C pressed
                 eprintln!("\nListen serial port connection closed or pipe ended. Retrying in 1s...");
                  std::thread::sleep(Duration::from_secs(1));
                  // Consider attempting to reopen the listen_port here, requires more complex state management
                  continue;
            }
             Ok(_) => {
                 // Successfully read a line
                  print!("\rSerial Listen <- {}", line_buf.trim()); // Log received command
                  io::stdout().flush().ok();

                match parse_command(&line_buf) {
                    Ok((buttons, dx, dy)) => {
                         // --- Forward Command to Control Port ---
                         if let Err(e) = send_control_command(control_port, buttons, dx, dy) {
                            eprintln!("\nError sending command to control port: {}", e);
                        }
                         // --- End Forward ---
                    }
                    Err(e) => {
                       eprintln!("\nError parsing command from Serial Listen: {}", e);
                    }
                }
            }
             Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Timeout is expected, allows checking the 'running' flag
                 print!("."); // Show activity
                 io::stdout().flush().ok();
                 std::thread::sleep(Duration::from_millis(50)); // Prevent busy loop
                continue;
            }
             Err(e) => {
                 // Other serial read error
                 eprintln!("\nListen serial read error: {}. Retrying in 1s...", e);
                 std::thread::sleep(Duration::from_secs(1)); // Avoid busy-looping
            }
        }
    }
    Ok(())
}
*/

/// Main application entry point
fn main() -> Result<(), Error> {
    // Parse command-line arguments
    let args = Args::parse();

    // Handle utility commands first
    if args.list {
        return handle_list_command();
    }

    if args.dependencies {
         // Ensure version_info function exists in version module and prints dependencies
        println!("KMBox UART Gateway version {}\n", version());
        println!("{}\n", version_info(true));
        return Ok(());
    }

    // --- Open the Control Serial Port ---
    println!(
        "Opening control serial port '{}' at {} baud (8N1)...",
        args.control_serial_port, args.control_baud
    );
    let mut control_port = serialport::new(&args.control_serial_port, args.control_baud)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        // Set timeouts: short write, very short read (if needed, often not for write-only)
        .timeout(Duration::from_millis(CONTROL_SERIAL_WRITE_TIMEOUT_MS))
        .open()
        .with_context(|| {
            format!(
                "Failed to open control serial port '{}'",
                args.control_serial_port
            )
        })?;
    println!("Control serial port opened successfully.");

    // --- Setup Ctrl+C Handler ---
    // Arc<AtomicBool> allows safe signal handling across threads (though not used here)
    // and provides a clear flag to check in loops.
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        print!("\rReceived Ctrl+C, shutting down..."); // Use print! + flush to ensure visibility
        io::stdout().flush().ok();
        r.store(false, Ordering::SeqCst); // Signal loops to stop
    })
    .context("Error setting Ctrl-C handler")?;

    // --- Run the selected server mode ---
    let result = if let Some(udp_addr) = args.udp {
        println!("Starting UDP listener mode.");
        run_udp_server(&udp_addr, &mut control_port, running.clone()) // Pass control port and running flag
    }
    /* // Enable this block if using the optional Serial Listener
    else if let Some(listen_serial_port_name) = args.listen_serial_port {
         println!("Starting serial listener mode.");
         run_serial_listener(
             &listen_serial_port_name,
             args.listen_baud,
             &mut control_port,
             running.clone()
         )
    }
    */
     else {
         // If no listening mode is specified, return an error or provide default behavior.
          bail!("No command input mode specified. Use --udp <IP:PORT>. (Or --listen-serial if enabled).");
          //TODO: Could run an interactive loop reading from stdin here.
    };

    // --- Cleanup & Exit ---
    // Check if the server loop exited due to an error (other than Ctrl+C)
    if let Err(ref e) = result {
        eprintln!("\nServer loop exited with error: {}", e);
    }

    println!("\nClosing control port...");
    // Serial port is closed automatically when `control_port` (which owns the Box<dyn SerialPort>)
    // goes out of scope due to its Drop implementation. No explicit close needed.
    println!("Exited.");

    // Return Ok if server loop finished normally (likely due to Ctrl+C)
    // Return the error if the server loop returned an error
    result
}
