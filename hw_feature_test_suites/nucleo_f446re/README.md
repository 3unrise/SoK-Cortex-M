Link to the GoogleDrive Document 

https://uccsoffice365-my.sharepoint.com/:w:/g/personal/apadiyal_uccs_edu/EbtBpDt2p7dPn8wQoQ-r1awBHwUQqn1_jXAvR8i2_dfkiA?e=0oHe4r



# Nucleo F446RE Development Board

This folder contains tests and documentation for the Nucleo F446RE development board.

---

## Purpose
- Demonstrate Cortex-M features specific to the Nucleo F446RE board.
- Organize test suites and relevant files for this board.

---

## Getting Started
- Load the appropriate default project files into **STM32CubeIDE**.

---

## MPU Configuration Overview

### **Configuration Steps and Initial Assignments**
1. **Disabling the MPU**  
   The MPU (Memory Protection Unit) is initially disabled to allow configuration changes.  

2. **Configuring Memory Regions**  
   Region 0 is selected and configured to cover the start of SRAM. The attributes include access permissions, memory type, and region size to ensure safe and controlled memory access.  

3. **Enabling MPU and Default Memory Mapping**  
   After configuration, the MPU is enabled, and the system retains a default memory map for regions not explicitly defined.  

4. **Enabling Memory Fault Handling**  
   The system is configured to trigger faults when any illegal or unauthorized memory access occurs, aiding in debugging and protection.  

5. **Applying Memory Barriers**  
   Data and instruction synchronization barriers ensure the MPU configuration takes effect correctly across the system.
   MPU Configuration:

Configures Region 0 to cover 64 KB of SRAM1 at address 0x20000000 with read/write access.
Enables memory faults to detect unauthorized accesses.
Privileged and Unprivileged Access Testing:

In privileged mode, memory accesses succeed, and GPIO pins (LEDs) toggle as feedback.
The CPU is switched to unprivileged mode, where memory writes may cause faults if access is restricted.
UART Communication:

Outputs messages via UART to monitor program status and transitions between modes.
Access Permissions Table
AP[2:0]	Privileged Access	Unprivileged Access	Notes
000	No access	No access	Any access generates a permission fault
001	Read/Write	No access	Privileged access only
010	Read/Write	Read-only	Unprivileged write generates a permission fault
011	Read/Write	Read/Write	Full access
100	UNPREDICTABLE	UNPREDICTABLE	Reserved
101	Read-only	No access	Privileged read-only
110	Read-only	Read-only	Privileged and unprivileged read-only
111	Read-only	Read-only	Privileged and unprivileged read-only with additional checks


---

## Console Setup

### **UART Console Configuration**
- The UART interface is used for serial communication between the development board and an external device (like a PC).  
- Key settings include the serial port, baud rate, data size, parity, and stop bits to ensure correct data transmission.

---

## Data Watch Point and Trace (`DWT`)

### **Purpose:**  
This hardware feature is used to measure the number of clock cycles required to execute specific functions, providing insights into performance and efficiency.

### **How It Works:**  
- The cycle counter is enabled and reset to measure the time taken by a function from start to finish.  
- The cycle count is then sent via UART to an external terminal or device for analysis.  
- This data is critical for profiling, debugging, and optimizing the code.

---

## ChaCha20 Encryption (`Monocipher`)

### **Overview:**  
ChaCha20 is a secure and efficient algorithm used to encrypt and decrypt data. It ensures confidentiality during data storage and transmission by transforming plaintext into unreadable ciphertext.

### **Main Components:**
- **Secret Key (`key[]`):** A 256-bit key that only authorized users possess. It is used to encrypt and decrypt the data securely.  
- **Nonce (`nonce[]`):** A unique value that ensures encryption uniqueness, even when the same data is encrypted multiple times.  
- **Plaintext (`plaintext[]`):** The original readable data that needs encryption.  
- **Ciphertext (`ciphertext[]`):** The encrypted version of the plaintext, which is unreadable without the key and nonce.  
- **Encryption and Decryption Functions:** The encryption process scrambles the plaintext into ciphertext, and the decryption process reverses it using the same key and nonce.  
- **Message Length:** Ensures that the correct amount of data is processed without errors or truncation.

---

## Memory Region Configurations (`F1` to F3)

### **Purpose:**  
Each directory (`F1` to `F6`) demonstrates the MPU's ability to control memory access by varying base addresses, sizes, and permission settings.

### **Region Settings Overview:**
- **Base Addresses:** Assigned to specific memory regions like SRAM or Flash.  
- **Region Sizes:** Defines the size of each memory region, typically 32 KB for SRAM and 1 MB for Flash memory.  
- **Access Permissions:** Ranges from no access to full access, depending on the security level needed.

### **Key Permissions (AP Settings):**
- No access for privileged and unprivileged modes.  
- Read/write access for privileged modes only.  
- Full access for both privileged and unprivileged modes.  
- Read-only access to protect critical data in Flash memory.

### **Fault Handling:**  
Attempts to access restricted regions trigger memory faults, which are handled using exception handlers to prevent system crashes.

---

## NVIC and GPIO Interrupts

### **Purpose:**  
The NVIC (Nested Vector Interrupt Controller) manages interrupts with different priority levels, ensuring real-time responsiveness.

### **Functionality:**  
- External interrupts (like button presses) are configured through GPIO pins.  
- The NVIC prioritizes and handles these interrupts efficiently.  
- LEDs or other indicators are used to provide feedback on interrupt handling.

---

## Initial Setup for MAX78000 Board

### **1. Flashing the Initial Bootloader**  
The board is connected via USB, and the insecure bootloader is flashed to enable further firmware updates. A blinking blue LED confirms successful flashing.

### **2. Cloning the Reference Design**  
The necessary project files and reference firmware are cloned from the GitHub repository, including components for decoders, designs, and tools.

### **3. Setting Up a Python Virtual Environment**  
A virtual environment is created to manage Python dependencies required for building and deploying the firmware.

### **4. Locating the Serial Port**  
The serial port used to communicate with the board is identified using the Windows Device Manager, ensuring the correct COM port is selected.

### **5. Generating Secrets for Secure Deployment**  
A shared secret is generated for both the encoder and decoder to ensure secure communication.

### **6. Building and Running the Decoder**  
The decoder firmware is built using Docker, creating the necessary binary files for deployment.

### **7. Generating a Subscription Update**  
A subscription file is generated to handle secure communication between devices and the decoder.

### **8. Flashing the Decoder Firmware**  
The built firmware is flashed onto the board, with the correct serial port and mode settings ensuring proper installation.

---

#Data Watch Point and Trace (DWT)
Purpose:
This hardware feature is used to measure performance by tracking memory accesses or the time taken by functions. It also enables watchpoints to debug access to specific variables.

DWT Watchpoint Program:
The DWT Watchpoint Program demonstrates the use of watchpoints to monitor memory read and write operations on the variable test_variable