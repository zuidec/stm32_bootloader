BOOTLOADER_SIZE = 0x8000
BOOTLOADER_FILE = "bootloader.bin"

# Open and read in bin
with open(BOOTLOADER_FILE, "rb") as f:
    raw_file = f.read()

# Calculate how much padding to create
bytes_to_pad = BOOTLOADER_SIZE - len(raw_file)
padding = bytes([0xff for _ in range(bytes_to_pad)])

# Write the original file along with padding
with open(BOOTLOADER_FILE, "wb") as f:
    f.write(raw_file + padding)
