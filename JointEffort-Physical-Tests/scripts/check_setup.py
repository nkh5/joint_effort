#!/usr/bin/env python3
"""Check Python and list USB serial devices. Never opens a motor port."""
import os
from pathlib import Path
import sys
if sys.platform != 'linux':
    raise SystemExit('Use Linux on the Pi or Ubuntu computer connected to the servo adapter.')
try:
    import serial
except ImportError:
    raise SystemExit('Missing pyserial. Run ./setup.sh on Ubuntu.')
print('Python:', sys.version.split()[0], '| pyserial:', serial.__version__)
ports=sorted(Path('/dev/serial/by-id').glob('*'))
if not ports:
    print('No USB serial devices found. Connect the servo adapter and check USB passthrough if using a VM.')
for port in ports:
    access=os.access(port, os.R_OK | os.W_OK)
    print(port.name, '->', port.resolve(), '| readable/writable:', access)
print('This check did not open a port or send a command. See README.md for the motor test.')
