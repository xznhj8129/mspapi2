# Examples

Practical examples showing how to use mspapi2.

## Running the Examples

All examples can run against either serial or TCP:

```bash
# Via serial (direct connection)
python examples/basic_usage.py --port /dev/ttyACM0

# Via TCP (through msp_server.py)
python examples/basic_usage.py --tcp localhost:9000
```

## Examples

### introspection.py

Helper functions for discovering message structure.

```bash
# Run standalone to see examples
python examples/introspection.py
```

**Use in your code:**
```python
from examples.introspection import print_message_info
from mspapi2 import InavMSP

print_message_info(InavMSP.MSP_YOUR_MESSAGE)
```

**Functions provided:**
- `get_message_info(code)` - Get message details as dict
- `print_message_info(code)` - Pretty-print message structure
- `list_all_messages(filter)` - List available messages
- `print_all_messages(filter)` - Pretty-print message list

### basic_usage.py

Fundamental operations: connecting, reading FC info, reading sensors.

```bash
python examples/basic_usage.py --port /dev/ttyACM0
```

**Shows:**
- Connection setup (serial and TCP)
- Reading FC info (version, board, variant)
- Reading sensors (attitude, altitude, GPS, battery)
- Error handling
- Context manager usage

### logic_conditions.py

Using messages that don't have convenience methods.

```bash
# Fetch condition #0
python examples/logic_conditions.py --port /dev/ttyACM0 --index 0
```

**Shows:**
- The 3-step pattern for ANY message:
  1. Pack request data
  2. Send request
  3. Process reply
- Using enums for readability
- How to find out what fields are available

This pattern works for all 249 MSP messages!

## Creating Your Own Examples

### Template for Simple Example

```python
from mspapi2 import MSPApi

with MSPApi(port="/dev/ttyACM0") as api:
    # Read some data
    info, attitude = api.get_attitude()
    print(f"Roll: {attitude['roll']}°")
```

### Template for Custom Message

```python
from mspapi2 import MSPApi, InavMSP
from examples.introspection import print_message_info

# First, find out what fields the message has
print_message_info(InavMSP.MSP_YOUR_MESSAGE)

# Then use it
with MSPApi(port="/dev/ttyACM0") as api:
    request = api._pack_request(
        InavMSP.MSP_YOUR_MESSAGE,
        {"field1": value1}  # Fields from print_message_info
    )

    info, reply = api._request(InavMSP.MSP_YOUR_MESSAGE, request)
    print(reply)
```

## See Also

- **[Getting Started Guide](../docs/GETTING_STARTED.md)** - Basic concepts
- **[Discovering Fields](../docs/DISCOVERING_FIELDS.md)** - How to find message structure
- **[Server Setup](../docs/SERVER.md)** - TCP server usage
- **[Main README](../README.md)** - Complete API reference
