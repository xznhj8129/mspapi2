# Discovering Message Fields

When using messages that don't have convenience methods, you need to know what fields to send in the request and what fields you'll get back in the reply.

## Quick Answer

Use the `introspection.py` helper from the examples directory:

```python
from examples.introspection import print_message_info
from mspapi2 import InavMSP

# See complete details about any message
print_message_info(InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE)
```

**Output:**
```
======================================================================
MSP2_INAV_LOGIC_CONDITIONS_SINGLE
======================================================================
Code:        8251
MSP Version: 2

--- REQUEST ---
  conditionIndex       uint8_t
    → Index of the condition to retrieve (0 to MAX_LOGIC_CONDITIONS - 1)

--- REPLY ---
  enabled              uint8_t
    → Boolean: 1 if enabled
  activatorId          int8_t
    → Activator ID (-1/255 if none)
  operation            uint8_t           (enum: logicOperation_e)
    → Enum logicOperation_e Logical operation
  ...
```

Now you know:
- Request needs: `{"conditionIndex": <number>}`
- Reply contains: `enabled`, `activatorId`, `operation`, etc.
- Which fields are enums

## Method 1: Use the Introspection Helper (Easiest)

```python
from examples.introspection import print_message_info, get_message_info
from mspapi2 import InavMSP

# Print human-readable info
print_message_info(InavMSP.MSP_YOUR_MESSAGE)

# Or get as a dict for programmatic use
info = get_message_info(InavMSP.MSP_YOUR_MESSAGE)
print("Request fields:", info['request']['field_names'])
print("Reply fields:", info['reply']['field_names'])
```

## Method 2: Search for Messages

```python
from examples.introspection import print_all_messages

# Find all messages related to "LOGIC"
print_all_messages("LOGIC")

# Output:
# MSP2_INAV_LOGIC_CONDITIONS                 = 8226
# MSP2_INAV_LOGIC_CONDITIONS_SINGLE          = 8251
# MSP2_INAV_LOGIC_CONDITIONS_STATUS          = 8230
# MSP2_INAV_SET_LOGIC_CONDITIONS             = 8227
```

## Method 3: Look at the JSON Schema

The schema file contains all message definitions:

```bash
# Using jq on command line
jq '.MSP2_INAV_LOGIC_CONDITIONS_SINGLE' mspapi2/lib/msp_messages.json

# Or in Python
import json

with open('mspapi2/lib/msp_messages.json') as f:
    schema = json.load(f)

msg = schema['MSP2_INAV_LOGIC_CONDITIONS_SINGLE']
print("Reply fields:", [f['name'] for f in msg['reply']['payload']])
```

## Method 4: Just Try It and See

```python
from mspapi2 import MSPApi, InavMSP

with MSPApi(port="/dev/ttyACM0") as api:
    # Make a request (it will tell you if you're missing fields)
    try:
        info, reply = api._request(InavMSP.MSP_API_VERSION)
        print("Reply contains:", reply.keys())
        print("Full reply:", reply)
    except Exception as e:
        print(f"Error: {e}")
```

## Practical Workflow

Here's how to use a new message:

### Step 1: Find the message you need

```python
from mspapi2 import InavMSP

# What messages deal with waypoints?
waypoint_msgs = [m for m in dir(InavMSP) if 'WP' in m or 'WAYPOINT' in m]
print(waypoint_msgs)
```

### Step 2: See what fields it needs

```python
from examples.introspection import print_message_info

print_message_info(InavMSP.MSP_WP)
```

### Step 3: Use it

```python
from mspapi2 import MSPApi, InavMSP

with MSPApi(port="/dev/ttyACM0") as api:
    # Now you know what fields are needed
    request = api._pack_request(
        InavMSP.MSP_WP,
        {"waypointIndex": 0}  # From step 2
    )

    info, reply = api._request(InavMSP.MSP_WP, request)

    # Access reply fields (also from step 2)
    print(f"WP Number: {reply['waypointIndex']}")
    print(f"Action: {reply['action']}")
    print(f"Lat: {reply['latitude']}")
    print(f"Lon: {reply['longitude']}")
```

## Understanding Field Types

The schema uses C types that map to Python types:

| C Type | Python Type | Range |
|--------|-------------|-------|
| `uint8_t` | int | 0-255 |
| `int8_t` | int | -128 to 127 |
| `uint16_t` | int | 0-65535 |
| `int16_t` | int | -32768 to 32767 |
| `uint32_t` | int | 0-4294967295 |
| `int32_t` | int | -2147483648 to 2147483647 |
| `float` | float | 32-bit float |
| `char` | bytes | Single byte |
| `bool` | bool | True/False |

Arrays are shown as: `uint8_t[4]` (array of 4 bytes)

## Understanding Enums

Some fields use enums instead of plain numbers:

```python
from mspapi2.lib import InavEnums

# Instead of magic numbers:
if operation == 2:  # What does 2 mean?

# Use enums for clarity:
if operation == InavEnums.logicOperation_e.LOGIC_CONDITION_GREATER_THAN:
    print("Greater than comparison")
```

**Common enum classes:**
- `InavEnums.navigationFSMState_t` - Navigation states
- `InavEnums.navWaypointActions_e` - Waypoint actions
- `InavEnums.logicOperation_e` - Logic operations
- `InavEnums.logicOperandType_e` - Operand types
- Many more in `mspapi2/lib/inav_enums.py`

## Tips

1. **Start with introspection** - Use `print_message_info()` first
2. **Use enums** - Convert enum fields for readability
3. **Check the schema** - All info is in `msp_messages.json`
4. **Look at examples** - See how similar messages are used
5. **Try it** - Python will tell you if fields are wrong

## Example: Complete Workflow

```python
from mspapi2 import MSPApi, InavMSP
from mspapi2.lib import InavEnums
from examples.introspection import print_message_info

# 1. Find out what fields the message has
print_message_info(InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE)

# 2. Use the message
with MSPApi(port="/dev/ttyACM0") as api:
    # Pack request with fields from step 1
    request = api._pack_request(
        InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
        {"conditionIndex": 0}
    )

    # Send request
    info, reply = api._request(
        InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
        request
    )

    # Process reply using fields from step 1
    condition = {
        "enabled": bool(reply["enabled"]),
        "operation": InavEnums.logicOperation_e(reply["operation"]),
        "operandAType": InavEnums.logicOperandType_e(reply["operandAType"]),
        "operandAValue": reply["operandAValue"],
        # ... etc
    }

    print(f"Condition: {condition}")
```

## Common Issues

### "Field 'xyz' not found in message"

You're trying to use a field that doesn't exist. Check the schema:

```python
from examples.introspection import print_message_info
print_message_info(InavMSP.YOUR_MESSAGE)
```

### "Message has no request payload"

Some messages don't need request data:

```python
# Wrong:
request = api._pack_request(InavMSP.MSP_API_VERSION, {})

# Right:
info, reply = api._request(InavMSP.MSP_API_VERSION)
```

### "Wrong number of fields"

Make sure you're providing all required fields. Check the schema to see what's needed.

## See Also

- **[Getting Started](./GETTING_STARTED.md)** - Basic library usage
- **[Examples](../examples/)** - Working code examples
- **[Main README](../README.md)** - API reference
