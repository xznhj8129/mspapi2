#!/bin/bash
SPEC_URL="https://raw.githubusercontent.com/iNavFlight/inav/refs/heads/master/docs/development/msp/msp_messages.json"
curl "$SPEC_URL" > mspapi2/lib/msp_messages.json