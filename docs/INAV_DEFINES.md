# INAV define provenance

`mspapi2/lib/inav_defines.py` is derived from object-like C macros in the
matching INAV source branch. It is not part of INAV's MSP JSON generation.
Consequently, `syncjson.sh` updates `msp_messages.json`, `inav_enums.json`, and
`inav_version.py`, but does not update `inav_defines.py`.

The generator lives in the companion `msp_sdk` repository:

1. From `msp_sdk/generator`, run `get_inav_defines.py`. It scans the configured
   INAV source tree and records object-like macros and their source files in
   `all_defines.h`.
2. Run `bad_define_parse.py`. It discovers typedef aliases from the recorded
   source files, parses supported expressions as C, resolves macro
   dependencies, and writes `inav_defines.py`.
3. Copy the generated file into `mspapi2/lib/` only after running both
   repositories' tests against the matching INAV branch.

The Python library currently requires five concrete definitions:

- `CHANNEL_RANGE_MIN`
- `CHANNEL_RANGE_STEP_WIDTH`
- `SETTING_TYPE_OFFSET`
- `SETTING_SECTION_OFFSET`
- `SETTING_MODE_OFFSET`

The first two are used directly by `MSPApi.get_mode_ranges()`. The three
setting offsets are made available while evaluating expressions from
`inav_enums.json`.

Macros that depend on a target's preprocessor configuration, external
functions, conflicting conditional branches, or unsupported compiler
extensions are emitted as `None`. `inav_enums.py` only exposes integer-valued
defines to enum evaluation, so configuration-dependent enum members remain
absent instead of receiving guessed values.
