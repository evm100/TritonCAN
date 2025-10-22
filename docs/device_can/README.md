# Device CAN Documentation Hub

This directory collects device-specific Controller Area Network (CAN) reference material for
TritonCAN. Each subdirectory captures the original vendor literature and a curated
integration guide written for our embedded and API teams.

## Goals

* Preserve authoritative vendor documentation (PDFs, CAD, datasheets) alongside our notes.
* Translate vendor terminology into the conventions we use for TritonCAN bridges and API
  modules.
* Provide a repeatable template so future devices can be documented and parsed by
  automation tooling (scraper/parser/interpreter) that will assemble CAN modules for the
  runtime API.

## Directory Layout

```
docs/
  device_can/
    README.md               # This file
    TEMPLATE.md             # Canonical outline for new device dossiers
    <vendor>/
      <device>/
        README.md           # Human-readable integration guide
        metadata.yaml       # Machine-friendly summary for automation
        <original docs>     # PDFs, images, CAD, etc. kept verbatim
```

*`metadata.yaml` files are optional for legacy devices, but required for new additions so that
our forthcoming automation pipeline has a consistent schema to ingest.*

## Adding a New Device

1. Copy `TEMPLATE.md` into a new `<vendor>/<device>/README.md` and fill in every section.
2. Place all vendor-supplied material (PDFs, spreadsheets, CAD files, etc.) in the same folder.
3. Capture structured data in `metadata.yaml` (see examples in existing devices).
4. Open a pull request describing the new device and how it should be loaded by the API.

## Using the Documentation

The embedded firmware team should rely on the per-device README as the authoritative guide
for wiring, CAN IDs, payload definitions, and API surface mappings. The API team can build a
module by reading `metadata.yaml` and aligning it with the dynamic loading strategy so the
runtime can expose new layman-friendly functions without hard-coding device logic.

The long-term plan is to have a documentation parser that converts each device dossier into a
module bundle. The consistency enforced here is what will make that automation tractable.

## Demo & Reference Devices

Looking for a software-only sandbox before touching production hardware? The
[examples/virtual_blinker](./examples/virtual_blinker/README.md) dossier shows
how to stand up two virtual CAN endpoints on a shared `vcan` bus and exercise
the TritonCAN daemon with a hello-world "blink" command. It includes the YAML
and DBC artifacts plus scripts that spawn paired interactive terminals.
