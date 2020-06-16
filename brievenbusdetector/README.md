# Brievenbusdetector Firmware


## Build and flash

Use flash storage:

```bash
bobbin load --bin brievenbusdetector --features use_flash
```

Erase flash storage

```bash
bobbin load --bin brievenbusdetector --features full_erase
```

No features

```bash
bobbin load --bin brievenbusdetector

# or

Cargo run

# or

Cargo build
```

For release

```
cargo build --release --features use_flash,low_power
bobbin load --bin brievenbusdetector --release --features use_flash,low_power
```
