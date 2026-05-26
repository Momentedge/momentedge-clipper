//! rclrs-subscriber — placeholder.
//!
//! Intended as the rclrs counterpart to `r2r-subscriber`: attach to every live
//! topic and record each message as raw serialized CDR into per-topic,
//! timestamp-indexed maps.
//!
//! Not implemented yet. rclrs (as of v0.7) has no raw-CDR / `SerializedMessage`
//! subscription — its `create_dynamic_subscription` fully decodes into a
//! `DynamicMessage`, which is the wrong tool for a minimal-copy recorder. The
//! gap is tracked upstream in ros2-rust/ros2_rust#326; the rcl FFI primitive
//! (`rcl_take_serialized_message`) exists in rclrs's generated bindings but has
//! no safe wrapper. This binary stays a stub until that API lands (or we
//! contribute it).
//!
//! Building rclrs also needs the ament/colcon toolchain (cargo-ament-build),
//! which the flake does not yet provide; wiring that up is part of this work.

fn main() {
    eprintln!(
        "rclrs-subscriber is not implemented yet: rclrs has no raw-CDR subscription \
         (see ros2-rust/ros2_rust#326). Use the r2r-subscriber binary for now."
    );
    std::process::exit(1);
}
