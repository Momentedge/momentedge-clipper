// The `copper_runtime` attribute macro requires `LOG_INDEX_DIR` at compile
// time, and the generated runtime reads the consuming crate's enabled cargo
// features out of `COPPER_CFG_FEATURES`. `cu29_build::setup()` emits both, and
// is the upstream helper every copper application's build script calls.
fn main() {
    cu29_build::setup();
}
