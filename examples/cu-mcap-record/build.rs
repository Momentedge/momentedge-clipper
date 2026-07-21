// The `copper_runtime` attribute macro requires `LOG_INDEX_DIR` at compile
// time. Its canonical helper `cu29_build::setup()` is not published to
// crates.io, so a consumer of the published cu29 1.0 crates inlines what it
// does: point `LOG_INDEX_DIR` at `OUT_DIR` and forward the enabled cargo
// features as `COPPER_CFG_FEATURES`.
fn main() {
    println!(
        "cargo::rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("Cargo must provide OUT_DIR")
    );

    let mut features: Vec<_> = std::env::var("CARGO_CFG_FEATURE")
        .unwrap_or_default()
        .split(',')
        .filter(|f| !f.is_empty())
        .map(str::to_owned)
        .collect();
    features.sort_unstable();
    features.dedup();
    println!(
        "cargo::rustc-env=COPPER_CFG_FEATURES={}",
        features.join(",")
    );
}
