# Contributing to active-ragdoll-rs

Thank you for your interest in contributing to `active-ragdoll-rs`! We welcome contributions ranging from bug reports and documentation fixes to joint solver enhancements, animation blending improvements, and physics engine integrations (such as Rapier3D or Bevy).

---

## Code of Conduct

All contributors and maintainers are expected to adhere to our [Code of Conduct](CODE_OF_CONDUCT.md). Please read it to understand our community standards.

---

## Development Setup

`active-ragdoll-rs` is written in pure Rust (2024 edition). You will need:
- Rust toolchain (`stable` 1.85+)
- `cargo`, `rustfmt`, and `clippy`

### Clone and Build

```bash
git clone https://github.com/bhubbard/active-ragdoll-rs.git
cd active-ragdoll-rs
cargo build
```

---

## Running Tests and Examples

Always verify that the test suite and example demos pass before opening a pull request:

```bash
# Run all unit and integration tests
cargo test --all-targets

# Run clippy with strict warnings
cargo clippy --all-targets -- -D warnings

# Run headless active ragdoll simulation
cargo run --example headless_simulation

# Run Rapier3D physics integration example
cargo run --example rapier_ragdoll --features rapier3d
```

---

## Pull Request Guidelines

1. **Keep Pull Requests Focused**: Limit changes to a single feature or bug fix.
2. **Determinism and Numerical Stability**: Ensure PD controllers, quaternions, and joint drive calculations are numerically stable under variable integration timesteps.
3. **Format and Lint**: Run `cargo fmt` and `cargo clippy --all-targets -- -D warnings` before submitting.
4. **Preserve Compatibility**: Keep the public API clean, modular, and idiomatic Rust.
