all:
	cargo build --release

size:
	du -bh ./target/wasm32-unknown-unknown/release/raycaster.wasm

run: all
	w4 run-native target/wasm32-unknown-unknown/release/raycaster.wasm

dev:
	cargo watch -s "make run"