all:
	cargo build --release
	cp target/wasm32-unknown-unknown/release/raycaster.wasm .
	wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm

size:
	du -bh ./target/wasm32-unknown-unknown/release/raycaster.wasm

run: all
	w4 run-native target/wasm32-unknown-unknown/release/raycaster.wasm

run-web: all
	w4 run --no-qr --no-open raycaster.wasm

dev:
	cargo watch -s "make run"