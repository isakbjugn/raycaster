build:
	cargo build --release
	cp target/wasm32-unknown-unknown/release/raycaster.wasm .
	wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm

build-web:
	cargo build --release --features save
	cp target/wasm32-unknown-unknown/release/raycaster.wasm .
	wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm

bundle:
	cp target/wasm32-unknown-unknown/release/raycaster.wasm .
	wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm

size:
	du -bh ./target/wasm32-unknown-unknown/release/raycaster.wasm

run: build bundle
	w4 run-native raycaster.wasm

run-web: build-web bundle
	w4 run --no-qr raycaster.wasm
	# w4 run --no-qr --no-open raycaster.wasm

dev:
	cargo watch -s "make run"