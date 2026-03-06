# Labyrint

Labyrint er et spill skrevet i Rust og kompilert til den svært begrensede fantasikonsollen `WASM-4`.

## Installasjon
Installasjonen av `WASM-4` er beskrevet på https://wasm4.org/docs/getting-started/setup.

```shell
npm install -g wasm4
```

```shell
cargo install wasm-snip
```

## Bygg og kjør på native

```shell
cargo build --release
cp target/wasm32-unknown-unknown/release/raycaster.wasm .
wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm
```

```shell
 w4 run-native raycaster.wasm
```

## Bygg og kjør til web

```shell
cargo build --release --features save
cp target/wasm32-unknown-unknown/release/raycaster.wasm .
wasm-snip --snip-rust-fmt-code --snip-rust-panicking-code raycaster.wasm -o raycaster.wasm
```

```shell
w4 run --no-qr raycaster.wasm
```


## Pakk til html

```shell
 w4 bundle raycaster.wasm --title "Labyrint" --html labyrint.html
```
