#![no_std]

mod state;
mod constants;
mod map;

use core::{arch::wasm32, panic::PanicInfo};
use core::f32::consts::{PI};
use libm::{floorf};
use crate::constants::SCREEN_SIZE;
use crate::map::{Orientation, Terrain};
use crate::state::{State, View};

static mut PALETTE: *mut [u32; 4] = 0x04 as *mut [u32; 4];

const GAMEPAD1: *const u8 = 0x16 as *const u8;
const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;

const BUTTON_LEFT: u8 = 16;  // 00010000
const BUTTON_RIGHT: u8 = 32; // 00100000
const BUTTON_UP: u8 = 64;    // 01000000
const BUTTON_DOWN: u8 = 128; // 10000000
const BUTTON_SPACE: u8 = 1; // 00000001
const _BUTTON_Z: u8 = 2; // 00000010

// WASM-4 hjelpe-funksjonar
fn set_colors(colors: u16) {
    unsafe { *DRAW_COLORS = colors; }
}

fn get_colors() -> u16 {
    unsafe { *DRAW_COLORS }
}

// extern functions linking to the wasm runtime
extern "C" {
    fn vline(x: i32, y: i32, len: u32);
    fn rect(x: i32, y: i32, width: u32, height: u32);
    #[link_name = "textUtf8"]
    fn extern_text(text: *const u8, length: usize, x: i32, y: i32);
}

fn text(text: &str, x: i32, y: i32) {
    unsafe { extern_text(text.as_ptr(), text.len(), x, y) }
}

fn extract_colors() -> (u16, u16) {
    let colors = get_colors();
    // Extract the first digit (6) and create the first u16 value (0x11)
    let primary_digit = (colors >> 4) & 0x0F;
    let primary = (primary_digit << 4) | primary_digit;

    // Extract the second digit (5) and create the second u16 value (0x44)
    let secondary_digit = colors & 0x0F;
    let secondary = (secondary_digit << 4) | secondary_digit;
    (primary, secondary)
}

fn dashed_vline(x: i32, y: i32, len: u32) {
    let (primary, secondary) = extract_colors();
    set_colors(primary);
    for b in (y..y + (1 - (x % 2)) + len as i32).step_by(2) {
        unsafe { vline(x, b, 1) }
    }
    set_colors(secondary);
    for b in (y + 1..y + (x % 2) + len as i32).step_by(2) {
        unsafe { vline(x, b, 1) }
    }
}

#[panic_handler]
fn panic_handler(panic_info: &PanicInfo<'_>) -> ! {
    unsafe { wasm32::unreachable() }
}

// Køyrer ved oppstart
#[no_mangle]
fn start() {
    unsafe {
        *PALETTE = [0x2B2D24, 0x606751, 0x949C81, 0x3E74BC];
    }
}

// Køyrer for kvart bilete
#[no_mangle]
unsafe fn update() {
    STATE.update(
        *GAMEPAD1 & BUTTON_UP != 0,
        *GAMEPAD1 & BUTTON_DOWN != 0,
        *GAMEPAD1 & BUTTON_LEFT != 0,
        *GAMEPAD1 & BUTTON_RIGHT != 0,
        *GAMEPAD1 & BUTTON_SPACE != 0,
    );

    // draw the ground and sky
    set_colors(0x44);
    rect(0, 0, SCREEN_SIZE, SCREEN_SIZE / 2);
    set_colors(0x33);
    rect(0, (SCREEN_SIZE / 2) as i32, SCREEN_SIZE, SCREEN_SIZE / 2);

    set_colors(0x41);
    match STATE.view {
        View::FirstPerson => text("Finn vegen ut!", 25, 10),
        View::Victory => {
            text("Du klarte det!", 25, 10);
        }
    }

    // let mut buffer = ryu::Buffer::new();
    // text(buffer.format(STATE.player_z), 30, 25);
    for (x, terrains) in STATE.get_terrains().iter().enumerate() {
        for direction in terrains.iter().filter(|terrain| terrain.1 != Terrain::Wall) {
            let (height, terrain) = direction;
            let scaling_factor = *height as f32 / SCREEN_SIZE as f32;
            let wall_top = 80 - (height / 2) + floorf(STATE.player_z * 80.0 * scaling_factor) as i32;
            let horison = 80 + floorf(STATE.player_z * 80.0 * scaling_factor) as i32;

            match terrain {
                Terrain::Wall => {
                    vline(x as i32, wall_top, *height as u32);
                },
                Terrain::Doorway => {
                    set_colors(0x24);
                    dashed_vline(x as i32, wall_top, *height as u32);
                },
                Terrain::Mirage => {
                    set_colors(0x24);
                    dashed_vline(x as i32, wall_top, *height as u32);
                },
                Terrain::Freeze => panic!("Terrain::Freeze is not an obstacle!"),
                Terrain::Open => panic!("Wall should never have Terrain::Open"),
            }
        }
    }

    // Gå gjennom kvar kolonne på skjermen og teikn ein vegg ut frå sentrum
    for (x, wall) in STATE.get_view().iter().enumerate() {
        let (height, terrain, orientation) = wall;
        let scaling_factor = *height as f32 / SCREEN_SIZE as f32;
        let wall_top = 80 - (height / 2) + floorf(STATE.player_z * 80.0 * scaling_factor) as i32;
        let horison = 80 + floorf(STATE.player_z * 80.0 * scaling_factor) as i32;

        match terrain {
            Terrain::Wall => {
                match orientation {
                    Orientation::Vertical => { set_colors(0x11); },
                    Orientation::Horizontal => { set_colors(0x22); },
                }
                vline(x as i32, wall_top, *height as u32);
            },
            Terrain::Doorway => {
                set_colors(0x24);
                dashed_vline(x as i32, wall_top, *height as u32);
            },
            Terrain::Mirage => {
                set_colors(0x24);
                dashed_vline(x as i32, wall_top, *height as u32);
            },
            Terrain::Freeze => panic!("Terrain::Freeze is not an obstacle!"),
            Terrain::Open => panic!("Wall should never have Terrain::Open"),
        }
    }
}

static mut STATE: State = State {
    view: View::FirstPerson,
    player_x: 1.5,
    player_y: 1.5,
    player_z: 0.0,
    player_velocity: 0.0,
    player_z_velocity: 0.0,
    player_angle: -PI / 2_f32,
    player_angular_velocity: 0.0,
};