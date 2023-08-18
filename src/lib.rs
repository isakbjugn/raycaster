#![no_std]

use core::{arch::wasm32, panic::PanicInfo};
use core::f32::consts::{FRAC_PI_2, PI};
use libm::{ceilf, cosf, fabsf, floorf, sinf, sqrtf, tanf};

// External WASM-4 Constants
const SCREEN_SIZE: u32 = 160;
const FRAME_RATE: u32 = 60;
const FRAME_WIDTH: f32 = 1_f32 / FRAME_RATE as f32;

static mut PALETTE: *mut [u32; 4] = 0x04 as *mut [u32; 4];

const GAMEPAD1: *const u8 = 0x16 as *const u8;
const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;

const BUTTON_LEFT: u8 = 16;  // 00010000
const BUTTON_RIGHT: u8 = 32; // 00100000
const BUTTON_UP: u8 = 64;    // 01000000
const BUTTON_DOWN: u8 = 128; // 10000000
const BUTTON_SPACE: u8 = 1; // 00000001
const _BUTTON_Z: u8 = 2; // 00000010

const STEP_SIZE: f32 = 0.045;
const GRAVITATIONAL_ACCELERATION: f32 = 6.0;
const INITIAL_JUMP_SPEED: f32 = 3.0;

const FOV: f32 = PI / 2.7; // Spelarens synsfelt
const HALF_FOV: f32 = FOV * 0.5; // Halve spelarens synsfelt
const ANGLE_STEP: f32 = FOV / (SCREEN_SIZE as f32); // Vinkelen mellom kvar stråle
const WALL_HEIGHT: f32 = 100.0; // Eit magisk tal?
const MAP_HEIGHT: usize = 8;
const MAP_WIDTH: usize = 16;

// WASM-4 hjelpe-funksjonar
fn set_colors(colors: u16) {
    unsafe { *DRAW_COLORS = colors; }
}

fn get_colors() -> u16 {
    unsafe { *DRAW_COLORS }
}

fn text(text: &str, x: i32, y: i32) {
    unsafe { extern_text(text.as_ptr(), text.len(), x, y) }
}

// extern functions linking to the wasm runtime
extern "C" {
    fn vline(x: i32, y: i32, len: u32);
    fn rect(x: i32, y: i32, width: u32, height: u32);
    #[link_name = "textUtf8"]
    fn extern_text(text: *const u8, length: usize, x: i32, y: i32);
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
fn phandler(_: &PanicInfo<'_>) -> ! {
    wasm32::unreachable();
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
    text("Finn vegen ut!", 25, 10);
    let mut buffer = ryu::Buffer::new();
    text(buffer.format(STATE.player_z), 30, 25);

    // Gå gjennom kvar kolonne på skjermen og teikn ein vegg ut frå sentrum
    for (x, wall) in STATE.get_view().iter().enumerate() {
        let (height, terrain, orientation) = wall;
        let scaling_factor = *height as f32 / SCREEN_SIZE as f32;
        let wall_top = 80 - (height / 2) + floorf(STATE.player_z * 80.0 * scaling_factor) as i32;

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
            Terrain::Open => panic!("Wall should never have Terrain::Open")
        }
    }
}

const MAP: [u8; MAP_HEIGHT * MAP_WIDTH] = [
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 2,
    1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1,
    1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1,
    1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1,
    1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

#[derive(Clone, Copy, PartialEq)]
enum Terrain {
    Open,
    Wall,
    Doorway,
}

#[derive(Clone, Copy)]
enum Orientation {
    Horizontal,
    Vertical,
}

/// Sjekk ka som finst eit punkt på kartet
fn read_map(x: f32, y: f32) -> Terrain {
    match MAP.get((y as i32 * MAP_WIDTH as i32 + x as i32) as usize) {
        Some(&square) if square == 0 => Terrain::Open,
        Some(&square) if square == 1 => Terrain::Wall,
        Some(&square) if square == 2 => Terrain::Doorway,
        _ => Terrain::Wall,
    }
}

fn distance(a: f32, b: f32) -> f32 {
    sqrtf((a * a) + (b * b))
}

struct State {
    player_x: f32,
    player_y: f32,
    player_z: f32,
    player_velocity: f32,
    player_z_velocity: f32,
    player_angle: f32,
    player_angular_velocity: f32,
}

static mut STATE: State = State {
    player_x: 1.5,
    player_y: 1.5,
    player_z: 0.0,
    player_velocity: 0.0,
    player_z_velocity: 0.0,
    player_angle: -PI / 2_f32,
    player_angular_velocity: 0.0,
};

impl State {
    /// Flytter spelaren
    pub fn update(&mut self, up: bool, down: bool, left: bool, right: bool, jump: bool) {
        // lagre noverandre posisjon i det høvet vi treng han seinare
        let previous_position = (self.player_x, self.player_y);

        if self.player_z == 0.0 && !jump {
            self.player_velocity = STEP_SIZE * up as i32 as f32 - STEP_SIZE * down as i32 as f32;
            self.player_angular_velocity = STEP_SIZE * left as i32 as f32 - STEP_SIZE * right as i32 as f32;
        }

        self.player_x += cosf(self.player_angle) * self.player_velocity;
        self.player_y += -sinf(self.player_angle) * self.player_velocity;
        self.player_angle += self.player_angular_velocity;

        if read_map(self.player_x, self.player_y) == Terrain::Wall {
            if read_map(self.player_x, previous_position.1) == Terrain::Open {
                self.player_y = previous_position.1;
            } else if read_map(previous_position.0, self.player_y) == Terrain::Open {
                self.player_x = previous_position.0;
            } else {
                self.player_x = previous_position.0;
                self.player_y = previous_position.1;
            }
        }

        if jump && self.player_z == 0.0 {
            self.player_z_velocity = INITIAL_JUMP_SPEED;
        }

        if self.player_z > 0.0 {
            self.player_velocity *= 0.975;
            self.player_angular_velocity *= 0.975;
        }

        self.player_z += self.player_z_velocity * FRAME_WIDTH;
        self.player_z_velocity -= GRAVITATIONAL_ACCELERATION * FRAME_WIDTH;
        if self.player_z <= 0.0 {
            self.player_z = 0.0;
            self.player_z_velocity = 0.0;
        }
    }

    /// Gjev tilbake næraste vegg som ei stråle skjer langsetter ei horisontal linje
    fn horizontal_intersection(&self, angle: f32) -> (f32, Terrain) {
        // Seier om vinkelen peikar nordover (i det heile)
        let up = fabsf(floorf(angle / PI) % 2.0) != 0.0;

        // first_x og first_y er dei første skjeringspunkt mellom stråle og gitter
        let first_y = if up {
            ceilf(self.player_y) - self.player_y
        } else {
            floorf(self.player_y) - self.player_y
        };
        let first_x = -first_y / tanf(angle);

        // dy og dx er «stråleforlenginga»
        let dy = if up { 1.0 } else { -1.0 };
        let dx = -dy / tanf(angle);

        // next_x og next_y held styr på kor langt strålen er frå spelaren
        let mut next_x = first_x;
        let mut next_y = first_y;
        let mut terrain = Terrain::Open;

        // Lykkje kor strålen forlengast til han når ein vegg
        for _ in 0..256 {
            // current_x og current_y er strålens noverande posisjon
            let current_x = next_x + self.player_x;
            let current_y = if up {
                next_y + self.player_y
            } else {
                next_y + self.player_y - 1.0
            };

            // Lykkja stoggar når strålen kjem til ein vegg
            terrain = read_map(current_x, current_y);
            if terrain != Terrain::Open {
                break;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        (distance(next_x, next_y), terrain)
    }

    /// Gjev tilbake næraste vegg som ei stråle skjer langsetter ei vertikal linje
    fn vertical_intersection(&self, angle: f32) -> (f32, Terrain) {
        // Seier om vinkelen peikar nordover (i det heile)
        let right = fabsf(floorf((angle - FRAC_PI_2) / PI) % 2.0) != 0.0;

        // first_x og first_y er dei første skjeringspunkt mellom stråle og gitter
        let first_x = if right {
            ceilf(self.player_x) - self.player_x
        } else {
            floorf(self.player_x) - self.player_x
        };
        let first_y = -tanf(angle) * first_x;

        // dy og dx er «stråleforlenginga»
        let dx = if right { 1.0 } else { -1.0 };
        let dy = dx * -tanf(angle);

        // next_x og next_y held styr på kor langt strålen er frå spelaren
        let mut next_x = first_x;
        let mut next_y = first_y;
        let mut terrain = Terrain::Open;

        // Lykkje kor strålen forlengast til han når ein vegg
        for _ in 0..256 {
            // current_x og current_y er strålens noverande posisjon
            let current_x = if right {
                next_x + self.player_x
            } else {
                next_x + self.player_x - 1.0
            };
            let current_y = next_y + self.player_y;

            // Lykkja stoggar når strålen kjem til ein vegg
            terrain = read_map(current_x, current_y);
            if terrain != Terrain::Open {
                break;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        (distance(next_x, next_y), terrain)
    }

    /// Gjev 160 vegghøgder og deira farge frå spelarens perspektiv
    pub fn get_view(&self) -> [(i32, Terrain, Orientation); SCREEN_SIZE as usize] {
        // Start ved enden av spelarens synsfelt
        let starting_angle = self.player_angle + HALF_FOV;

        let mut walls = [(0, Terrain::Open, Orientation::Horizontal); SCREEN_SIZE as usize];

        for (idx, wall) in walls.iter_mut().enumerate() {
            // idx er veggens indeks, wall er ein muterbar referanse til wall-vektoren
            let angle = starting_angle - idx as f32 * ANGLE_STEP;

            // Hentar næraste skjering i horisontal og vertikal retning
            let (h_distance, h_terrain) = self.horizontal_intersection(angle);
            let (v_distance, v_terrain) = self.vertical_intersection(angle);

            let (min_distance, terrain, orientation) = if h_distance < v_distance {
                (h_distance, h_terrain, Orientation::Horizontal)
            } else {
                (v_distance, v_terrain, Orientation::Vertical)
            };

            // Vel minste avstand og konverterer til vegg-høgde
            *wall = (
                (WALL_HEIGHT / (min_distance * cosf(angle - self.player_angle)) ) as i32,
                terrain,
                orientation,
            );
        }

        walls
    }
}