#![no_std]

use core::{arch::wasm32, panic::PanicInfo};
use core::f32::consts::{FRAC_PI_2, PI};
use libm::{ceilf, cosf, fabsf, floorf, sinf, sqrtf, tanf};

// External WASM-4 Constants
const SCREEN_SIZE: u32 = 160;

static mut PALETTE: *mut [u32; 4] = 0x04 as *mut [u32; 4];

const GAMEPAD1: *const u8 = 0x16 as *const u8;
const DRAW_COLORS: *mut u16 = 0x14 as *mut u16;

const BUTTON_LEFT: u8 = 16;  // 00010000
const BUTTON_RIGHT: u8 = 32; // 00100000
const BUTTON_UP: u8 = 64;    // 01000000
const BUTTON_DOWN: u8 = 128; // 10000000

const STEP_SIZE: f32 = 0.045;

const FOV: f32 = PI / 2.7; // Spelarens synsfelt
const HALF_FOV: f32 = FOV * 0.5; // Halve spelarens synsfelt
const ANGLE_STEP: f32 = FOV / (SCREEN_SIZE as f32); // Vinkelen mellom kvar stråle
const WALL_HEIGHT: f32 = 100.0; // Eit magisk tal?


extern "C" {
    fn vline(x: i32, y: i32, len: u32);
    fn rect(x: i32, y: i32, width: u32, height: u32);
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
    );

    // draw the ground and sky
    *DRAW_COLORS = 0x44;
    rect(0, 0, SCREEN_SIZE, SCREEN_SIZE / 2);
    *DRAW_COLORS = 0x33;
    rect(0, (SCREEN_SIZE / 2) as i32, SCREEN_SIZE, SCREEN_SIZE / 2);

    // Gå gjennom kvar colonne på skjermen og teikn ein vegg ut frå sentrum
    for (x, wall) in STATE.get_view().iter().enumerate() {
        let (height, shadow) = wall;

        if *shadow {
            *DRAW_COLORS = 0x11;
        } else {
            *DRAW_COLORS = 0x22;
        }

        vline(x as i32, 80 - (height / 2), *height as u32);
    }
}

const MAP: [u16; 8] = [
    0b1111111111111111,
    0b1000001010000101,
    0b1011100000110101,
    0b1000111010010001,
    0b1010001011110111,
    0b1011101001100001,
    0b1000100000001101,
    0b1111111111111111,
];

/// Sjekk om det finst ein vegg på eit punkt på kartet
fn point_in_wall(x: f32, y: f32) -> bool {
    match MAP.get(y as usize) {
        Some(line) => (line & (0b1 << x as usize)) != 0,
        None => true,
    }
}

enum Wall {
    Horizontal,
    Vertical,
}

fn met_wall(new_position: (f32, f32), previous_position: (f32, f32)) -> Option<Wall> {
    if point_in_wall(new_position.0, previous_position.1) { Some(Wall::Horizontal) }
    else if point_in_wall(previous_position.0, new_position.1) { Some(Wall::Vertical) }
    else { None }
}

fn distance(a: f32, b: f32) -> f32 {
    sqrtf((a * a) + (b * b))
}

struct State {
    player_x: f32,
    player_y: f32,
    player_angle: f32,
}

static mut STATE: State = State {
    player_x: 1.5,
    player_y: 1.5,
    player_angle: 0.0,
};

impl State {
    /// Flytter spelaren
    pub fn update(&mut self, up: bool, down: bool, left: bool, right: bool) {
        // lagre noverandre posisjon i det høvet vi treng han seinare
        let previous_position = (self.player_x, self.player_y);

        if up {
            self.player_x += cosf(self.player_angle) * STEP_SIZE;
            self.player_y += -sinf(self.player_angle) * STEP_SIZE;
        }

        if down {
            self.player_x -= cosf(self.player_angle) * STEP_SIZE;
            self.player_y -= -sinf(self.player_angle) * STEP_SIZE;
        }

        if right {
            self.player_angle -= STEP_SIZE;
        }

        if left {
            self.player_angle += STEP_SIZE;
        }

        match met_wall((self.player_x, self.player_y), previous_position) {
            Some(Wall::Horizontal) => {
                self.player_x = previous_position.0;
                self.player_y = self.player_y - (self.player_y - previous_position.1) / 2_f32;
            }
            Some(Wall::Vertical) => {
                self.player_x = self.player_x - (self.player_x - previous_position.0) / 2_f32;
                self.player_y = previous_position.1;
            }
            None => {},
        }
    }

    /// Gjev tilbake næraste vegg som ei stråle skjer langsetter ei horisontal linje
    fn horizontal_intersection(&self, angle: f32) -> f32 {
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
            if point_in_wall(current_x, current_y) {
                break;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        distance(next_x, next_y)
    }

    /// Gjev tilbake næraste vegg som ei stråle skjer langsetter ei vertikal linje
    fn vertical_intersection(&self, angle: f32) -> f32 {
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
            if point_in_wall(current_x, current_y) {
                break;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        distance(next_x, next_y)
    }

    /// Gjev 160 vegghøgder og deira farge frå spelarens perspektiv
    pub fn get_view(&self) -> [(i32, bool); SCREEN_SIZE as usize] {
        // Start ved enden av spelarens synsfelt
        let starting_angle = self.player_angle + HALF_FOV;

        let mut walls = [(0, false); SCREEN_SIZE as usize];

        for (idx, wall) in walls.iter_mut().enumerate() {
            // idx er veggens indeks, wall er ein muterbar referanse til wall-vektoren
            let angle = starting_angle - idx as f32 * ANGLE_STEP;

            // Hentar næraste skjering i horisontal og vertikal retning
            let h_dist = self.horizontal_intersection(angle);
            let v_dist = self.vertical_intersection(angle);

            let (min_dist, shadow) = if h_dist < v_dist {
                (h_dist, false)
            } else {
                (v_dist, true)
            };

            // Vel minste avstand og konverterer til vegg-høgde
            *wall = (
                (WALL_HEIGHT / (min_dist * cosf(angle - self.player_angle)) ) as i32,
                shadow,
            );
        }

        walls
    }
}