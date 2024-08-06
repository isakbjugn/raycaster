use libm::{ceilf, cosf, fabsf, floorf, sinf, sqrtf, tanf};
use core::f32::consts::{FRAC_PI_2, PI};
use crate::constants::{FRAME_WIDTH, SCREEN_SIZE};
use crate::map::{MAP_HEIGHT, MAP_WIDTH, Orientation, read_map, Terrain};

const STEP_SIZE: f32 = 0.045;
const GRAVITATIONAL_ACCELERATION: f32 = 6.0;
const INITIAL_JUMP_SPEED: f32 = 3.0;

const FOV: f32 = PI / 2.7; // Spelarens synsfelt
const HALF_FOV: f32 = FOV * 0.5; // Halve spelarens synsfelt
const ANGLE_STEP: f32 = FOV / (SCREEN_SIZE as f32); // Vinkelen mellom kvar stråle
const WALL_HEIGHT: f32 = 100.0; // Eit magisk tal?
const FLOOR_HEIGHT: f32 = 50.0;

pub enum View {
    FirstPerson,
    Victory,
    Fooled,
}

pub struct State {
    pub game_won: bool,
    pub view: View,
    pub player_x: f32,
    pub player_y: f32,
    pub player_z: f32,
    pub player_velocity: f32,
    pub player_z_velocity: f32,
    pub player_angle: f32,
    pub player_angular_velocity: f32,
}

fn distance(a: f32, b: f32) -> f32 {
    sqrtf((a * a) + (b * b))
}

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

        if read_map(previous_position.0, previous_position.1) == Terrain::Freeze &&
            read_map(self.player_x, self.player_y) != Terrain::Freeze {
            self.player_x = previous_position.0;
            self.player_y = previous_position.1;
        }

        match read_map(self.player_x, self.player_y) {
            Terrain::Open => {
                if !self.game_won {
                    self.view = View::FirstPerson;
                }
            },
            Terrain::Wall => {
                if read_map(self.player_x, previous_position.1) == Terrain::Open {
                    self.player_y = previous_position.1;
                } else if read_map(previous_position.0, self.player_y) == Terrain::Open {
                    self.player_x = previous_position.0;
                } else {
                    self.player_x = previous_position.0;
                    self.player_y = previous_position.1;
                }
            },
            Terrain::Freeze => {},
            Terrain::Doorway => {
                self.view = View::Victory;
                self.player_x = previous_position.0;
                self.player_y = previous_position.1;
            },
            Terrain::Mirage => {
                self.view = View::Fooled;
            },
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

    /// Gjev alle terreng til og med første horisontale vegg som ein stråle skjer
    fn horizontal_intersections(&self, angle: f32) -> [(f32, Terrain); MAP_HEIGHT] {
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

        let mut intersections = [(100.0, Terrain::Open); MAP_HEIGHT];

        // Lykkje kor strålen forlengast til han når ein vegg
        for grid_line in intersections.iter_mut() {
            // current_x og current_y er strålens noverande posisjon
            let current_x = next_x + self.player_x;
            let current_y = if up {
                next_y + self.player_y
            } else {
                next_y + self.player_y - 1.0
            };

            // Lykkja stoggar når strålen kjem til ein vegg
            *grid_line = (distance(next_x, next_y), read_map(current_x, current_y));

            if grid_line.1 != Terrain::Open {
                //intersections[MAP_HEIGHT - 1] = (1.0, Terrain::Wall); // avlusing
                return intersections;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        intersections[MAP_HEIGHT - 1] = (intersections[MAP_HEIGHT - 1].0, Terrain::Wall);

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        intersections
    }

    /// Gjev alle terreng til og med første vertikale vegg som ein stråle skjer
    fn vertical_intersections(&self, angle: f32) -> [(f32, Terrain); MAP_WIDTH] {
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

        let mut intersections = [(100.0, Terrain::Open); MAP_WIDTH];
        //intersections[MAP_HEIGHT - 1] = (100.0, Terrain::Wall);

        // Lykkje kor strålen forlengast til han når ein vegg
        for grid_line in intersections.iter_mut() {
            // current_x og current_y er strålens noverande posisjon
            let current_x = if right {
                next_x + self.player_x
            } else {
                next_x + self.player_x - 1.0
            };
            let current_y = next_y + self.player_y;

            // Lykkja stoggar når strålen kjem til ein vegg
            *grid_line = (distance(next_x, next_y), read_map(current_x, current_y));

            if grid_line.1 != Terrain::Open {
                //intersections[MAP_WIDTH - 1] = (1.0, Terrain::Wall); // avlusing
                return intersections;
            }

            // forleng strålen så lenge me ikkje har nådd ein vegg
            next_x += dx;
            next_y += dy;
        }

        intersections[MAP_WIDTH - 1] = (intersections[MAP_WIDTH - 1].0, Terrain::Wall);

        // gje tilbake avstanden fra (next_x, next_y) til spelarens posisjon
        intersections
    }

    pub fn get_terrains(&self) -> [[(i32, Terrain); MAP_WIDTH]; SCREEN_SIZE as usize] {
        // Start ved enden av spelarens synsfelt
        let starting_angle = self.player_angle + HALF_FOV;

        let mut terrains = [[(0, Terrain::Open); MAP_WIDTH]; SCREEN_SIZE as usize];

        for (idx, direction) in terrains.iter_mut().enumerate() {
            // idx er veggens indeks, wall er ein muterbar referanse til wall-vektoren
            let angle = starting_angle - idx as f32 * ANGLE_STEP;
            let h_intersections = self.horizontal_intersections(angle);
            let v_intersections = self.vertical_intersections(angle);

            if h_intersections.find_first_obstacle_or_wall().0 < v_intersections.find_first_obstacle_or_wall().0 {
                for (cell, terrain) in h_intersections.iter().enumerate() {
                    direction[cell] = (
                        (FLOOR_HEIGHT / (terrain.0 * cosf(angle - self.player_angle)) ) as i32,
                        terrain.1,
                    );
                }
            } else {
                for (cell, terrain) in v_intersections.iter().enumerate() {
                    direction[cell] = (
                        (FLOOR_HEIGHT / (terrain.0 * cosf(angle - self.player_angle)) ) as i32,
                        terrain.1,
                    );
                }
            }
        }

        terrains
    }

    pub fn get_view(&self) -> [(i32, Terrain, Orientation); SCREEN_SIZE as usize] {
        // Start ved enden av spelarens synsfelt
        let starting_angle = self.player_angle + HALF_FOV;

        let mut walls = [(0, Terrain::Wall, Orientation::Horizontal); SCREEN_SIZE as usize];

        for (idx, wall) in walls.iter_mut().enumerate() {
            // idx er veggens indeks, wall er ein muterbar referanse til wall-vektoren
            let angle = starting_angle - idx as f32 * ANGLE_STEP;

            // Hentar næraste skjering i horisontal og vertikal retning
            let (h_distance, h_terrain) = self.horizontal_intersections(angle)
                .find_first_obstacle_or_wall();

            let (v_distance, v_terrain) = self.vertical_intersections(angle)
                .find_first_obstacle_or_wall();

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

trait Rays {
    fn find_first_obstacle_or_wall(&self) -> (f32, Terrain);
}

impl Rays for [(f32, Terrain)] {
    fn find_first_obstacle_or_wall(&self) -> (f32, Terrain) {
        for &tuple in self {
            /*
                != Terrain::Open for å tegne vegger og utganger likt
                == Terrain::Wall for å lage uendelig lang utgang (ganske kult)
             */
            if tuple.1 == Terrain::Wall {
                return tuple
            }
        }
        (self.last().unwrap().0, Terrain::Wall)
    }
}
