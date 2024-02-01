use bevy::prelude::*;
use bevy_xpbd_3d::{parry::bounding_volume::BoundingVolume, prelude::*};
use std::{
    cmp::Ordering,
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .add_systems(Update, move_agent)
        .run();
}

#[derive(Component)]
struct Obstacle;

#[derive(Component)]
struct Goal;

#[derive(Component, Default)]
struct Agent {
    path: Option<VecDeque<IVec2>>,
}

impl Agent {
    pub fn has_path(&self) -> bool {
        self.path.is_some()
    }

    pub fn compute_path(&mut self, agent_transform: &Transform, goal_transform: &Transform) {
        // A* Algorithm

        let mut dissallowed = HashSet::<IVec2>::new();
        for coords in [(-1, 0), (0, -1), (-2, 1), (1, -2), (0, -3), (-3, 0)] {
            dissallowed.insert(IVec2::new(coords.0, coords.1));
        }

        let mut start_cell = Cell::from(agent_transform.translation);
        let goal_cell = Cell::from(goal_transform.translation);
        start_cell.set_heuristic(0, goal_cell);

        let mut open_set = HashSet::<IVec2>::new();
        open_set.insert(start_cell.coordinates());

        let mut open_queue = BinaryHeap::<Cell>::new();
        open_queue.push(start_cell);

        let mut parent_map = HashMap::<IVec2, IVec2>::new();
        let mut cost_map = HashMap::<IVec2, u32>::new();
        cost_map.insert(start_cell.into(), 0);

        let mut final_cell = None;
        while !open_set.is_empty() {
            let current = open_queue.pop().unwrap();

            if current.coordinates() == goal_cell.coordinates() {
                final_cell = Some(current);
                break;
            }

            let current_cost = cost_map.get(&current.coordinates()).unwrap().to_owned();

            for movement in [IVec2::X, IVec2::Y, IVec2::NEG_X, IVec2::NEG_Y] {
                let coordinates = current.coordinates() + movement;
                if dissallowed.contains(&coordinates) {
                    continue;
                }

                let previous_cost = cost_map.get(&coordinates);
                let new_cost = current_cost + 1;
                if previous_cost.is_none() || previous_cost.unwrap().to_owned() > new_cost {
                    parent_map.insert(coordinates, current.coordinates());
                    cost_map.insert(coordinates, current_cost + 1);
                    if !open_set.contains(&coordinates) {
                        let neighbor_cell = Cell::new(coordinates, new_cost, goal_cell);
                        open_queue.push(neighbor_cell);
                        open_set.insert(coordinates);
                    }
                }
            }
        }

        if let Some(cell) = final_cell {
            let mut path = VecDeque::<IVec2>::new();
            let start_coordinates = cell.coordinates();
            let mut parent = Some(&start_coordinates);
            while parent.is_some() {
                let coordinates = parent.unwrap().to_owned();
                path.push_front(coordinates);
                parent = parent_map.get(&coordinates);
            }
            self.path = Some(path);
        }
    }

    pub fn next_point(&mut self, current_point: Vec3) -> Option<Vec3> {
        if let Some(path) = &mut self.path {
            if let Some(next) = path.front() {
                if next.as_vec2().distance(current_point.xz()) < 0.1 {
                    println!("Reached {}", next);
                    path.pop_front();
                    if let Some(next2) = path.front() {
                        println!("Going to {}", next2);
                    } else {
                        println!("No more points!");
                    }
                }
            }

            if let Some(next) = path.front() {
                return Some(Vec3::new(next.x as f32, current_point.y, next.y as f32));
            }
        }
        None
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Cell {
    coordinates: IVec2,
    heuristic: f32,
}

impl Cell {
    pub fn new(coordinates: IVec2, cost: u32, goal: Cell) -> Self {
        let mut new = Self {
            coordinates,
            heuristic: 0.,
        };
        new.set_heuristic(cost, goal);
        new
    }

    pub fn coordinates(&self) -> IVec2 {
        self.coordinates
    }

    pub fn set_heuristic(&mut self, cost: u32, goal: Cell) {
        self.heuristic = cost as f32 + self.distance(goal);
    }

    pub fn distance(&self, other: Cell) -> f32 {
        self.coordinates
            .as_vec2()
            .distance(other.coordinates.as_vec2())
    }
}

impl From<Vec3> for Cell {
    fn from(position: Vec3) -> Self {
        Self {
            coordinates: position.round().as_ivec3().xz(),
            heuristic: 0.,
        }
    }
}

impl Eq for Cell {}

impl Ord for Cell {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).expect(
            "Used non-ordinary float value in cell value, which caused a comparison to fail",
        )
    }
}

impl PartialOrd for Cell {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.heuristic.partial_cmp(&self.heuristic)
    }
}

impl Into<IVec2> for Cell {
    fn into(self) -> IVec2 {
        self.coordinates
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0., 13., 0.).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_xyz(5., 10., 5.).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Ground plane
    commands.spawn((
        PbrBundle {
            transform: Transform::from_xyz(0., 0., 0.),
            mesh: meshes.add(shape::Plane::from_size(10.).into()),
            material: materials.add(Color::GREEN.into()),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(10., 0.1, 10.),
    ));

    let red_mat = materials.add(Color::RED.into());

    // Obstacle Wall
    commands.spawn((
        PbrBundle {
            transform: Transform::from_xyz(-1., 0., -1.).looking_at(Vec3::ZERO, Vec3::Y),
            mesh: meshes.add(shape::Box::new(4., 2., 1.).into()),
            material: red_mat.clone(),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(4., 2., 1.),
        Obstacle,
    ));

    let wall_box = meshes.add(shape::Box::new(10., 2., 1.).into());

    // Boundary Walls
    for position in [Vec3::X, Vec3::NEG_X, Vec3::Z, Vec3::NEG_Z] {
        commands.spawn((
            PbrBundle {
                transform: Transform::from_translation(position * 5.)
                    .looking_at(Vec3::ZERO, Vec3::Y),
                mesh: wall_box.clone(),
                material: red_mat.clone(),
                ..default()
            },
            RigidBody::Static,
            Collider::cuboid(10., 2., 1.),
            Obstacle,
        ));
    }

    // Goal spot
    commands.spawn((
        PbrBundle {
            transform: Transform::from_xyz(-4., 0., -4.),
            mesh: meshes.add(shape::Cylinder::default().into()),
            material: materials.add(Color::YELLOW.into()),
            ..default()
        },
        RigidBody::Static,
        Collider::cylinder(2., 0.5),
        Goal,
    ));

    // Player
    let mut player = commands.spawn((
        PbrBundle {
            transform: Transform::from_xyz(4., 0.1, 4.),
            mesh: meshes.add(
                shape::Icosphere {
                    radius: 0.5,
                    subdivisions: 2,
                }
                .try_into()
                .unwrap(),
            ),
            material: materials.add(Color::CYAN.into()),
            ..default()
        },
        RigidBody::Dynamic,
        Collider::ball(0.5),
        LinearVelocity::ZERO,
        Agent::default(),
    ));
}

fn move_agent(
    mut agent_query: Query<(&Transform, &mut LinearVelocity, &mut Agent)>,
    goal_query: Query<&Transform, With<Goal>>,
) {
    let (agent_transform, mut agent_velocity, mut agent) = agent_query.single_mut();
    let goal_transform = goal_query.single();

    if !agent.has_path() {
        agent.compute_path(agent_transform, goal_transform);
    }

    if let Some(next) = agent.next_point(agent_transform.translation) {
        let direction = (next - agent_transform.translation) * 2.;

        agent_velocity.x = direction.x;
        agent_velocity.z = direction.z;
    }
}
