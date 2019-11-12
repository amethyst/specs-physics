use amethyst::{
    controls::{HideCursor, WindowFocus},
    core::Transform,
    derive::SystemDesc,
    ecs::prelude::*,
    input::{get_input_axis_simple, InputHandler, StringBindings},
    renderer::ActiveCamera,
    shrev::{EventChannel, ReaderId},
    winit::{DeviceEvent, Event},
};
use specs_physics::{
    BodyComponent,
    nalgebra::Unit,
    nphysics::math::{Vector, Velocity},
    world::GeometricalWorldRes,
};

#[derive(Default, SystemDesc)]
#[system_desc(name(CollisionDetectionSystemDesc))]
pub struct CollisionDetectionSystem;

// Here is a system that can perform additional logic when a collision is detected
impl<'a> System<'a> for CollisionDetectionSystem {
    type SystemData = ReadExpect<'a, GeometricalWorldRes<f32>>;

    fn run(&mut self, geo_world: Self::SystemData) {
        for contact in geo_world.contact_events() {
            println!("CONTACT = {:?}", contact);
        }
    }
}

#[derive(SystemDesc)]
#[system_desc(name(CameraMovementSystemDesc))]
pub struct CameraMovementSystem {
    speed: f32,
}

// Here is a system that sets the velocity of the camera's RigidBody component depending on user input
impl<'a> System<'a> for CameraMovementSystem {
    type SystemData = (
        Read<'a, ActiveCamera>,
        WriteStorage<'a, BodyComponent<f32>>,
        Read<'a, InputHandler<StringBindings>>,
    );

    fn run(&mut self, (active_camera, mut bodies, input): Self::SystemData) {
        let x = get_input_axis_simple(&Some(String::from("move_x")), &input);
        let y = get_input_axis_simple(&Some(String::from("move_y")), &input);
        let z = get_input_axis_simple(&Some(String::from("move_z")), &input);

        if let Some(camera) = active_camera.entity {
            if let Some(body) = bodies.get_mut(camera) {
                let b = body.as_rigid_body_mut().expect("Camera must be RigidBody");
                // Sets the velocity local to the object's orientation if WASD is pressed
                if let Some(dir) = Unit::try_new(Vector::new(x, y, z), 1.0e-6) {
                    b.set_linear_velocity(b.position().rotation * dir.as_ref() * self.speed);
                } else {
                    b.set_velocity(Velocity::zero());
                }
            }
        }
    }
}

#[derive(SystemDesc)]
#[system_desc(name(CameraRotationSystemDesc))]
pub struct CameraRotationSystem {
    sensitivity_x: f32,
    sensitivity_y: f32,
    #[system_desc(event_channel_reader)]
    event_reader: ReaderId<Event>,
}

// This system updates the camera's orientation using mouse motion as input
impl<'a> System<'a> for CameraRotationSystem {
    type SystemData = (
        Read<'a, ActiveCamera>,
        WriteStorage<'a, BodyComponent<f32>>,
        Read<'a, EventChannel<Event>>,
        Read<'a, HideCursor>,
        Read<'a, WindowFocus>,
    );

    fn run(&mut self, (active_camera, mut bodies, event_channel, hide, window_focus): Self::SystemData) {
        for event in event_channel.read(&mut self.event_reader) {
            if window_focus.is_focused && hide.hide {
                if let Event::DeviceEvent { ref event, .. } = *event {
                    if let DeviceEvent::MouseMotion { delta: (x, y) } = *event {
                        if let Some(entity) = active_camera.entity {
                            if let Some(body) = bodies.get_mut(entity) {
                                let body = body
                                    .as_rigid_body_mut()
                                    .expect("Active camera must be RigidBody");
                                // Update camera rotation using cleaner `Tansform` api instead of manually applying maths to `Isometry`
                                body.set_position({
                                    let isometry = body.position();
                                    let mut transform = Transform::new(isometry.translation, isometry.rotation, Vector::zeros());
                                    transform.append_rotation_x_axis((-y as f32 * self.sensitivity_y).to_radians());
                                    transform.prepend_rotation(Vector::y_axis(), (-x as f32 * self.sensitivity_x).to_radians());
                                    *transform.isometry()
                                });
                            }
                        }
                    }
                }
            }
        }
    }
}