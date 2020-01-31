use super::answers::*;

const CAPSULE_CABIN_COUNT: usize = 16;
const CAPSULE_MEASUREMENT_COUNT: usize = CAPSULE_CABIN_COUNT * 2;

struct Capsule {
    start_angle: f32,
    cabins: Vec<CabinMeasurement>,
}

struct CabinMeasurement {
    angle: f32,
    distance: f32,
}

pub struct CapsuledParser {
    prev_capsule: Option<Capsule>,
}

impl CapsuledParser {
    pub fn new() -> Self {
        Self { prev_capsule: None }
    }

    pub fn parse(
        &mut self,
        nodes: RplidarResponseCapsuleMeasurementNodes,
    ) -> Vec<RplidarResponseMeasurementNodeHq> {
        let capsule = self.parse_measurements(nodes);

        let prev_capsule = match &self.prev_capsule {
            Some(prev) => prev,
            None => {
                self.prev_capsule = Some(capsule);
                return Vec::new();
            }
        };

        let diff_angle = self.calc_diff_angle(capsule.start_angle, prev_capsule.start_angle);
        let angle_ratio = diff_angle / (CAPSULE_MEASUREMENT_COUNT as f32);

        let is_new_scan = self.is_new_scan(capsule.start_angle, prev_capsule.start_angle);

        let result = prev_capsule
            .cabins
            .iter()
            .enumerate()
            .map(|(index, cabin)| {
                let angle = prev_capsule.start_angle + (angle_ratio * index as f32) - cabin.angle;
                let dist = cabin.distance;
                let flag = (index == 0 && is_new_scan) as u8;

                RplidarResponseMeasurementNodeHq {
                    angle_z_q14: angle as u16,
                    dist_mm_q2: dist as u32,
                    quality: 0,
                    flag,
                }
            })
            .collect();

        self.prev_capsule = Some(capsule);

        result
    }

    fn parse_measurements(&self, capsule: RplidarResponseCapsuleMeasurementNodes) -> Capsule {
        // checksum stuff

        let start_angle = self.parse_start_angle(capsule.start_angle_sync_q6);

        let mut cabin_measurements = Vec::with_capacity(CAPSULE_MEASUREMENT_COUNT);
        for cabin in capsule.cabins.iter() {
            let dist1 = self.parse_cabin_dist(cabin.distance_angle_1);
            let dist2 = self.parse_cabin_dist(cabin.distance_angle_2);
            let (angle1, angle2) = self.parse_cabin_angles(cabin);

            cabin_measurements.push(CabinMeasurement {
                angle: angle1,
                distance: dist1,
            });

            cabin_measurements.push(CabinMeasurement {
                angle: angle2,
                distance: dist2,
            });
        }

        Capsule {
            start_angle,
            cabins: cabin_measurements,
        }
    }

    fn calc_diff_angle(&self, angle: f32, prev_angle: f32) -> f32 {
        (angle - prev_angle + 360.) % 360.
    }

    fn is_new_scan(&self, start_angle: f32, prev_angle: f32) -> bool {
        start_angle < prev_angle
    }

    fn parse_start_angle(&self, start_angle: u16) -> f32 {
        (start_angle & 0x7fff) as f32 / 64.0
    }

    fn parse_cabin_dist(&self, distance_angle: u16) -> f32 {
        // Remove 2 compensation angle bits
        ((distance_angle & 0xfffc) >> 2) as f32
    }

    fn parse_cabin_angles(&self, cabin: &RplidarResponseCabinNodes) -> (f32, f32) {
        let angle1 = (cabin.offset_angles_q3 & 0xf) as u16 | ((cabin.distance_angle_1 & 0x3) << 4);
        let angle2 = (cabin.offset_angles_q3 >> 4) as u16 | ((cabin.distance_angle_2 & 0x3) << 4);

        // divide angles by 8 here?
        // Convert to i32 first (because the angles are signed) and then to float
        ((angle1 as i32) as f32, (angle2 as i32) as f32)
    }
}
