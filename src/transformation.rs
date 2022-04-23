use glam::{DMat4, EulerRot, Mat4};


/// transformations.euler_matrix equivalent
///
/// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1083-L1143)
///
/// [Python documentation](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_matrix)
pub fn euler_matrix(ai: f64, aj: f64, ak: f64, axes: Axes) -> DMat4 {
    match axes {
        Axes::Static(_) => todo!("We need static for our implementation"),
        Axes::Rotational(euler_rot) => DMat4::from_euler(euler_rot, ai, aj, ak),
    }
}

/// transformations.euler_matrix equivalent
///
/// https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1083-L1143
///
pub fn euler_matrix_f32(ai: f32, aj: f32, ak: f32, axes: Axes) -> Mat4 {
    match axes {
        Axes::Static(_) => todo!("We need static for our implementation"),
        Axes::Rotational(euler_rot) => Mat4::from_euler(euler_rot, ai, aj, ak),
    }
}

/// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1146-L1201)
///
/// [Python documentation](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_matrix)
pub fn euler_from_matrix(_matrix: DMat4, _axes: Axes) -> (f64, f64, f64) {
    todo!("Implement for static, as we need static for our implementation")
}

pub enum Axes {
    Static(EulerRot),
    Rotational(EulerRot),
}

#[cfg(test)]
mod test {
    use glam::{DMat4, DVec3, EulerRot, Mat4};
    use inline_python::{python, Context};

    use super::{Axes, euler_matrix};
    use crate::{ util::FromRow, transformation::euler_matrix_f32};

    #[test]
    fn test_euler_matrix() {
        let context: Context = python! {
            import transformations as tf
            import math as m

            # Angles are in radians unless specified otherwise.
            # ...
            # "sxyz" is the default axis sequence
            # ai, aj, ak : Euler s roll, pitch and yaw angles axes
            # https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_matrix
            # euler_matrix = tf.euler_matrix(m.pi / 4, 0, 0, "sxyz")
            math_pi = m.pi;
            euler_matrix_static = tf.euler_matrix(1.0, 2.0, 3.0, "sxyz")
            euler_matrix_rotational = tf.euler_matrix(1.0, 2.0, 3.0, "rxyz")
        };

        assert_eq!(std::f64::consts::PI, context.get::<f64>("math_pi"));

        let euler_matrix_static = {
            let python = &context.get::<[[f64; 4]; 4]>("euler_matrix_static");

            DMat4::from_rows_array_2d(&python)
        };
        
        let euler_matrix_rotational_f32 = {
            let python = &context.get::<[[f32; 4]; 4]>("euler_matrix_rotational");

            Mat4::from_rows_array_2d(&python)
        };

        let euler_matrix_rotational = {
            let python = &context.get::<[[f64; 4]; 4]>("euler_matrix_rotational");

            DMat4::from_rows_array_2d(&python)
        };

        
        let actual_rotational_f32 = euler_matrix_f32(1.0, 2.0, 3.0, Axes::Rotational(EulerRot::XYZ));
        
        // assert_relative_eq!(actual_rotational_f32,epsilon = f64::EPSILON);

        // very close, but not equal due to float numbers
        pretty_assertions::assert_eq!(
            actual_rotational_f32,
            euler_matrix_rotational_f32,
            "Glam uses rotational from transformation by default"
        );
        
        // very close, but not equal due to float numbers
        pretty_assertions::assert_eq!(
            euler_matrix(1.0_f64, 2.0_f64, 3.0_f64, Axes::Rotational(EulerRot::XYZ)),
            euler_matrix_rotational,
            "Glam uses rotational from transformation by default"
        );

        pretty_assertions::assert_eq!(
            DMat4::from_scale(DVec3::new(1.0, 2.0, 3.0)),
            euler_matrix_rotational,
            "does not match with from_scale"
        );

    }
}
