//! Module containing transformation functions from the python library [`transformations`][https://pypi.org/project/transformations]
//!
//! This module extends on [`glam`] and adds a few additional functions for working with matrices and euler angles.
//!
//! References:
//!
//! - PyPI: https://pypi.org/project/transformations
//! - GitHub: https://github.com/cgohlke/transformations
//! - Docs: https://docs.ros.org/en/jade/api/tf/html/python/transformations.html
use std::fmt;

use glam::EulerRot;

#[cfg(test)]
pub use test_utils::*;

#[derive(Debug, Clone, Copy)]
pub enum Axes {
    Static(EulerRot),
    Rotational(EulerRot),
}

impl fmt::Display for Axes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (frame, euler) = match self {
            Axes::Static(euler) => ('s', euler),
            Axes::Rotational(euler) => ('r', euler),
        };

        #[allow(deprecated)]
        let euler = match euler {
            EulerRot::ZYX => "zyx",
            EulerRot::ZXY => "zxy",
            EulerRot::YXZ => "yxz",
            EulerRot::YZX => "yzx",
            EulerRot::XYZ => "xyz",
            EulerRot::XZY => "xzy",
            EulerRot::ZYZ => "zyz",
            EulerRot::ZXZ => "zxz",
            EulerRot::YXY => "yxy",
            EulerRot::YZY => "yzy",
            EulerRot::XYX => "xyx",
            EulerRot::XZX => "xzx",
        };

        f.write_fmt(format_args!("{frame}{euler}"))
    }
}

/// `transformation` switches the places of ai & ak when it's using rotational frame
/// This is why for the static frame we need this helper trait.
/// It will __reverse__ the Euler rotation and will result in correct results when we switch the `ai` & `ak` params.
pub(crate) trait ToStatic {
    fn to_static(&self) -> EulerRot;
}

impl ToStatic for EulerRot {
    fn to_static(&self) -> EulerRot {
        match self {
            EulerRot::ZYX => EulerRot::XYZ,
            EulerRot::ZXY => EulerRot::YXZ,
            EulerRot::YXZ => EulerRot::ZXY,
            EulerRot::YZX => EulerRot::XZY,
            EulerRot::XYZ => EulerRot::ZYX,
            EulerRot::XZY => EulerRot::YZX,
            // two axis rotations are kept the same
            #[allow(deprecated)]
            two_axis_rotations @ EulerRot::ZYZ
            | two_axis_rotations @ EulerRot::ZXZ
            | two_axis_rotations @ EulerRot::YXY
            | two_axis_rotations @ EulerRot::XYX
            | two_axis_rotations @ EulerRot::XZX
            | two_axis_rotations @ EulerRot::YZY => *two_axis_rotations,
        }
    }
}

#[cfg(test)]
mod test_utils {
    use once_cell::sync::Lazy;

    /// Static of the Python context we use for both [`f32::euler_matrix`] and [`f64::euler_matrix`] implementations
    pub static EULER_MATRIX_CONTEXT: Lazy<inline_python::Context> = Lazy::new(|| {
        inline_python::python! {
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
        }
    });

    /// Static of the Python context we use for both [`f32::euler_from_matrix`] and [`f64::euler_from_matrix`] implementations
    pub static EULER_FROM_MATRIX_CONTEXT: Lazy<inline_python::Context> = Lazy::new(|| {
        inline_python::python! {
            import transformations as tf
            import math as m

            # Angles are in radians unless specified otherwise.
            # ...
            # "sxyz" is the default axis sequence
            # ai, aj, ak : Euler s roll, pitch and yaw angles axes
            # https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_matrix

            matrix_static = tf.euler_matrix(1.0, 2.0, 3.0, "syxz")
            static_al, static_be, static_ga = tf.euler_from_matrix(matrix_static, "syxz")

            matrix_rotational = tf.euler_matrix(1.0, 2.0, 3.0, "ryxz")
            rotational_al, rotational_be, rotational_ga = tf.euler_from_matrix(matrix_rotational, "ryxz")
        }
    });
}

pub mod f32 {
    use glam::{Mat4, Quat};

    use super::{Axes, ToStatic};

    /// transformations.euler_matrix equivalent for [`f32`]
    ///
    /// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1083-L1143)
    ///
    pub fn euler_matrix(ai: f32, aj: f32, ak: f32, axes: Axes) -> Mat4 {
        match axes {
            Axes::Static(euler_rot) => {
                // switch the places of ai & ak
                // make the euler rotation statically correct
                // switches first and last axes of the sequence
                Mat4::from_euler(euler_rot.to_static(), ak, aj, ai)
            }
            Axes::Rotational(euler_rot) => Mat4::from_euler(euler_rot, ai, aj, ak),
        }
    }

    /// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1146-L1201)
    ///
    /// [Python documentation](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_matrix)
    pub fn euler_from_matrix(matrix: Mat4, axes: Axes) -> (f32, f32, f32) {
        match axes {
            Axes::Static(euler_static) => {
                let (ak, aj, ai) = Quat::from_mat4(&matrix).to_euler(euler_static.to_static());

                (ai, aj, ak)
            }
            Axes::Rotational(euler_rotational) => {
                Quat::from_mat4(&matrix).to_euler(euler_rotational)
            }
        }
    }

    #[cfg(test)]
    mod test {
        use glam::{EulerRot, Mat4};

        use super::{euler_matrix, Axes};
        use crate::{matrices::FromPy, transformations::EULER_MATRIX_CONTEXT};

        #[test]
        fn test_euler_matrix() {
            let context = &EULER_MATRIX_CONTEXT;

            assert_eq!(std::f32::consts::PI, context.get::<f32>("math_pi"));

            // Rotational
            {
                let euler_matrix_rotational = Mat4::try_get(&context, "euler_matrix_rotational");

                let actual_rotational =
                    euler_matrix(1.0, 2.0, 3.0, Axes::Rotational(EulerRot::XYZ));

                // very close, but not equal due to float numbers
                // TODO: Check if this is the right value for abs. diff for floating number comparison
                assert!(
                    actual_rotational.abs_diff_eq(euler_matrix_rotational, 0.000_000_2),
                    "Glam uses the rotational frame from transformation by default"
                );
            }

            // Static
            {
                let euler_matrix_static = Mat4::try_get(&context, "euler_matrix_static");

                let actual_static =
                    euler_matrix(1.0_f32, 2.0_f32, 3.0_f32, Axes::Static(EulerRot::XYZ));

                // pretty_assertions::assert_eq!(actual_static.as_dmat4(), euler_matrix_static.as_dmat4());

                // very close, but not equal due to float numbers
                // TODO: Check if this is the right value for abs. diff for floating number comparison
                assert!(
                    actual_static.abs_diff_eq(euler_matrix_static, 0.000_000_2),
                    "Should match static frame result from transformation"
                );
            }
        }
    }
}

pub mod f64 {
    use glam::{DMat4, DQuat};

    use super::{Axes, ToStatic};

    /// transformations.euler_matrix equivalent
    ///
    /// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1083-L1143)
    ///
    /// [Python documentation](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_matrix)
    pub fn euler_matrix(ai: f64, aj: f64, ak: f64, axes: Axes) -> DMat4 {
        match axes {
            Axes::Static(euler_rot) => {
                // switch the places of ai & ak
                // make the euler rotation statically correct
                // switches first and last axes of the sequence
                DMat4::from_euler(euler_rot.to_static(), ak, aj, ai)
            }
            Axes::Rotational(euler_rot) => DMat4::from_euler(euler_rot, ai, aj, ak),
        }
    }

    /// [Python implementation](https://github.com/cgohlke/transformations/blob/deb1a195dab70f0f36365a104f9b70505e37b473/transformations/transformations.py#L1146-L1201)
    ///
    /// [Python documentation](https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_matrix)
    pub fn euler_from_matrix(matrix: DMat4, axes: Axes) -> (f64, f64, f64) {
        match axes {
            Axes::Static(euler_static) => {
                let (ak, aj, ai) = DQuat::from_mat4(&matrix).to_euler(euler_static.to_static());

                (ai, aj, ak)
            }
            Axes::Rotational(euler_rotational) => {
                DQuat::from_mat4(&matrix).to_euler(euler_rotational)
            }
        }
    }

    #[cfg(test)]
    mod test {
        use glam::{DMat4, DVec4, EulerRot};

        use super::{euler_matrix, Axes};
        use crate::{
            matrices::FromPy,
            transformations::{
                f64::euler_from_matrix, EULER_FROM_MATRIX_CONTEXT, EULER_MATRIX_CONTEXT,
            },
        };

        #[test]
        fn test_euler_matrix() {
            let context = &EULER_MATRIX_CONTEXT;

            assert_eq!(std::f64::consts::PI, context.get::<f64>("math_pi"));

            // Rotational
            {
                let euler_matrix_rotational = DMat4::try_get(&context, "euler_matrix_rotational");

                let actual_rotational =
                    euler_matrix(1.0, 2.0, 3.0, Axes::Rotational(EulerRot::XYZ));

                // very close, but not equal due to float numbers
                // TODO: Check if this is the right value for abs. diff for floating number comparison
                assert!(
                    actual_rotational.abs_diff_eq(euler_matrix_rotational, 0.000_000_000_000_000_5),
                    "Glam uses the rotational frame from transformation by default"
                );
            }

            // Static
            {
                let euler_matrix_static = DMat4::try_get(&context, "euler_matrix_static");

                let actual_static =
                    euler_matrix(1.0_f64, 2.0_f64, 3.0_f64, Axes::Static(EulerRot::XYZ));

                // very close, but not equal due to float numbers
                // TODO: Check if this is the right value for abs. diff for floating number comparison
                assert!(
                    actual_static.abs_diff_eq(euler_matrix_static, 0.000_000_000_000_000_3),
                    "Should match static frame result from transformation"
                );
            }
        }

        #[test]
        fn test_euler_from_matrix() {
            let context = &EULER_FROM_MATRIX_CONTEXT;

            // Static
            {
                // take from python result
                // using Vecs and axis!
                let matrix_static = DMat4::from_cols(
                    DVec4::new(
                        -0.642872836134547,
                        -0.6812427202564033,
                        0.35017548837401463,
                        0.0,
                    ),
                    // y
                    DVec4::new(
                        0.05872664492762098,
                        0.411982245665683,
                        0.9092974268256817,
                        0.0,
                    ),
                    // z
                    DVec4::new(
                        -0.7637183366502791,
                        0.6051272472413688,
                        -0.2248450953661529,
                        0.0,
                    ),
                    // w
                    DVec4::new(0.0, 0.0, 0.0, 1.0),
                );

                // double check the matrices used in python
                {
                    let python_matrix_static = DMat4::try_get(&context, "matrix_static");

                    pretty_assertions::assert_eq!(matrix_static, python_matrix_static);

                    assert!(
                        matrix_static.abs_diff_eq(python_matrix_static, 0.000_000_000_000_000_3),
                        "Should match static frame used in transformation python code"
                    );
                }

                let (al_expected, be_expected, ga_expected) = (
                    context.get::<f64>("static_al"),
                    context.get::<f64>("static_be"),
                    context.get::<f64>("static_ga"),
                );

                let (al, be, ga) = euler_from_matrix(matrix_static, Axes::Static(EulerRot::YXZ));

                let actual_matrix = euler_matrix(al, be, ga, Axes::Static(EulerRot::YXZ));

                assert!(
                    actual_matrix.abs_diff_eq(matrix_static, 0.000_000_000_000_000_2),
                    "Actual matrix resulting from al, be, ga should match with the original matrix"
                );

                approx::assert_abs_diff_eq!(al_expected, al);
                approx::assert_abs_diff_eq!(be_expected, be);
                approx::assert_abs_diff_eq!(ga_expected, ga);
            }

            // Rotational
            {
                // take from python result
                // using Vecs and axis!
                let matrix_rotational = DMat4::from_cols(
                    DVec4::new(
                        -0.42691762127620736,
                        -0.05872664492762098,
                        0.9023815854833308,
                        0.0,
                    ),
                    // y
                    DVec4::new(
                        -0.8337376517741568,
                        0.411982245665683,
                        -0.36763046292489926,
                        0.0,
                    ),
                    // z
                    DVec4::new(
                        -0.35017548837401463,
                        -0.9092974268256817,
                        -0.2248450953661529,
                        0.0,
                    ),
                    // w
                    DVec4::new(0.0, 0.0, 0.0, 1.0),
                );

                // double check the matrices used in python
                {
                    let python_matrix_rotational = DMat4::try_get(&context, "matrix_rotational");

                    assert!(
                        matrix_rotational
                            .abs_diff_eq(python_matrix_rotational, 0.000_000_000_000_000_000_001),
                        "Should match rotational frame used in transformation python code"
                    );
                }

                let (al_expected, be_expected, ga_expected) = (
                    context.get::<f64>("rotational_al"),
                    context.get::<f64>("rotational_be"),
                    context.get::<f64>("rotational_ga"),
                );

                let (al, be, ga) =
                    euler_from_matrix(matrix_rotational, Axes::Rotational(EulerRot::YXZ));

                let actual_matrix = euler_matrix(al, be, ga, Axes::Rotational(EulerRot::YXZ));

                assert!(
                    actual_matrix.abs_diff_eq(matrix_rotational, 0.000_000_000_000_000_9),
                    "Actual matrix resulting from al, be, ga should match with the original matrix"
                );

                approx::assert_relative_eq!(al_expected, al, max_relative = 0.000_000_000_000_001);
                approx::assert_relative_eq!(be_expected, be, max_relative = 0.000_000_000_000_001);
                approx::assert_relative_eq!(ga_expected, ga, max_relative = 0.000_000_000_000_001);
            }
        }
    }
}
