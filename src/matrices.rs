use glam::{DMat4, Mat4};

/// From row major
pub trait FromRow<T> {
    // from_cols_array_2d
    fn from_rows_array_2d(m: &[[T; 4]; 4]) -> Self;
    
    fn to_rows_array_2d(&self) -> [[T; 4]; 4];
}

// From `InnerF32`, however, glem does not expose this type
impl FromRow<f32> for Mat4 {
    /// Default method of `numpy` expects row-major
    /// Transposes the [`Mat4::from_cols_array_2d()`] call to get the row-major.
    fn from_rows_array_2d(m: &[[f32; 4]; 4]) -> Self {
        Self::from_cols_array_2d(m).transpose()
    }
    
    /// Default method of `numpy` expects row-major
    /// Transposes and then calls the [`Mat4::to_cols_array_2d()`] to get the row-major.
    fn to_rows_array_2d(&self) -> [[f32; 4]; 4] {
        self.transpose().to_cols_array_2d()
    }
}

// From `InnerF32`, however, glem does not expose this type
impl FromRow<f64> for DMat4 {
    /// Default method of `numpy` expects row-major
    /// Transposes the [`DMat4::from_cols_array_2d()`] call to get the row-major.
    fn from_rows_array_2d(m: &[[f64; 4]; 4]) -> Self {
        Self::from_cols_array_2d(m).transpose()
    }

    /// Default method of `numpy` expects row-major
    /// Transposes and then calls the [`DMat4::to_cols_array_2d()`] to get the row-major.
    fn to_rows_array_2d(&self) -> [[f64; 4]; 4] {
        self.transpose().to_cols_array_2d()
    }
}

#[cfg(test)]
mod test {
    use crate::util::FromRow;
    use glam::{Mat4, Vec4, EulerRot, DMat4, DVec3, DMat3};
    use inline_python::{python, Context};

    #[test]
    fn test_against_dot_matrix_from_numpy_in_python() {
        let context: Context = python! {
            import numpy as np

            dot_result = np.array([
                [1.0, 2.0, 3.0, 4.0],
                [5.0, 6.0, 7.0, 8.0],
                [9.0, 10.0, 11.0, 12.0],
                [13.0, 14.0, 15.0, 16.0],
            ]).dot(np.array([
                [17.0, 18.0, 19.0, 20.0],
                [21.0, 22.0, 23.0, 24.0],
                [25.0, 26.0, 27.0, 28.0],
                [29.0, 30.0, 31.0, 32.0],
            ]))
        };

        // Python uses row-major for numpy.array & dot
        let py_result = Mat4::from_rows_array_2d(&context.get::<[[f32; 4]; 4]>("dot_result"));

        let m4_1 = Mat4::from_rows_array_2d(&[
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        let m4_2 = Mat4::from_rows_array_2d(&[
            [17.0, 18.0, 19.0, 20.0],
            [21.0, 22.0, 23.0, 24.0],
            [25.0, 26.0, 27.0, 28.0],
            [29.0, 30.0, 31.0, 32.0],
        ]);

        let expected = {
            let x_axis = Vec4::new(250.0, 618.0, 986.0, 1354.0);
            let y_axis = Vec4::new(260.0, 644.0, 1028.0, 1412.0);
            let z_axis = Vec4::new(270.0, 670.0, 1070.0, 1470.0);
            let w_axis = Vec4::new(280.0, 696.0, 1112.0, 1528.0);

            Mat4::from_cols(x_axis, y_axis, z_axis, w_axis)
        };

        assert_eq!(expected, py_result);
        assert_eq!(py_result, m4_1.mul_mat4(&m4_2),);
    }

}
