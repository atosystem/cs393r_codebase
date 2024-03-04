map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees


-- Initialize()
M_PI = 3.14159265358979323846264338327950288;
x_std = 0.0; -- [0, 1] with step = 0.1
y_std = 0.0; -- [0, 1] with step = 0.1
r_std = 0.0; -- [0, M_PI/2] with step = M_PI / 18 (10 degrees)

-- Predict()
-- trans error from trans model
k1 = 0.1; -- [0.1, 1] with step = 0.1
-- trans error from rotat model
k2 = 0.1; -- [0.01, 0.1] with step = 0.01
-- rotat error from trans model
k3 = 0.1; -- [0.01, 0.1] with step = 0.01
-- rotat error from rotat model
k4 = 0.1; -- [0.1, 1] with step = 0.1

--Update()
sigma_s = 1.0; -- [1.0, 3.0] with step = 0.5
gamma_pow = -1; -- [-1.0, 0.0] with step = 0.1
d_short_d_long = 1.0; -- (0.5, 1, 3, 5)
