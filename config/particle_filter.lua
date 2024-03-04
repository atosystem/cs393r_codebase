map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees


-- Initialize()
M_PI = 3.14159265358979323846264338327950288;
x_std = 0.5;
y_std = 0.5;
r_std = 0.0;

-- Predict()
-- trans error from trans model
k1 = 0.5;
-- trans error from rotat model
k2 = 0.1;
-- rotat error from trans model
k3 = 0.1;
-- rotat error from rotat model
k4 = 0.5;

--Update()
sigma_s = 2;
gamma_pow = -1;
d_short_d_long = 0.5;
