function q = euler_to_quaternion(phi, theta, psi)
    cy = cos(psi * 0.5);
    sy = sin(psi * 0.5);
    cp = cos(theta * 0.5);
    sp = sin(theta * 0.5);
    cr = cos(phi * 0.5);
    sr = sin(phi * 0.5);

    q = [cr * cp * cy + sr * sp * sy;
         sr * cp * cy - cr * sp * sy;
         cr * sp * cy + sr * cp * sy;
         cr * cp * sy - sr * sp * cy];
end