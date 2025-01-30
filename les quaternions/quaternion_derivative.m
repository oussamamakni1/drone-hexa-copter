function q_dot = quaternion_derivative(q, Omega)
    Omega_quat = [0; Omega];
    q_dot = 0.5 * quatmultiply(q', Omega_quat');
end