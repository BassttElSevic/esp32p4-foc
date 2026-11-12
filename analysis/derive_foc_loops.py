"""Derive the ESP32-P4 FOC current and speed loops at aligned, unloaded rest.

Run with Python 3 and SymPy: python analysis/derive_foc_loops.py
An alternative esp32p4_foc_data.m can be passed as the positional argument.
Currents are in A, speeds in electrical rad/s. The sampled plant uses the
model's RK4 step, with averaged PWM and smoothed ADC/PWM/encoder quantizers.
"""

import argparse
import ast
import operator
from pathlib import Path
import re

import sympy as sp


DEFAULT_DATA = Path(__file__).resolve().parents[1] / "simulink" / "esp32p4_foc_data.m"
s, z = sp.symbols("s z")


def read_scalars(path):
    """Read scalar arithmetic from the MATLAB model workspace; skip vectors."""
    values = {"pi": sp.pi}
    binary = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
              ast.Div: operator.truediv, ast.Pow: operator.pow}

    def scalar(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return sp.Rational(str(node.value))
        if isinstance(node, ast.Name):
            return values[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in binary:
            return binary[type(node.op)](scalar(node.left), scalar(node.right))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            return scalar(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "sqrt" and len(node.args) == 1 and not node.keywords):
            return sp.sqrt(scalar(node.args[0]))
        raise ValueError("Unsupported MATLAB scalar expression")

    for line in path.read_text(encoding="utf-8-sig").splitlines():
        line = line.partition("%")[0].strip()
        match = re.fullmatch(r"(\w+)\s*=\s*(.+)\s*;", line)
        if match and not match[2].startswith("["):
            expression = match[2].replace("^", "**")
            try:
                values[match[1]] = scalar(ast.parse(expression, mode="eval").body)
            except (ValueError, KeyError, SyntaxError) as error:
                raise ValueError(f"Could not read parameter {match[1]} from {path}") from error
    return values


def continuous_loops():
    # E = Lq*s + R, M = J*s + B, g = p*Kt, K = PWM volts per unit.
    E, M, g, K, psi = sp.symbols("E M g K psi")
    ci, cw, hi, hw = sp.symbols("C_i C_w H_i H_w")
    i, w, u, r_i, r_w = sp.symbols("i_q omega_e u i_ref w_ref")
    winding = E*i + psi*w - K*u
    shaft = M*w - g*i
    control = u - ci*(r_i - hi*i)
    lock = sp.solve([winding.subs(w, 0), control], [i, u], dict=True)[0]
    inner = sp.solve([winding, shaft, control], [i, w, u], dict=True)[0]
    outer = sp.solve([winding, shaft, control, r_i - cw*(r_w - hw*w)],
                     [i, w, u, r_i], dict=True)[0]
    speed_plant = sp.cancel(inner[w]/r_i)
    speed_loop = sp.cancel(outer[w]/r_w)
    assert sp.cancel(speed_loop - cw*speed_plant/(1 + hw*cw*speed_plant)) == 0
    return {
        "i/i_ref, fixed speed": sp.factor(lock[i]/r_i),
        "i/i_ref, free rotor": sp.factor(inner[i]/r_i),
        "omega_e/i_ref, inner closed": speed_plant,
        "omega_e/w_ref, both closed": speed_loop,
    }


def sampled_loops():
    # Pi, Pw, Pm: normalized q command to true current, true speed, estimated speed.
    pi, pw, pm = sp.symbols("P_i P_w P_m")
    ci, cw, hi = sp.symbols("C_i C_w H_i")
    u, i, w, wm, ri, rw = sp.symbols("u i w w_hat i_ref w_ref")
    equations = [i - pi*u, w - pw*u, wm - pm*u, u - ci*(ri - hi*i)]
    inner = sp.solve(equations, [u, i, w, wm], dict=True)[0]
    outer = sp.solve(equations + [ri - cw*(rw - wm)], [u, i, w, wm, ri], dict=True)[0]
    return {
        "i/i_ref, free rotor": sp.cancel(inner[i]/ri),
        "i_filtered/i_ref, free rotor": sp.cancel(hi*inner[i]/ri),
        "omega_e/i_ref, inner closed": sp.cancel(inner[w]/ri),
        "outer return ratio": sp.cancel(cw*inner[wm]/ri),
        "omega_e/w_ref, both closed": sp.cancel(outer[w]/rw),
        "omega_est/w_ref, both closed": sp.cancel(outer[wm]/rw),
    }, (pi, pw, pm, ci, cw, hi)


def motor_jacobian(p, psi, r, ld, lq, inertia, friction, k):
    # Express build_motor.m in electrical angle and speed, preserving order of
    # linearization: the q equation contains an EMF term even at zero speed.
    id_, iq, speed, angle, ud, uq = sp.symbols("i_d i_q w_e theta_e u_d u_q")
    torque = sp.Rational(3, 2)*p*(psi + (ld-lq)*id_)*iq
    rhs = sp.Matrix([(k*ud-r*id_+speed*lq*iq)/ld,
                     (k*uq-r*iq-speed*(ld*id_+psi))/lq,
                     (p*torque-friction*speed)/inertia, speed])
    variables = (id_, iq, speed, angle, ud, uq)
    jac = rhs.jacobian(variables).subs(dict.fromkeys(variables, 0))
    # d current decouples at this point. Append duq/dt = 0 for the hold.
    return jac.extract([1, 2, 3], [1, 2, 3, 5]).col_join(sp.zeros(1, 4))


def rk4_transition(generator, step, steps):
    # Rationalize only the final matrix, keeping polynomial cancellation exact.
    generator_step = (step*generator).evalf(65)
    identity = sp.eye(generator.rows)
    one_step = sum((generator_step**j / sp.factorial(j) for j in range(1, 5)), identity)
    return (one_step**steps).applyfunc(lambda v: sp.Rational(str(v.evalf(50))))


def numerical_loops(data, formulas, symbols):
    p, psi, r = data["p_pairs"], data["psi_pm"], data["R_s"]
    lq, ld, j, b = (data[k] for k in ("L_q", "L_d", "J_m", "B_m"))
    ts, h = data["Ts_ctrl"], data["Ts_plant"]
    n_steps = ts / h
    if n_steps.is_integer is not True or n_steps < 1:
        raise ValueError("Ts_ctrl must be a positive integer multiple of Ts_plant")
    if data["pwm_switching"] != 0 or data["enc_ok"] != 1 or data["id_target_mA"] != 0:
        raise ValueError("This operating point assumes averaged PWM, valid encoder reads and id_ref=0")
    if data["b_u_mV"] < data["unipolar_threshold_mV"] and data["b_v_mV"] < data["unipolar_threshold_mV"]:
        raise ValueError("The model's unipolar current clamp is active at this operating point")

    # build_control.m uses clamped dt for PI and microsecond-rounded dt for AS5600.
    dt_pi = min(ts, data["ctrl_dt_max_s"])
    dt_us = sp.floor(ts * 10**6 + sp.Rational(1, 2))
    if not data["enc_dt_min_us"] < dt_us < data["enc_dt_max_us"]:
        raise ValueError("The encoder estimator would skip updates with this sample period")
    dt_encoder = dt_us / 10**6
    levels = 2**data["pwm_bits"]
    k = data["V_dc"] * (levels-1)/levels
    kt = sp.Rational(3, 2)*p*psi
    aip, aii = 1000*data["Kp_i"], 1000*data["Ki_i"]
    awp, awi = data["Kp_w"]/1000, data["Ki_w"]/1000
    ci = aip + aii*dt_pi/(z-1)
    cw = data["torque_sign"]*data["s_enc"]*(awp + awi*dt_pi/(z-1))
    ai, aw = data["alpha_i"], data["alpha_w"]
    hi = data["sense_wiring_sign"]*data["sense_sign"] * ai*z/(z-1+ai)
    hw = aw*z/(z-1+aw)

    # Augment [iq, omega_e, theta_e] with the normalized held voltage command.
    generator = motor_jacobian(p, psi, r, ld, lq, j, b, k)
    update = rk4_transition(generator, h, int(n_steps))
    phi, gamma = update[:2, :2], update[:2, 3]
    iq_omega = (z*sp.eye(2)-phi).inv() * gamma
    pi, pw = map(sp.cancel, iq_omega)
    # Previous interval's angle increment, observed at its END: one factor 1/z.
    delta_theta = (update[2:3, :2]*iq_omega)[0] + update[2, 3]
    pm = sp.cancel(data["s_enc"]*hw*delta_theta/(z*dt_encoder))
    substitutions = dict(zip(symbols, (pi, pw, pm, ci, cw, hi)))
    result = {name: sp.cancel(tf.xreplace(substitutions)) for name, tf in formulas.items()}

    # Compare the scalar fixed-speed channel and the free-rotor q channel.
    for label, inductance in (("d at rest", ld), ("q, fixed speed", lq)):
        decay = rk4_transition(sp.Matrix([[-r/inductance]]), h, int(n_steps))[0]
        plant = k*(1-decay)/(r*(z-decay))
        result[f"i/i_ref, {label}"] = sp.cancel(ci*plant/(1+ci*plant*hi))

    continuous_values = dict(zip(sp.symbols("E M g K psi C_i C_w H_i H_w"),
                                 (r+lq*s, j*s+b, p*kt, k, psi, aip+aii/s,
                                  data["torque_sign"]*data["s_enc"]*(awp+awi/s),
                                  data["sense_sign"]*data["sense_wiring_sign"], data["s_enc"])))
    return result, continuous_values, k, kt


def describe(name, tf, variable, show_poles=False):
    numerator, denominator = [sp.Poly(term, variable) for term in sp.fraction(tf)]
    print(f"\n{name}\n  descending powers of {variable}, normalized denominator")
    for label, poly in (("num", numerator), ("den", denominator)):
        coeffs = [float(value/denominator.LC()) for value in poly.all_coeffs()]
        print(f"  {label} = [{', '.join(f'{value:.14g}' for value in coeffs)}]")
    if show_poles:
        roots = denominator.nroots(n=16, maxsteps=1000)
        print("  poles =", ", ".join(str(root.evalf(10)) for root in roots))
        print("  DC gain =", sp.limit(tf, variable, 1).evalf(12))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path, nargs="?", default=DEFAULT_DATA)
    options = parser.parse_args()
    data = read_scalars(options.data)
    print("Model data:", options.data)
    print("s domain: E=R+Lq*s, M=J*s+B, g=p*Kt, K=PWM gain. Zero load disturbance.")
    continuous = continuous_loops()
    for name, tf in continuous.items():
        print(f"  {name}: {tf}")
    formulas, symbols = sampled_loops()
    print("\nz domain: P_m includes encoder differencing and speed filtering.")
    for name, tf in formulas.items():
        print(f"  {name}: {tf}")
    transfers, continuous_values, k, kt = numerical_loops(data, formulas, symbols)
    print(f"\nK={float(k):.10g} V/unit, psi={float(data['psi_pm']):.12g} Wb, "
          f"Kt={float(kt):.12g} N*m/A, T={float(data['Ts_ctrl']):.10g} s")
    print("\ns domain illustration only: continuous PI and ideal instantaneous sensing")
    for name, tf in continuous.items():
        describe(name, sp.cancel(tf.xreplace(continuous_values)), s)
    print("\nz domain: sampled feedback, held drive and RK4 plant")
    for name, tf in transfers.items():
        describe(name, tf, z, show_poles=name.startswith(("i/i_ref", "omega_e/w_ref", "omega_est/w_ref")))


if __name__ == "__main__":
    main()
