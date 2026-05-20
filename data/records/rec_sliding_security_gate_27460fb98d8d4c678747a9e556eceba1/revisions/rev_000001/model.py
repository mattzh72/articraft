from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    concrete = model.material("poured_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.44, 0.47, 0.48, 1.0))
    dark_steel = model.material("powder_coated_black_steel", rgba=(0.02, 0.025, 0.025, 1.0))
    worn_edge = model.material("worn_track_edge", rgba=(0.70, 0.72, 0.69, 1.0))
    rubber = model.material("black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.07, 1.0))
    red = model.material("emergency_red", rgba=(0.85, 0.03, 0.02, 1.0))
    green = model.material("access_green", rgba=(0.00, 0.62, 0.18, 1.0))
    glass = model.material("smoked_glass", rgba=(0.03, 0.06, 0.08, 0.82))
    white = model.material("engraved_white", rgba=(0.92, 0.92, 0.86, 1.0))

    fixed = model.part("fixed_installation")

    def fixed_box(name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
        fixed.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def fixed_cylinder(name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
        fixed.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Civil base, ground track, and the fixed structural posts.
    fixed_box("concrete_sill", (8.20, 1.35, 0.16), (0.40, 0.0, 0.08), concrete)
    fixed_box("track_base", (7.85, 0.18, 0.06), (0.35, 0.0, 0.19), galvanized)
    fixed_box("track_cap", (7.85, 0.055, 0.030), (0.35, 0.0, 0.235), worn_edge)
    fixed_box("track_anchor_strip", (7.85, 0.040, 0.020), (0.35, 0.13, 0.175), galvanized)
    for i, x in enumerate((-3.25, -2.35, -1.45, -0.55, 0.35, 1.25, 2.15, 3.05, 3.95)):
        fixed_box(f"track_bolt_{i}", (0.08, 0.08, 0.018), (x, 0.135, 0.184), dark_steel)

    for name, x, height in (("receiver_post", -2.68, 2.45), ("drive_post", 2.38, 2.55)):
        fixed_box(name, (0.20, 0.20, height), (x, 0.34, 0.15 + height / 2.0), dark_steel)
        fixed_box(f"{name}_cap", (0.24, 0.24, 0.075), (x, 0.34, 0.17 + height), galvanized)
        fixed_box(f"{name}_base_plate", (0.42, 0.42, 0.035), (x, 0.34, 0.177), galvanized)
        for sx in (-0.12, 0.12):
            for sy in (-0.12, 0.12):
                fixed_cylinder(f"{name}_anchor_{sx}_{sy}", 0.025, 0.024, (x + sx, 0.34 + sy, 0.204), dark_steel)

    # Receiver pocket and top guide yoke mounted to the support posts.
    fixed_box("receiver_pocket_back", (0.13, 0.045, 1.05), (-2.53, 0.235, 1.22), galvanized)
    fixed_box("receiver_pocket_top_lip", (0.18, 0.17, 0.055), (-2.51, 0.225, 1.77), galvanized)
    fixed_box("receiver_pocket_bottom_lip", (0.18, 0.17, 0.055), (-2.51, 0.225, 0.67), galvanized)
    fixed_box("upper_guide_arm", (0.50, 0.075, 0.075), (2.17, 0.205, 1.88), galvanized)
    fixed_box("lower_guide_arm", (0.50, 0.075, 0.075), (2.17, 0.205, 1.62), galvanized)
    fixed_box("guide_yoke_plate", (0.060, 0.18, 0.34), (1.93, 0.125, 1.75), galvanized)
    fixed_cylinder("upper_guide_roller", 0.040, 0.26, (1.93, 0.090, 1.88), rubber)
    fixed_cylinder("lower_guide_roller", 0.040, 0.26, (1.93, 0.090, 1.62), rubber)

    # Electric drive unit and rack-and-pinion guard.
    fixed_box("motor_plinth", (0.64, 0.48, 0.14), (2.92, -0.39, 0.23), concrete)
    fixed_box("motor_cabinet", (0.58, 0.38, 0.62), (2.92, -0.39, 0.61), dark_steel)
    fixed_box("motor_lid", (0.62, 0.42, 0.055), (2.92, -0.39, 0.947), galvanized)
    fixed_box("motor_front_panel", (0.48, 0.030, 0.36), (2.92, -0.592, 0.62), galvanized)
    fixed_box("pinion_guard", (0.42, 0.08, 0.30), (2.46, -0.180, 0.69), yellow)
    fixed_cylinder("drive_pinion", 0.115, 0.045, (2.30, -0.145, 0.70), dark_steel, rpy=(pi / 2.0, 0.0, 0.0))
    fixed_cylinder("beacon_stem", 0.025, 0.16, (2.92, -0.39, 1.050), dark_steel)
    fixed.visual(Sphere(radius=0.075), origin=Origin(xyz=(2.92, -0.39, 1.175)), material=yellow, name="warning_beacon")

    # Short fence returns show that the installation continues the perimeter line.
    for side, start_x, sign in (("rear", 2.50, 1.0), ("front", -2.80, -1.0)):
        fixed_box(f"{side}_fence_top_rail", (1.00, 0.075, 0.075), (start_x + sign * 0.48, 0.34, 2.03), dark_steel)
        fixed_box(f"{side}_fence_bottom_rail", (1.00, 0.075, 0.075), (start_x + sign * 0.48, 0.34, 0.63), dark_steel)
        for j in range(4):
            x = start_x + sign * (0.20 + 0.22 * j)
            fixed_box(f"{side}_fence_picket_{j}", (0.045, 0.045, 1.45), (x, 0.34, 1.33), dark_steel)

    # Access-control pedestal: card reader, intercom, status lights, and keypad frame.
    fixed_box("pedestal_base", (0.46, 0.34, 0.055), (2.18, -0.53, 0.180), galvanized)
    fixed_box("pedestal_post", (0.13, 0.13, 1.13), (2.18, -0.53, 0.73), dark_steel)
    fixed_box("control_housing", (0.42, 0.12, 0.62), (2.18, -0.55, 1.38), dark_steel)
    fixed_box("control_faceplate", (0.36, 0.008, 0.54), (2.18, -0.614, 1.38), galvanized)
    fixed_box("card_reader_window", (0.24, 0.006, 0.09), (2.18, -0.621, 1.59), glass)
    fixed_box("speaker_bar_0", (0.22, 0.006, 0.012), (2.18, -0.619, 1.47), dark_steel)
    fixed_box("speaker_bar_1", (0.22, 0.006, 0.012), (2.18, -0.619, 1.43), dark_steel)
    fixed_box("speaker_bar_2", (0.22, 0.006, 0.012), (2.18, -0.619, 1.39), dark_steel)
    fixed_cylinder("green_status_lamp", 0.019, 0.007, (2.095, -0.619, 1.64), green, rpy=(pi / 2.0, 0.0, 0.0))
    fixed_cylinder("red_status_lamp", 0.019, 0.007, (2.265, -0.619, 1.64), red, rpy=(pi / 2.0, 0.0, 0.0))
    fixed_box("keypad_recess", (0.25, 0.006, 0.30), (2.18, -0.619, 1.215), glass)
    fixed_box("stop_button_plate", (0.16, 0.010, 0.14), (2.31, -0.613, 1.00), galvanized)

    gate = model.part("gate_body")

    def gate_box(name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
        gate.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def gate_cylinder(name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
        gate.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Heavy welded sliding gate frame.
    gate_box("bottom_tube", (4.32, 0.11, 0.13), (-0.30, 0.0, 0.555), dark_steel)
    gate_box("top_tube", (4.32, 0.11, 0.13), (-0.30, 0.0, 2.085), dark_steel)
    gate_box("receiver_stile", (0.13, 0.11, 1.67), (-2.40, 0.0, 1.32), dark_steel)
    gate_box("tail_stile", (0.13, 0.11, 1.67), (1.80, 0.0, 1.32), dark_steel)
    gate_box("center_stile", (0.10, 0.10, 1.62), (-0.30, 0.0, 1.32), dark_steel)
    for i, x in enumerate((-2.05, -1.70, -1.35, -1.00, -0.65, 0.05, 0.40, 0.75, 1.10, 1.45)):
        gate_box(f"picket_{i}", (0.050, 0.060, 1.52), (x, 0.0, 1.32), dark_steel)
        gate_cylinder(f"spear_{i}", 0.024, 0.20, (x, 0.0, 2.205), dark_steel)

    # Diagonal anti-racking braces, sized to overlap both rails like welded flat bar.
    left_dx, left_dz = 2.05, 1.43
    left_len = sqrt(left_dx * left_dx + left_dz * left_dz)
    left_angle = atan2(left_dz, left_dx)
    gate_box("left_diagonal_brace", (left_len + 0.08, 0.070, 0.070), (-1.34, 0.0, 1.325), dark_steel, rpy=(0.0, -left_angle, 0.0))
    right_dx, right_dz = 2.05, -1.43
    right_len = sqrt(right_dx * right_dx + right_dz * right_dz)
    right_angle = atan2(-right_dz, right_dx)
    gate_box("right_diagonal_brace", (right_len + 0.08, 0.070, 0.070), (0.74, 0.0, 1.325), dark_steel, rpy=(0.0, right_angle, 0.0))

    # Rack rail and individual teeth for the motor drive.
    gate_box("gear_rack_spine", (3.72, 0.030, 0.040), (-0.10, -0.065, 0.735), galvanized)
    for i in range(22):
        x = -1.88 + 0.17 * i
        gate_box(f"rack_tooth_{i}", (0.075, 0.035, 0.045), (x, -0.092, 0.695), galvanized)

    # Captive rolling wheels and their axle boxes.
    for name, x in (("rear_wheel", -1.78), ("front_wheel", 1.32)):
        gate_box(f"{name}_fork", (0.24, 0.13, 0.14), (x, 0.0, 0.425), dark_steel)
        gate_cylinder(name, 0.125, 0.064, (x, 0.0, 0.375), rubber, rpy=(pi / 2.0, 0.0, 0.0))
        gate_cylinder(f"{name}_hub", 0.040, 0.080, (x, 0.0, 0.375), galvanized, rpy=(pi / 2.0, 0.0, 0.0))

    gate_box("warning_stripe_left", (0.080, 0.014, 1.12), (-2.05, -0.035, 1.30), yellow)
    gate_box("warning_stripe_right", (0.080, 0.014, 1.12), (1.45, -0.035, 1.30), yellow)
    gate_box("latch_striker", (0.12, 0.08, 0.32), (-2.50, 0.0, 1.18), galvanized)

    slide = model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=gate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.45, lower=0.0, upper=2.20),
    )
    slide.meta["description"] = "Gate leaf translates along the rail toward the drive side."

    # Individually sprung keypad buttons.
    key_centers = [
        (2.105, 1.315),
        (2.180, 1.315),
        (2.255, 1.315),
        (2.105, 1.245),
        (2.180, 1.245),
        (2.255, 1.245),
        (2.105, 1.175),
        (2.180, 1.175),
        (2.255, 1.175),
        (2.105, 1.105),
        (2.180, 1.105),
        (2.255, 1.105),
    ]
    for idx, (x, z) in enumerate(key_centers):
        row, col = divmod(idx, 3)
        key = model.part(f"key_{row}_{col}")
        key.visual(Box((0.047, 0.018, 0.038)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=rubber, name="key_cap")
        key.visual(Box((0.020, 0.003, 0.003)), origin=Origin(xyz=(0.0, -0.0080, 0.0)), material=white, name="key_mark")
        model.articulation(
            f"key_{row}_{col}_press",
            ArticulationType.PRISMATIC,
            parent=fixed,
            child=key,
            origin=Origin(xyz=(x, -0.631, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.07, lower=0.0, upper=0.008),
        )

    stop = model.part("stop_button")
    stop.visual(Cylinder(radius=0.050, length=0.026), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=red, name="mushroom_cap")
    stop.visual(Cylinder(radius=0.026, length=0.018), origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="button_stem")
    model.articulation(
        "stop_button_press",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=stop,
        origin=Origin(xyz=(2.31, -0.637, 1.00)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.012),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_installation")
    gate = object_model.get_part("gate_body")
    slide = object_model.get_articulation("gate_slide")
    sample_key = object_model.get_part("key_1_1")
    sample_key_press = object_model.get_articulation("key_1_1_press")
    stop = object_model.get_part("stop_button")
    stop_press = object_model.get_articulation("stop_button_press")

    ctx.expect_contact(
        gate,
        fixed,
        elem_a="front_wheel",
        elem_b="track_cap",
        contact_tol=0.002,
        name="front wheel sits on the steel track",
    )
    ctx.expect_contact(
        gate,
        fixed,
        elem_a="rear_wheel",
        elem_b="track_cap",
        contact_tol=0.002,
        name="rear wheel sits on the steel track",
    )
    ctx.expect_within(
        gate,
        fixed,
        axes="y",
        inner_elem="front_wheel",
        outer_elem="track_cap",
        margin=0.010,
        name="front wheel is centered over the rail",
    )

    rest_gate = ctx.part_world_position(gate)
    with ctx.pose({slide: 2.20}):
        ctx.expect_overlap(
            gate,
            fixed,
            axes="x",
            elem_a="front_wheel",
            elem_b="track_cap",
            min_overlap=0.05,
            name="opened gate still has its lead wheel on the track",
        )
        ctx.expect_within(
            gate,
            fixed,
            axes="y",
            inner_elem="front_wheel",
            outer_elem="track_cap",
            margin=0.010,
            name="opened gate remains laterally guided by the rail",
        )
        opened_gate = ctx.part_world_position(gate)

    ctx.check(
        "gate slides toward the drive post",
        rest_gate is not None and opened_gate is not None and opened_gate[0] > rest_gate[0] + 2.0,
        details=f"rest={rest_gate}, opened={opened_gate}",
    )

    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_press: 0.008}):
        key_pressed = ctx.part_world_position(sample_key)
    ctx.check(
        "keypad button depresses inward",
        key_rest is not None and key_pressed is not None and key_pressed[1] > key_rest[1] + 0.006,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    stop_rest = ctx.part_world_position(stop)
    with ctx.pose({stop_press: 0.012}):
        stop_pressed = ctx.part_world_position(stop)
    ctx.check(
        "emergency stop button has a short press stroke",
        stop_rest is not None and stop_pressed is not None and stop_pressed[1] > stop_rest[1] + 0.010,
        details=f"rest={stop_rest}, pressed={stop_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
