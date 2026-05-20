from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_visuals(part, *, mesh_prefix: str, tire_radius: float, tire_width: float, tire_mat, core_mat) -> None:
    wheel_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.73, -half_width * 0.86),
        (tire_radius * 0.88, -half_width * 0.98),
        (tire_radius * 0.97, -half_width * 0.58),
        (tire_radius, 0.0),
        (tire_radius * 0.97, half_width * 0.58),
        (tire_radius * 0.88, half_width * 0.98),
        (tire_radius * 0.73, half_width * 0.86),
        (tire_radius * 0.69, half_width * 0.38),
        (tire_radius * 0.67, 0.0),
        (tire_radius * 0.69, -half_width * 0.38),
        (tire_radius * 0.73, -half_width * 0.86),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_x(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_mat, name="tire")
    part.visual(
        Cylinder(radius=tire_radius * 0.69, length=tire_width * 0.76),
        origin=wheel_axis,
        material=core_mat,
        name="rim_core",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.53, length=tire_width * 0.12),
        origin=Origin(xyz=(0.0, tire_width * 0.24, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_mat,
        name="left_plate",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.53, length=tire_width * 0.12),
        origin=Origin(xyz=(0.0, -tire_width * 0.24, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_mat,
        name="right_plate",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.25, length=tire_width * 1.26),
        origin=wheel_axis,
        material=core_mat,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=wheel_axis,
        material=core_mat,
        name="axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_kick_scooter")

    deck_black = model.material("deck_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel_black = model.material("steel_black", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))

    wheel_radius = 0.055
    wheel_width = 0.024
    rear_axle_x = -0.240
    steering_origin_xyz = (0.244, 0.0, 0.135)
    front_axle_rel = (0.048, 0.0, -0.080)
    steer_tilt = math.radians(13.0)
    steering_axis = (-math.sin(steer_tilt), 0.0, math.cos(steer_tilt))
    steering_rpy = (0.0, -steer_tilt, 0.0)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.14, 0.22)),
        mass=3.7,
        origin=Origin(xyz=(-0.03, 0.0, 0.11)),
    )

    deck_mesh = _save_mesh(
        "deck_shell",
        ExtrudeGeometry(rounded_rect_profile(0.355, 0.11, 0.014), 0.018),
    )
    frame.visual(
        deck_mesh,
        origin=Origin(xyz=(-0.005, 0.0, 0.069)),
        material=deck_black,
        name="deck_shell",
    )
    frame.visual(
        Box((0.235, 0.092, 0.003)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0795)),
        material=grip_black,
        name="grip_tape",
    )
    frame.visual(
        Box((0.100, 0.072, 0.012)),
        origin=Origin(xyz=(0.110, 0.0, 0.084)),
        material=deck_black,
        name="neck_plate",
    )
    frame.visual(
        Box((0.106, 0.008, 0.078)),
        origin=Origin(xyz=(rear_axle_x, 0.019, 0.093)),
        material=steel_black,
        name="left_dropout",
    )
    frame.visual(
        Box((0.106, 0.008, 0.078)),
        origin=Origin(xyz=(rear_axle_x, -0.019, 0.093)),
        material=steel_black,
        name="right_dropout",
    )
    frame.visual(
        Box((0.114, 0.008, 0.036)),
        origin=Origin(xyz=(-0.184, 0.019, 0.078)),
        material=steel_black,
        name="left_tail_strut",
    )
    frame.visual(
        Box((0.114, 0.008, 0.036)),
        origin=Origin(xyz=(-0.184, -0.019, 0.078)),
        material=steel_black,
        name="right_tail_strut",
    )
    frame.visual(
        Box((0.074, 0.074, 0.012)),
        origin=Origin(xyz=(rear_axle_x, 0.0, 0.124)),
        material=steel_black,
        name="tail_bridge",
    )
    frame.visual(
        Box((0.040, 0.078, 0.016)),
        origin=Origin(xyz=(rear_axle_x - 0.018, 0.0, 0.136)),
        material=steel_black,
        name="fender",
    )
    frame.visual(
        Box((0.070, 0.012, 0.032)),
        origin=Origin(xyz=(0.214, 0.024, 0.123)),
        material=deck_black,
        name="left_head_gusset",
    )
    frame.visual(
        Box((0.070, 0.012, 0.032)),
        origin=Origin(xyz=(0.214, -0.024, 0.123)),
        material=deck_black,
        name="right_head_gusset",
    )

    left_neck_rail = tube_from_spline_points(
        [
            (0.120, 0.040, 0.078),
            (0.175, 0.032, 0.100),
            (0.232, 0.022, 0.124),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_neck_rail = tube_from_spline_points(
        [
            (0.120, -0.040, 0.078),
            (0.175, -0.032, 0.100),
            (0.232, -0.022, 0.124),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
    )
    frame.visual(_save_mesh("left_neck_rail", left_neck_rail), material=steel_black, name="left_neck_rail")
    frame.visual(_save_mesh("right_neck_rail", right_neck_rail), material=steel_black, name="right_neck_rail")

    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.019, -0.055),
            (0.021, -0.018),
            (0.021, 0.018),
            (0.019, 0.055),
        ],
        [
            (0.015, -0.049),
            (0.0165, -0.016),
            (0.0165, 0.016),
            (0.015, 0.049),
        ],
        segments=48,
    ).rotate_y(-steer_tilt)
    frame.visual(
        _save_mesh("head_tube_shell", head_tube_shell),
        origin=Origin(
            xyz=(
                steering_origin_xyz[0] + steering_axis[0] * 0.055,
                steering_origin_xyz[1],
                steering_origin_xyz[2] + steering_axis[2] * 0.055,
            )
        ),
        material=steel_black,
        name="head_tube",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.58, 0.56, 0.84)),
        mass=1.8,
        origin=Origin(xyz=(-0.06, 0.0, 0.30)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.013, length=0.640),
        origin=Origin(
            xyz=(steering_axis[0] * 0.310, 0.0, steering_axis[2] * 0.310),
            rpy=steering_rpy,
        ),
        material=steel_black,
        name="steerer_bar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(steering_axis[0] * -0.005, 0.0, steering_axis[2] * -0.005), rpy=steering_rpy),
        material=satin_alloy,
        name="lower_headset",
    )
    steering_assembly.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(steering_axis[0] * 0.115, 0.0, steering_axis[2] * 0.115), rpy=steering_rpy),
        material=satin_alloy,
        name="upper_headset",
    )
    steering_assembly.visual(
        Box((0.058, 0.030, 0.020)),
        origin=Origin(xyz=(0.031, 0.0, -0.006)),
        material=steel_black,
        name="fork_crown",
    )
    steering_assembly.visual(
        Box((0.016, 0.008, 0.094)),
        origin=Origin(xyz=(0.041, 0.019, -0.047)),
        material=steel_black,
        name="left_leg",
    )
    steering_assembly.visual(
        Box((0.016, 0.008, 0.094)),
        origin=Origin(xyz=(0.041, -0.019, -0.047)),
        material=steel_black,
        name="right_leg",
    )
    steering_assembly.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.047, 0.019, -0.086)),
        material=steel_black,
        name="left_dropout_tab",
    )
    steering_assembly.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.047, -0.019, -0.086)),
        material=steel_black,
        name="right_dropout_tab",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.540),
        origin=Origin(xyz=(-0.142, 0.0, 0.614), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_black,
        name="bar_cross",
    )
    steering_assembly.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(-0.142, 0.225, 0.614), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    steering_assembly.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(-0.142, -0.225, 0.614), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    steering_assembly.visual(
        Box((0.056, 0.032, 0.028)),
        origin=Origin(xyz=(-0.142, 0.0, 0.599)),
        material=steel_black,
        name="bar_junction",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        mesh_prefix="front_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        tire_mat=rubber,
        core_mat=satin_alloy,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_wheel,
        mesh_prefix="rear_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        tire_mat=rubber,
        core_mat=satin_alloy,
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=steering_assembly,
        origin=Origin(xyz=steering_origin_xyz),
        axis=steering_axis,
        motion_limits=MotionLimits(effort=16.0, velocity=9.0),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_wheel,
        origin=Origin(xyz=front_axle_rel),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=40.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(rear_axle_x, 0.0, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    steering_assembly = object_model.get_part("steering_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        frame,
        steering_assembly,
        reason=(
            "The steerer and headset stack are captured inside the frame head tube; "
            "the simplified visual model omits the bearing bores and cup recesses."
        ),
    )
    ctx.allow_overlap(
        front_wheel,
        steering_assembly,
        elem_a="axle",
        elem_b="left_dropout_tab",
        reason="The front axle passes through the left fork dropout slot, which is not explicitly cut out.",
    )
    ctx.allow_overlap(
        front_wheel,
        steering_assembly,
        elem_a="axle",
        elem_b="right_dropout_tab",
        reason="The front axle passes through the right fork dropout slot, which is not explicitly cut out.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "only primary scooter articulations present",
        len(object_model.articulations) == 3,
        f"Expected steering plus two wheel-spin articulations, found {len(object_model.articulations)}.",
    )
    ctx.check(
        "steering axis is tilted back",
        steering_yaw.axis[0] < -0.1 and steering_yaw.axis[2] > 0.9,
        f"Unexpected steering axis {steering_yaw.axis}.",
    )
    ctx.check(
        "wheel joints are continuous axle spins",
        (
            front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and front_wheel_spin.motion_limits is not None
            and rear_wheel_spin.motion_limits is not None
            and front_wheel_spin.motion_limits.lower is None
            and front_wheel_spin.motion_limits.upper is None
            and rear_wheel_spin.motion_limits.lower is None
            and rear_wheel_spin.motion_limits.upper is None
        ),
        "Front and rear wheels should spin continuously without hard stops.",
    )

    ctx.expect_contact(front_wheel, steering_assembly)
    ctx.expect_contact(rear_wheel, frame)
    ctx.expect_gap(
        steering_assembly,
        front_wheel,
        axis="y",
        positive_elem="left_leg",
        negative_elem="tire",
        min_gap=0.002,
        max_gap=0.0045,
        name="front wheel clears left fork leg closely",
    )
    ctx.expect_gap(
        front_wheel,
        steering_assembly,
        axis="y",
        positive_elem="tire",
        negative_elem="right_leg",
        min_gap=0.002,
        max_gap=0.0045,
        name="front wheel clears right fork leg closely",
    )
    ctx.expect_gap(
        frame,
        rear_wheel,
        axis="z",
        positive_elem="tail_bridge",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.014,
        name="rear wheel sits tucked close under tail bridge",
    )

    front_rest = ctx.part_world_position(front_wheel)
    assert front_rest is not None
    with ctx.pose({steering_yaw: 1.1}):
        front_turned = ctx.part_world_position(front_wheel)
        assert front_turned is not None
        ctx.check(
            "steering swings fork and front wheel together",
            abs(front_turned[1] - front_rest[1]) > 0.02,
            f"Expected front wheel to move laterally when steering; rest={front_rest}, turned={front_turned}.",
        )
        ctx.expect_contact(front_wheel, steering_assembly)

    with ctx.pose({steering_yaw: math.pi}):
        ctx.expect_gap(
            front_wheel,
            frame,
            axis="x",
            positive_elem="tire",
            negative_elem="deck_shell",
            min_gap=0.0,
            max_gap=0.012,
            name="barspin pose keeps front wheel ahead of deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
