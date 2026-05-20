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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _segment_pitch(dx: float, dz: float) -> float:
    return -math.atan2(dz, dx)


def _segment_length(dx: float, dz: float) -> float:
    return math.hypot(dx, dz)


def _segment_point(dx: float, dz: float, t: float) -> tuple[float, float, float]:
    return (dx * t, 0.0, dz * t)


def _shade_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.021, 0.000),
            (0.032, 0.018),
            (0.049, 0.070),
            (0.068, 0.140),
            (0.078, 0.176),
        ],
        [
            (0.010, 0.000),
            (0.023, 0.020),
            (0.040, 0.072),
            (0.061, 0.140),
            (0.075, 0.176),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp")

    enamel_green = model.material("enamel_green", rgba=(0.16, 0.20, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.30, 0.31, 0.34, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=enamel_green,
        name="base_weight",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark_hardware,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.342),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=satin_steel,
        name="post",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.393)),
        material=dark_hardware,
        name="post_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.40)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_hardware,
        name="turret_body",
    )
    swivel_head.visual(
        Box((0.030, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_hardware,
        name="shoulder_pedestal",
    )
    swivel_head.visual(
        Box((0.020, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.013, 0.044)),
        material=satin_steel,
        name="shoulder_left_cheek",
    )
    swivel_head.visual(
        Box((0.020, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.013, 0.044)),
        material=satin_steel,
        name="shoulder_right_cheek",
    )
    swivel_head.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.060)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    lower_arm = model.part("lower_arm")
    lower_dx = 0.245
    lower_dz = 0.150
    lower_pitch = _segment_pitch(lower_dx, lower_dz)
    lower_len = _segment_length(lower_dx, lower_dz)
    lower_rail_len = lower_len - 0.060
    lower_mid_x, _, lower_mid_z = _segment_point(lower_dx, lower_dz, 0.50)
    lower_root_x, _, lower_root_z = _segment_point(lower_dx, lower_dz, 0.130)
    lower_bridge_x, _, lower_bridge_z = _segment_point(lower_dx, lower_dz, 0.055)
    lower_brace_x, _, lower_brace_z = _segment_point(lower_dx, lower_dz, 0.56)
    lower_tip_bridge_x, _, lower_tip_bridge_z = _segment_point(lower_dx, lower_dz, 0.84)

    lower_arm.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Box((0.050, 0.018, 0.018)),
        origin=Origin(xyz=(lower_root_x, 0.0, lower_root_z), rpy=(0.0, lower_pitch, 0.0)),
        material=dark_hardware,
        name="root_knuckle",
    )
    lower_arm.visual(
        Box((0.024, 0.014, 0.012)),
        origin=Origin(
            xyz=(lower_bridge_x, 0.0, lower_bridge_z),
            rpy=(0.0, lower_pitch, 0.0),
        ),
        material=dark_hardware,
        name="root_bridge",
    )
    lower_arm.visual(
        Box((lower_rail_len, 0.008, 0.010)),
        origin=Origin(xyz=(lower_mid_x, 0.013, lower_mid_z), rpy=(0.0, lower_pitch, 0.0)),
        material=graphite,
        name="left_rail",
    )
    lower_arm.visual(
        Box((lower_rail_len, 0.008, 0.010)),
        origin=Origin(xyz=(lower_mid_x, -0.013, lower_mid_z), rpy=(0.0, lower_pitch, 0.0)),
        material=graphite,
        name="right_rail",
    )
    lower_arm.visual(
        Box((0.028, 0.038, 0.012)),
        origin=Origin(xyz=(lower_brace_x, 0.0, lower_brace_z), rpy=(0.0, lower_pitch, 0.0)),
        material=graphite,
        name="mid_brace",
    )
    lower_arm.visual(
        Box((0.024, 0.022, 0.016)),
        origin=Origin(
            xyz=(lower_tip_bridge_x, 0.0, lower_tip_bridge_z),
            rpy=(0.0, lower_pitch, 0.0),
        ),
        material=dark_hardware,
        name="tip_bridge",
    )
    lower_arm.visual(
        Box((0.060, 0.008, 0.026)),
        origin=Origin(xyz=(lower_dx - 0.012, 0.013, lower_dz), rpy=(0.0, lower_pitch, 0.0)),
        material=satin_steel,
        name="elbow_left_cheek",
    )
    lower_arm.visual(
        Box((0.060, 0.008, 0.026)),
        origin=Origin(xyz=(lower_dx - 0.012, -0.013, lower_dz), rpy=(0.0, lower_pitch, 0.0)),
        material=satin_steel,
        name="elbow_right_cheek",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.29, 0.05, 0.19)),
        mass=0.45,
        origin=Origin(xyz=(0.125, 0.0, 0.080)),
    )

    upper_arm = model.part("upper_arm")
    upper_dx = 0.228
    upper_dz = 0.115
    upper_pitch = _segment_pitch(upper_dx, upper_dz)
    upper_len = _segment_length(upper_dx, upper_dz)
    upper_rail_len = upper_len - 0.055
    upper_mid_x, _, upper_mid_z = _segment_point(upper_dx, upper_dz, 0.50)
    upper_root_x, _, upper_root_z = _segment_point(upper_dx, upper_dz, 0.070)
    upper_brace_x, _, upper_brace_z = _segment_point(upper_dx, upper_dz, 0.58)
    upper_tip_bridge_x, _, upper_tip_bridge_z = _segment_point(upper_dx, upper_dz, 0.92)

    upper_arm.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_barrel",
    )
    upper_arm.visual(
        Box((0.030, 0.020, 0.016)),
        origin=Origin(xyz=(upper_root_x, 0.0, upper_root_z), rpy=(0.0, upper_pitch, 0.0)),
        material=dark_hardware,
        name="root_knuckle",
    )
    upper_arm.visual(
        Box((upper_rail_len, 0.008, 0.010)),
        origin=Origin(xyz=(upper_mid_x, 0.014, upper_mid_z), rpy=(0.0, upper_pitch, 0.0)),
        material=graphite,
        name="left_rail",
    )
    upper_arm.visual(
        Box((upper_rail_len, 0.008, 0.010)),
        origin=Origin(xyz=(upper_mid_x, -0.014, upper_mid_z), rpy=(0.0, upper_pitch, 0.0)),
        material=graphite,
        name="right_rail",
    )
    upper_arm.visual(
        Box((0.024, 0.034, 0.012)),
        origin=Origin(xyz=(upper_brace_x, 0.0, upper_brace_z), rpy=(0.0, upper_pitch, 0.0)),
        material=graphite,
        name="mid_brace",
    )
    upper_arm.visual(
        Box((0.026, 0.020, 0.016)),
        origin=Origin(
            xyz=(upper_tip_bridge_x, 0.0, upper_tip_bridge_z),
            rpy=(0.0, upper_pitch, 0.0),
        ),
        material=dark_hardware,
        name="tip_bridge",
    )
    upper_arm.visual(
        Box((0.022, 0.008, 0.022)),
        origin=Origin(xyz=(upper_dx, 0.012, upper_dz), rpy=(0.0, upper_pitch, 0.0)),
        material=satin_steel,
        name="shade_left_cheek",
    )
    upper_arm.visual(
        Box((0.022, 0.008, 0.022)),
        origin=Origin(xyz=(upper_dx, -0.012, upper_dz), rpy=(0.0, upper_pitch, 0.0)),
        material=satin_steel,
        name="shade_right_cheek",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.27, 0.05, 0.16)),
        mass=0.38,
        origin=Origin(xyz=(0.115, 0.0, 0.060)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="tilt_barrel",
    )
    shade.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, -0.006)),
        material=dark_hardware,
        name="yoke_block",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.036, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="shade_neck",
    )
    shade.visual(
        mesh_from_geometry(_shade_shell(), "architect_lamp_shade_shell"),
        origin=Origin(xyz=(0.032, 0.0, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel_green,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.21, 0.16, 0.12)),
        mass=0.55,
        origin=Origin(xyz=(0.100, 0.0, -0.028)),
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swivel_head,
        origin=Origin(xyz=(0.0, 0.0, 0.398)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "swivel_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.10,
            upper=1.15,
        ),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(lower_dx, 0.0, lower_dz)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-2.00,
            upper=1.20,
        ),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(upper_dx, 0.0, upper_dz)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-1.05,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    swivel_head = object_model.get_part("swivel_head")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    swivel = object_model.get_articulation("base_to_swivel")
    shoulder = object_model.get_articulation("swivel_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    tilt = object_model.get_articulation("upper_to_shade")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
        "key lamp visuals present",
        all(
            (
                base.get_visual("base_weight") is not None,
                base.get_visual("post") is not None,
                swivel_head.get_visual("turret_body") is not None,
                lower_arm.get_visual("shoulder_barrel") is not None,
                upper_arm.get_visual("elbow_barrel") is not None,
                shade.get_visual("shade_shell") is not None,
            )
        ),
        details="Expected named visuals for the base, post, arm pivots, and shade shell.",
    )
    ctx.check(
        "architect lamp joint axes",
        swivel.axis == (0.0, 0.0, 1.0)
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and tilt.axis == (0.0, -1.0, 0.0),
        details=(
            f"swivel={swivel.axis}, shoulder={shoulder.axis}, "
            f"elbow={elbow.axis}, tilt={tilt.axis}"
        ),
    )

    ctx.expect_gap(
        swivel_head,
        base,
        axis="z",
        positive_elem="turret_body",
        negative_elem="post_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel head sits on the post cap",
    )
    ctx.expect_contact(
        lower_arm,
        swivel_head,
        elem_a="shoulder_barrel",
        elem_b="shoulder_left_cheek",
        name="lower arm shoulder barrel is supported by the swivel head",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a="elbow_barrel",
        elem_b="elbow_left_cheek",
        name="upper arm elbow barrel is supported by the lower arm fork",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a="tilt_barrel",
        elem_b="shade_left_cheek",
        name="shade tilt barrel is supported by the upper arm fork",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({swivel: 0.9}):
        turned_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "swivel joint turns the lamp around the post",
        rest_shade_pos is not None
        and turned_shade_pos is not None
        and abs(turned_shade_pos[1]) > 0.10
        and turned_shade_pos[0] < rest_shade_pos[0] - 0.05,
        details=f"rest={rest_shade_pos}, turned={turned_shade_pos}",
    )

    rest_elbow_pos = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.45}):
        raised_elbow_pos = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder joint lifts the elbow",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.06,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_shade_from_elbow = ctx.part_world_position(shade)
    with ctx.pose({elbow: 0.55}):
        raised_shade_from_elbow = ctx.part_world_position(shade)
    ctx.check(
        "elbow joint changes the reach of the upper arm",
        rest_shade_from_elbow is not None
        and raised_shade_from_elbow is not None
        and raised_shade_from_elbow[2] > rest_shade_from_elbow[2] + 0.04,
        details=f"rest={rest_shade_from_elbow}, raised={raised_shade_from_elbow}",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        (mn_x, mn_y, mn_z), (mx_x, mx_y, mx_z) = aabb
        return (
            0.5 * (mn_x + mx_x),
            0.5 * (mn_y + mx_y),
            0.5 * (mn_z + mx_z),
        )

    rest_shade_shell = _center_from_aabb(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({tilt: 0.45}):
        raised_shade_shell = _center_from_aabb(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade tilt raises the nose when opened",
        rest_shade_shell is not None
        and raised_shade_shell is not None
        and raised_shade_shell[2] > rest_shade_shell[2] + 0.015,
        details=f"rest={rest_shade_shell}, raised={raised_shade_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
