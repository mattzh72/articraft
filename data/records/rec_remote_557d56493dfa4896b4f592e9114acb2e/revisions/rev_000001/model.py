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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_rf_pendant_controller")

    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.73, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.18, 0.19, 0.20, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.60, 0.62, 0.65, 1.0))
    alarm_red = model.material("alarm_red", rgba=(0.79, 0.12, 0.12, 1.0))
    signal_green = model.material("signal_green", rgba=(0.18, 0.54, 0.26, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.53, 0.12, 1.0))
    smoked_guard = model.material("smoked_guard", rgba=(0.76, 0.18, 0.14, 0.55))

    body_shell = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.118, 0.205, radius=0.016, corner_segments=8),
            0.048,
            cap=True,
            closed=True,
        ),
        "pendant_body_shell",
    )
    antenna_boot = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0085, 0.0),
                (0.0078, 0.030),
                (0.0068, 0.080),
                (0.0058, 0.110),
            ],
            inner_profile=[
                (0.0048, 0.0),
                (0.0044, 0.030),
                (0.0040, 0.080),
                (0.0037, 0.110),
            ],
            segments=32,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "pendant_antenna_boot",
    )

    enclosure = model.part("enclosure")
    enclosure.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.1025), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="main_shell",
    )
    enclosure.visual(
        Box((0.102, 0.006, 0.172)),
        origin=Origin(xyz=(0.0, 0.024, 0.104)),
        material=bezel_black,
        name="front_bezel",
    )
    enclosure.visual(
        Box((0.090, 0.012, 0.118)),
        origin=Origin(xyz=(0.0, -0.018, 0.100)),
        material=bezel_black,
        name="rear_battery_door",
    )
    enclosure.visual(
        Box((0.010, 0.020, 0.160)),
        origin=Origin(xyz=(-0.055, 0.0, 0.100)),
        material=rubber_black,
        name="left_rubber_rail",
    )
    enclosure.visual(
        Box((0.010, 0.020, 0.160)),
        origin=Origin(xyz=(0.055, 0.0, 0.100)),
        material=rubber_black,
        name="right_rubber_rail",
    )
    for index, (x_sign, y_sign) in enumerate(((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))):
        enclosure.visual(
            Cylinder(radius=0.0115, length=0.198),
            origin=Origin(xyz=(0.048 * x_sign, 0.018 * y_sign, 0.102)),
            material=rubber_black,
            name=f"corner_guard_{index}",
        )
    enclosure.visual(
        Box((0.094, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.027, 0.162)),
        material=bezel_black,
        name="estop_shroud",
    )
    enclosure.visual(
        Box((0.086, 0.004, 0.052)),
        origin=Origin(xyz=(0.0, 0.037, 0.162)),
        material=metal_gray,
        name="estop_faceplate",
    )
    enclosure.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, 0.038, 0.159), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="estop_collar",
    )
    enclosure.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.0, 0.048, 0.159), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alarm_red,
        name="estop_cap",
    )
    enclosure.visual(
        Box((0.080, 0.010, 0.090)),
        origin=Origin(xyz=(0.0, 0.022, 0.078)),
        material=bezel_black,
        name="control_pad",
    )
    for row, z_pos in enumerate((0.096, 0.066)):
        for col, x_pos in enumerate((-0.020, 0.020)):
            enclosure.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x_pos, 0.029, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=signal_green if row == 0 else amber,
                name=f"command_button_{row}_{col}",
            )
    enclosure.visual(
        Box((0.042, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.0305, 0.040)),
        material=metal_gray,
        name="toggle_rocker",
    )
    enclosure.visual(
        Cylinder(radius=0.0095, length=0.008),
        origin=Origin(xyz=(0.036, -0.010, 0.209), rpy=(0.0, 0.0, 0.0)),
        material=metal_gray,
        name="antenna_base_collar",
    )
    enclosure.visual(
        antenna_boot,
        origin=Origin(xyz=(0.036, -0.010, 0.211)),
        material=rubber_black,
        name="antenna_boot",
    )
    enclosure.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.0, 0.035, 0.194), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="guard_hinge_knuckle",
    )
    enclosure.inertial = Inertial.from_geometry(
        Box((0.132, 0.078, 0.325)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.1625)),
    )

    safety_guard = model.part("safety_guard")
    safety_guard.visual(
        Box((0.080, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, -0.004)),
        material=rubber_black,
        name="guard_top_bar",
    )
    safety_guard.visual(
        Box((0.078, 0.003, 0.040)),
        origin=Origin(xyz=(0.0, 0.031, -0.026)),
        material=smoked_guard,
        name="guard_shield",
    )
    safety_guard.visual(
        Box((0.006, 0.036, 0.046)),
        origin=Origin(xyz=(-0.036, 0.018, -0.023)),
        material=rubber_black,
        name="guard_side_left",
    )
    safety_guard.visual(
        Box((0.006, 0.036, 0.046)),
        origin=Origin(xyz=(0.036, 0.018, -0.023)),
        material=rubber_black,
        name="guard_side_right",
    )
    safety_guard.visual(
        Box((0.078, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.030, -0.047)),
        material=rubber_black,
        name="guard_lower_lip",
    )
    safety_guard.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(-0.026, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="hinge_knuckle_left",
    )
    safety_guard.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.026, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="hinge_knuckle_right",
    )
    safety_guard.inertial = Inertial.from_geometry(
        Box((0.084, 0.028, 0.055)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.015, -0.024)),
    )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(radius=0.0036, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=metal_gray,
        name="antenna_lower_stage",
    )
    antenna.visual(
        Cylinder(radius=0.0022, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=metal_gray,
        name="antenna_whip",
    )
    antenna.visual(
        Sphere(radius=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=metal_gray,
        name="antenna_tip",
    )
    antenna.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.460)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=safety_guard,
        origin=Origin(xyz=(0.0, 0.035, 0.194)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.9,
        ),
    )
    model.articulation(
        "antenna_extension",
        ArticulationType.PRISMATIC,
        parent=enclosure,
        child=antenna,
        origin=Origin(xyz=(0.036, -0.010, 0.321)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.20,
            lower=0.0,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    enclosure = object_model.get_part("enclosure")
    safety_guard = object_model.get_part("safety_guard")
    antenna = object_model.get_part("antenna")
    guard_hinge = object_model.get_articulation("guard_hinge")
    antenna_extension = object_model.get_articulation("antenna_extension")

    ctx.expect_overlap(
        safety_guard,
        enclosure,
        axes="xz",
        elem_a="guard_shield",
        elem_b="estop_cap",
        min_overlap=0.035,
        name="closed guard covers the emergency stop cluster",
    )
    ctx.expect_gap(
        safety_guard,
        enclosure,
        axis="y",
        positive_elem="guard_shield",
        negative_elem="estop_cap",
        min_gap=0.004,
        max_gap=0.020,
        name="closed guard sits just proud of the emergency stop",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    closed_guard_center = _aabb_center(ctx.part_element_world_aabb(safety_guard, elem="guard_shield"))
    open_guard_center = None
    with ctx.pose({guard_hinge: 1.2}):
        ctx.expect_gap(
            safety_guard,
            enclosure,
            axis="z",
            positive_elem="guard_shield",
            negative_elem="estop_cap",
            min_gap=0.005,
            name="opened guard lifts above the emergency stop",
        )
        open_guard_center = _aabb_center(ctx.part_element_world_aabb(safety_guard, elem="guard_shield"))

    ctx.check(
        "guard opens upward and outward",
        closed_guard_center is not None
        and open_guard_center is not None
        and open_guard_center[1] > closed_guard_center[1] + 0.003
        and open_guard_center[2] > closed_guard_center[2] + 0.030,
        details=f"closed_center={closed_guard_center}, open_center={open_guard_center}",
    )

    ctx.expect_within(
        antenna,
        enclosure,
        axes="xy",
        inner_elem="antenna_lower_stage",
        outer_elem="antenna_boot",
        margin=0.001,
        name="antenna lower stage stays centered in the boot",
    )
    ctx.expect_overlap(
        antenna,
        enclosure,
        axes="z",
        elem_a="antenna_lower_stage",
        elem_b="antenna_boot",
        min_overlap=0.080,
        name="antenna lower stage remains inserted at rest",
    )

    rest_pos = ctx.part_world_position(antenna)
    extended_pos = None
    with ctx.pose({antenna_extension: 0.080}):
        ctx.expect_within(
            antenna,
            enclosure,
            axes="xy",
            inner_elem="antenna_lower_stage",
            outer_elem="antenna_boot",
            margin=0.001,
            name="antenna lower stage stays centered when extended",
        )
        ctx.expect_overlap(
            antenna,
            enclosure,
            axes="z",
            elem_a="antenna_lower_stage",
            elem_b="antenna_boot",
            min_overlap=0.030,
            name="antenna retains insertion when extended",
        )
        extended_pos = ctx.part_world_position(antenna)

    ctx.check(
        "antenna extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.060,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
