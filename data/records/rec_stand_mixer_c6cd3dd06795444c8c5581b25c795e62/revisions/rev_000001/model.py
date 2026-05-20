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
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    cx, cy = center
    return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width_y, height_z, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_footprint_stand_mixer")

    enamel = model.material("enamel", rgba=(0.93, 0.93, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.25, 0.26, 0.28, 1.0))

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.24, 0.15, 0.035), 0.026),
        "base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.045, 0.0, 0.013)),
        material=enamel,
        name="base_plate",
    )

    neck_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.105, 0.092, 0.028, z=0.018, center=(-0.030, 0.0)),
                _xy_section(0.082, 0.084, 0.026, z=0.125, center=(-0.048, 0.0)),
                _xy_section(0.064, 0.078, 0.023, z=0.235, center=(-0.063, 0.0)),
                _xy_section(0.058, 0.074, 0.020, z=0.305, center=(-0.072, 0.0)),
            ]
        ),
        "rear_neck",
    )
    base.visual(neck_mesh, material=enamel, name="rear_neck")

    base.visual(
        Box((0.105, 0.018, 0.008)),
        origin=Origin(xyz=(0.085, -0.040, 0.030)),
        material=satin_dark,
        name="left_rail",
    )
    base.visual(
        Box((0.105, 0.018, 0.008)),
        origin=Origin(xyz=(0.085, 0.040, 0.030)),
        material=satin_dark,
        name="right_rail",
    )
    base.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, 0.032)),
        material=enamel,
        name="rail_bridge",
    )

    for y, name in ((-0.026, "left_hinge_lug"), (0.026, "right_hinge_lug")):
        base.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(
                xyz=(-0.072, y, 0.318),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_dark,
            name=name,
        )

    base.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(-0.034, 0.045, 0.182),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_dark,
        name="lever_boss",
    )

    base.visual(
        Box((0.038, 0.016, 0.004)),
        origin=Origin(xyz=(-0.006, 0.046, 0.087)),
        material=satin_dark,
        name="lock_guide_lower",
    )
    base.visual(
        Box((0.038, 0.016, 0.004)),
        origin=Origin(xyz=(-0.006, 0.046, 0.101)),
        material=satin_dark,
        name="lock_guide_upper",
    )
    base.visual(
        Box((0.006, 0.016, 0.018)),
        origin=Origin(xyz=(-0.025, 0.046, 0.094)),
        material=satin_dark,
        name="lock_guide_backstop",
    )

    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.15, 0.34)),
        mass=8.2,
        origin=Origin(xyz=(0.015, 0.0, 0.17)),
    )

    carriage = model.part("bowl_carriage")
    carriage.visual(
        Box((0.068, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.005)),
        material=dark_trim,
        name="left_shoe",
    )
    carriage.visual(
        Box((0.068, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.040, 0.005)),
        material=dark_trim,
        name="right_shoe",
    )
    carriage.visual(
        Box((0.088, 0.106, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=enamel,
        name="carriage_deck",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=enamel,
        name="bowl_pedestal",
    )
    carriage.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=satin_dark,
        name="bowl_support_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.09, 0.11, 0.07)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    bowl = model.part("bowl")
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.028, 0.000),
                (0.054, 0.012),
                (0.074, 0.050),
                (0.081, 0.094),
                (0.086, 0.106),
            ],
            [
                (0.000, 0.004),
                (0.046, 0.014),
                (0.070, 0.050),
                (0.076, 0.100),
            ],
            segments=64,
            lip_samples=8,
        ),
        "mixing_bowl",
    )
    bowl.visual(bowl_mesh, material=steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.106),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
    )

    head = model.part("head")
    head_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.000, 0.016, 0.030, 0.006, z_center=0.012),
                _yz_section(0.022, 0.020, 0.034, 0.008, z_center=0.018),
                _yz_section(0.058, 0.072, 0.068, 0.024, z_center=0.022),
                _yz_section(0.125, 0.086, 0.096, 0.030, z_center=0.010),
                _yz_section(0.180, 0.072, 0.088, 0.024, z_center=-0.006),
                _yz_section(0.215, 0.050, 0.060, 0.016, z_center=-0.010),
            ]
        ),
        "tilt_head",
    )
    head.visual(head_mesh, material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_dark,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.019, length=0.032),
        origin=Origin(xyz=(0.154, 0.0, -0.036)),
        material=satin_dark,
        name="nose_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.09, 0.11)),
        mass=3.6,
        origin=Origin(xyz=(0.110, 0.0, -0.002)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=satin_dark,
        name="drive_stub",
    )
    spindle.visual(
        Cylinder(radius=0.0055, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
        material=satin_dark,
        name="paddle_coupler",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.085),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    paddle = model.part("paddle")
    paddle_outer = [
        (-0.006, 0.000),
        (0.006, 0.000),
        (0.007, -0.018),
        (0.019, -0.030),
        (0.020, -0.078),
        (-0.019, -0.074),
        (-0.019, -0.030),
        (-0.007, -0.018),
    ]
    paddle_inner = [
        (-0.008, -0.028),
        (0.007, -0.028),
        (0.009, -0.050),
        (-0.009, -0.048),
    ]
    paddle_geom = ExtrudeWithHolesGeometry(
        paddle_outer,
        [paddle_inner],
        0.010,
        center=True,
    )
    paddle_geom.rotate_x(math.pi / 2.0)
    paddle_geom.merge(CylinderGeometry(radius=0.0048, height=0.020).translate(0.0, 0.0, -0.010))
    paddle_mesh = mesh_from_geometry(paddle_geom, "paddle_attachment")
    paddle.visual(paddle_mesh, material=steel, name="paddle_shell")
    paddle.inertial = Inertial.from_geometry(
        Box((0.050, 0.010, 0.080)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_hub",
    )
    speed_lever.visual(
        Box((0.040, 0.006, 0.010)),
        origin=Origin(xyz=(0.017, 0.0, 0.013), rpy=(0.0, -0.55, 0.0)),
        material=dark_trim,
        name="lever_arm",
    )
    speed_lever.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(
            xyz=(0.034, 0.0, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="lever_grip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.050, 0.022, 0.035)),
        mass=0.07,
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
    )

    lock_release = model.part("lock_release")
    lock_release.visual(
        Box((0.022, 0.008, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
        material=dark_trim,
        name="lock_tongue",
    )
    lock_release.visual(
        Box((0.014, 0.012, 0.020)),
        origin=Origin(xyz=(0.002, 0.010, 0.003)),
        material=dark_trim,
        name="lock_button",
    )
    lock_release.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.022)),
        mass=0.05,
        origin=Origin(xyz=(0.001, 0.006, 0.003)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.085, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.10, lower=0.0, upper=0.032),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.072, 0.0, 0.318)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    model.articulation(
        "head_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.154, 0.0, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    model.articulation(
        "spindle_to_paddle",
        ArticulationType.FIXED,
        parent=spindle,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
    )

    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.034, 0.061, 0.182)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(26.0),
        ),
    )

    model.articulation(
        "base_to_lock_release",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_release,
        origin=Origin(xyz=(-0.008, 0.056, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.010),
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

    base = object_model.get_part("base")
    carriage = object_model.get_part("bowl_carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    spindle = object_model.get_part("spindle")
    paddle = object_model.get_part("paddle")
    speed_lever = object_model.get_part("speed_lever")
    lock_release = object_model.get_part("lock_release")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    spindle_spin = object_model.get_articulation("head_to_spindle")
    lever_joint = object_model.get_articulation("base_to_speed_lever")
    lock_joint = object_model.get_articulation("base_to_lock_release")

    def _center_of(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="left_shoe",
        negative_elem="left_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="left carriage shoe sits on rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="left_shoe",
        elem_b="left_rail",
        min_overlap=0.050,
        name="left carriage shoe remains guided at rest",
    )
    ctx.expect_overlap(
        bowl,
        paddle,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="paddle_shell",
        min_overlap=0.010,
        name="paddle is centered over the bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.030,
        max_gap=0.090,
        name="closed head clears the bowl rim",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.032}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="left_shoe",
            elem_b="left_rail",
            min_overlap=0.030,
            name="left carriage shoe stays engaged when extended",
        )
    ctx.check(
        "bowl carriage slides forward",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.025,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    spindle_rest = ctx.part_world_position(spindle)
    with ctx.pose({head_tilt: math.radians(58.0)}):
        spindle_open = ctx.part_world_position(spindle)
    ctx.check(
        "tilt head raises the spindle",
        spindle_rest is not None
        and spindle_open is not None
        and spindle_open[2] > spindle_rest[2] + 0.10
        and spindle_open[0] < spindle_rest[0] - 0.025,
        details=f"rest={spindle_rest}, open={spindle_open}",
    )

    paddle_rest_aabb = ctx.part_element_world_aabb(paddle, elem="paddle_shell")
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        paddle_turned_aabb = ctx.part_element_world_aabb(paddle, elem="paddle_shell")
    paddle_rest_x = None if paddle_rest_aabb is None else paddle_rest_aabb[1][0] - paddle_rest_aabb[0][0]
    paddle_rest_y = None if paddle_rest_aabb is None else paddle_rest_aabb[1][1] - paddle_rest_aabb[0][1]
    paddle_turned_x = None if paddle_turned_aabb is None else paddle_turned_aabb[1][0] - paddle_turned_aabb[0][0]
    paddle_turned_y = None if paddle_turned_aabb is None else paddle_turned_aabb[1][1] - paddle_turned_aabb[0][1]
    ctx.check(
        "paddle rotates with the spindle",
        paddle_rest_x is not None
        and paddle_rest_y is not None
        and paddle_turned_x is not None
        and paddle_turned_y is not None
        and paddle_rest_x > paddle_rest_y * 3.0
        and paddle_turned_y > paddle_turned_x * 3.0,
        details=(
            f"rest_spans=({paddle_rest_x}, {paddle_rest_y}), "
            f"turned_spans=({paddle_turned_x}, {paddle_turned_y})"
        ),
    )

    grip_rest = _center_of(ctx.part_element_world_aabb(speed_lever, elem="lever_grip"))
    with ctx.pose({lever_joint: math.radians(26.0)}):
        grip_raised = _center_of(ctx.part_element_world_aabb(speed_lever, elem="lever_grip"))
    ctx.check(
        "speed lever pivots upward",
        grip_rest is not None
        and grip_raised is not None
        and grip_raised[2] > grip_rest[2] + 0.010,
        details=f"rest={grip_rest}, raised={grip_raised}",
    )

    lock_rest = ctx.part_world_position(lock_release)
    with ctx.pose({lock_joint: 0.010}):
        lock_extended = ctx.part_world_position(lock_release)
    ctx.check(
        "lock release translates forward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.008,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
