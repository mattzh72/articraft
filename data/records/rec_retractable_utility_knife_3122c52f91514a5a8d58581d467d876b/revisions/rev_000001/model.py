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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _extrude_xz_profile(
    profile_xz: list[tuple[float, float]],
    thickness: float,
):
    # Extrude in local +Z, then rotate so the authored XZ profile becomes an
    # object-side plate extruded across Y.
    return ExtrudeGeometry(
        [(x, -z) for x, z in profile_xz],
        thickness,
        center=True,
    ).rotate_x(-math.pi / 2.0)


def _handle_side_profile() -> list[tuple[float, float]]:
    return [
        (-0.090, 0.004),
        (-0.084, 0.000),
        (-0.062, 0.000),
        (-0.026, 0.002),
        (0.018, 0.004),
        (0.060, 0.007),
        (0.084, 0.013),
        (0.090, 0.019),
        (0.077, 0.030),
        (0.040, 0.036),
        (-0.018, 0.037),
        (-0.064, 0.034),
        (-0.086, 0.024),
        (-0.090, 0.013),
    ]


def _blade_profile() -> list[tuple[float, float]]:
    return [
        (-0.018, 0.000),
        (0.008, 0.000),
        (0.028, 0.0045),
        (0.017, 0.010),
        (-0.018, 0.010),
    ]


def _build_wheel_mesh():
    return ExtrudeWithHolesGeometry(
        _circle_profile(0.014, segments=40),
        [_circle_profile(0.0034, segments=24)],
        0.0045,
        center=True,
    ).rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_utility_knife")

    body_charcoal = model.material("body_charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.86, 0.42, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.38, 1.0))

    side_plate_mesh = mesh_from_geometry(
        _extrude_xz_profile(_handle_side_profile(), 0.003),
        "utility_knife_side_plate",
    )
    blade_mesh = mesh_from_geometry(
        _extrude_xz_profile(_blade_profile(), 0.0012),
        "utility_knife_blade",
    )
    lock_wheel_mesh = mesh_from_geometry(_build_wheel_mesh(), "utility_knife_lock_wheel")

    handle_body = model.part("handle_body")
    handle_body.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0095, 0.0)),
        material=body_charcoal,
        name="side_plate_left",
    )
    handle_body.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.0095, 0.0)),
        material=body_charcoal,
        name="side_plate_right",
    )
    handle_body.visual(
        Box((0.018, 0.022, 0.018)),
        origin=Origin(xyz=(-0.078, 0.0, 0.015)),
        material=grip_black,
        name="rear_cap",
    )
    handle_body.visual(
        Box((0.038, 0.022, 0.010)),
        origin=Origin(xyz=(-0.056, 0.0, 0.005)),
        material=grip_black,
        name="heel_bridge",
    )
    handle_body.visual(
        Box((0.082, 0.022, 0.007)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0035)),
        material=grip_black,
        name="lower_spine",
    )
    handle_body.visual(
        Box((0.110, 0.004, 0.006)),
        origin=Origin(xyz=(0.010, 0.007, 0.031)),
        material=grip_black,
        name="top_rail_left",
    )
    handle_body.visual(
        Box((0.110, 0.004, 0.006)),
        origin=Origin(xyz=(0.010, -0.007, 0.031)),
        material=grip_black,
        name="top_rail_right",
    )
    handle_body.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.058, 0.0075, 0.010)),
        material=body_charcoal,
        name="front_cheek_left",
    )
    handle_body.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.058, -0.0075, 0.010)),
        material=body_charcoal,
        name="front_cheek_right",
    )
    handle_body.visual(
        Box((0.028, 0.010, 0.006)),
        origin=Origin(xyz=(0.069, 0.0, 0.031)),
        material=body_charcoal,
        name="nose_cap",
    )
    handle_body.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.032, -0.0115, 0.019), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_boss",
    )
    handle_body.visual(
        Cylinder(radius=0.0024, length=0.006),
        origin=Origin(xyz=(0.032, -0.014, 0.019), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lock_shaft",
    )
    handle_body.inertial = Inertial.from_geometry(
        Box((0.185, 0.022, 0.040)),
        mass=0.46,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.085, 0.0135, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=accent_orange,
        name="carriage_shoe",
    )
    blade_carriage.visual(
        Box((0.020, 0.008, 0.018)),
        origin=Origin(xyz=(0.005, 0.0, 0.012)),
        material=accent_orange,
        name="thumb_stem",
    )
    blade_carriage.visual(
        Box((0.014, 0.009, 0.004)),
        origin=Origin(xyz=(0.008, 0.0, 0.022)),
        material=accent_orange,
        name="thumb_pad",
    )
    blade_carriage.visual(
        Box((0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, 0.003)),
        material=accent_orange,
        name="carrier_nose",
    )
    blade_carriage.visual(
        Box((0.012, 0.010, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.005)),
        material=accent_orange,
        name="blade_clamp",
    )
    blade_carriage.visual(
        blade_mesh,
        origin=Origin(xyz=(0.083, 0.0, 0.002)),
        material=steel,
        name="blade",
    )
    blade_carriage.inertial = Inertial.from_geometry(
        Box((0.132, 0.014, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
    )

    lock_wheel = model.part("lock_wheel")
    lock_wheel.visual(
        lock_wheel_mesh,
        material=accent_orange,
        name="wheel_ring",
    )
    lock_wheel.visual(
        Box((0.005, 0.0045, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=accent_orange,
        name="wheel_grip_upper",
    )
    lock_wheel.visual(
        Box((0.005, 0.0045, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=accent_orange,
        name="wheel_grip_lower",
    )
    lock_wheel.inertial = Inertial.from_geometry(
        Box((0.028, 0.006, 0.028)),
        mass=0.02,
        origin=Origin(),
    )

    slide_limits = MotionLimits(
        effort=25.0,
        velocity=0.25,
        lower=0.0,
        upper=0.040,
    )
    model.articulation(
        "handle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=handle_body,
        child=blade_carriage,
        origin=Origin(xyz=(-0.020, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=slide_limits,
    )
    model.articulation(
        "handle_to_lock_wheel",
        ArticulationType.REVOLUTE,
        parent=handle_body,
        child=lock_wheel,
        origin=Origin(xyz=(0.032, -0.014, 0.019)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.10,
            upper=1.10,
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

    handle_body = object_model.get_part("handle_body")
    blade_carriage = object_model.get_part("blade_carriage")
    lock_wheel = object_model.get_part("lock_wheel")
    slide_joint = object_model.get_articulation("handle_to_carriage")
    wheel_joint = object_model.get_articulation("handle_to_lock_wheel")

    ctx.check(
        "carriage slides along handle axis",
        slide_joint.axis == (1.0, 0.0, 0.0)
        and slide_joint.motion_limits is not None
        and slide_joint.motion_limits.lower == 0.0
        and slide_joint.motion_limits.upper is not None
        and slide_joint.motion_limits.upper >= 0.035,
        details=f"axis={slide_joint.axis}, limits={slide_joint.motion_limits}",
    )
    ctx.check(
        "lock wheel rotates on side-mounted shaft",
        wheel_joint.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0))
        and wheel_joint.motion_limits is not None
        and wheel_joint.motion_limits.lower is not None
        and wheel_joint.motion_limits.upper is not None
        and wheel_joint.motion_limits.lower < 0.0 < wheel_joint.motion_limits.upper,
        details=f"axis={wheel_joint.axis}, limits={wheel_joint.motion_limits}",
    )

    rest_carriage_pos = ctx.part_world_position(blade_carriage)
    rest_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="blade")

    with ctx.pose({slide_joint: 0.0}):
        ctx.expect_gap(
            blade_carriage,
            handle_body,
            axis="z",
            positive_elem="carriage_shoe",
            negative_elem="lower_spine",
            min_gap=0.0015,
            max_gap=0.0045,
            name="carriage shoe clears the lower spine",
        )
        ctx.expect_within(
            blade_carriage,
            handle_body,
            axes="yz",
            inner_elem="carriage_shoe",
            margin=0.0,
            name="carriage shoe fits between the handle walls at rest",
        )
        ctx.expect_overlap(
            blade_carriage,
            handle_body,
            axes="x",
            elem_a="carriage_shoe",
            min_overlap=0.070,
            name="carriage stays captured inside the handle at rest",
        )
        ctx.expect_gap(
            handle_body,
            lock_wheel,
            axis="y",
            positive_elem="side_plate_right",
            negative_elem="wheel_ring",
            min_gap=0.0,
            max_gap=0.0020,
            name="lock wheel sits just outside the right side plate",
        )
        ctx.expect_overlap(
            handle_body,
            lock_wheel,
            axes="xz",
            elem_a="side_plate_right",
            elem_b="wheel_ring",
            min_overlap=0.018,
            name="lock wheel overlaps the side-wall footprint",
        )

    extended_carriage_pos = None
    extended_blade_aabb = None
    slide_upper = 0.040
    if slide_joint.motion_limits is not None and slide_joint.motion_limits.upper is not None:
        slide_upper = slide_joint.motion_limits.upper

    with ctx.pose({slide_joint: slide_upper}):
        ctx.expect_within(
            blade_carriage,
            handle_body,
            axes="yz",
            inner_elem="carriage_shoe",
            margin=0.0,
            name="carriage shoe stays guided between the walls when extended",
        )
        ctx.expect_overlap(
            blade_carriage,
            handle_body,
            axes="x",
            elem_a="carriage_shoe",
            min_overlap=0.018,
            name="carriage keeps retained insertion at full extension",
        )
        extended_carriage_pos = ctx.part_world_position(blade_carriage)
        extended_blade_aabb = ctx.part_element_world_aabb(blade_carriage, elem="blade")

    ctx.check(
        "blade carriage moves forward when extended",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.030,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "blade projects farther from the nose at full extension",
        rest_blade_aabb is not None
        and extended_blade_aabb is not None
        and extended_blade_aabb[1][0] > rest_blade_aabb[1][0] + 0.030,
        details=f"rest={rest_blade_aabb}, extended={extended_blade_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
