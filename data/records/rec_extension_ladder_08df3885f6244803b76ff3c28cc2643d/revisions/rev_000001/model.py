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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _lerp(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _rotate_x(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _unit(
    v: tuple[float, float, float]
) -> tuple[float, float, float]:
    length = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if length <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _add_box_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def _add_cylinder_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_a_frame_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.62, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.18, 0.18, 0.19, 1.0))
    molded_plastic = model.material(
        "molded_plastic", rgba=(0.24, 0.26, 0.28, 1.0)
    )
    safety_orange = model.material("safety_orange", rgba=(0.85, 0.36, 0.12, 1.0))

    open_angle = 0.36

    front = model.part("front_section")
    rear = model.part("rear_section")
    fly = model.part("fly_section")
    brace = model.part("spreader_brace")

    front_left_top = (-0.055, 0.0, -0.03)
    front_left_bottom = (-0.175, 0.0, -1.72)
    front_right_top = (0.055, 0.0, -0.03)
    front_right_bottom = (0.175, 0.0, -1.72)

    front.visual(
        Box((0.046, 0.028, _distance(front_left_top, front_left_bottom))),
        origin=Origin(
            xyz=_midpoint(front_left_top, front_left_bottom),
            rpy=_rpy_for_z_axis(front_left_top, front_left_bottom),
        ),
        material=aluminum,
        name="front_left_rail",
    )
    front.visual(
        Box((0.046, 0.028, _distance(front_right_top, front_right_bottom))),
        origin=Origin(
            xyz=_midpoint(front_right_top, front_right_bottom),
            rpy=_rpy_for_z_axis(front_right_top, front_right_bottom),
        ),
        material=aluminum,
        name="front_right_rail",
    )
    front.visual(
        Box((0.18, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, -0.024, -0.050)),
        material=molded_plastic,
        name="front_top_cap",
    )

    for index, t in enumerate((0.18, 0.33, 0.48, 0.63, 0.78), start=1):
        left_pt = _lerp(front_left_top, front_left_bottom, t)
        right_pt = _lerp(front_right_top, front_right_bottom, t)
        tread_width = (right_pt[0] - left_pt[0]) - 0.018
        front.visual(
            Box((tread_width, 0.082, 0.032)),
            origin=Origin(xyz=(0.0, -0.030, left_pt[2])),
            material=aluminum,
            name=f"front_tread_{index}",
        )

    front.visual(
        Box((0.36, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -1.665)),
        material=aluminum,
        name="front_bottom_tie",
    )
    front.visual(
        Box((0.070, 0.028, 0.034)),
        origin=Origin(xyz=(-0.155, 0.0, -1.720)),
        material=dark_rubber,
        name="front_left_foot",
    )
    front.visual(
        Box((0.070, 0.028, 0.034)),
        origin=Origin(xyz=(0.155, 0.0, -1.720)),
        material=dark_rubber,
        name="front_right_foot",
    )

    for name, rail_point in (
        ("front_upper_left_guide", _lerp(front_left_top, front_left_bottom, 0.22)),
        ("front_upper_right_guide", _lerp(front_right_top, front_right_bottom, 0.22)),
        ("front_lower_left_guide", _lerp(front_left_top, front_left_bottom, 0.73)),
        ("front_lower_right_guide", _lerp(front_right_top, front_right_bottom, 0.73)),
    ):
        guide_x, _, guide_z = rail_point
        front.visual(
            Box((0.040, 0.052, 0.150)),
            origin=Origin(xyz=(guide_x, -0.040, guide_z)),
            material=steel,
            name=name,
        )

    front.visual(
        Box((0.022, 0.014, 0.090)),
        origin=Origin(xyz=(0.147, 0.006, -0.920)),
        material=safety_orange,
        name="front_brace_mount",
    )
    front.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(
            xyz=(-0.040, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="front_left_hinge_knuckle",
    )
    front.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(
            xyz=(0.040, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="front_right_hinge_knuckle",
    )
    _add_box_beam(
        front,
        front_left_top,
        (-0.040, 0.0, 0.0),
        width=0.018,
        depth=0.014,
        material=molded_plastic,
        name="front_left_hinge_cheek",
    )
    _add_box_beam(
        front,
        front_right_top,
        (0.040, 0.0, 0.0),
        width=0.018,
        depth=0.014,
        material=molded_plastic,
        name="front_right_hinge_cheek",
    )

    rear_plane_y = 0.055
    rear_left_top = (-0.045, rear_plane_y, -0.03)
    rear_left_bottom = (-0.145, rear_plane_y, -1.66)
    rear_right_top = (0.045, rear_plane_y, -0.03)
    rear_right_bottom = (0.145, rear_plane_y, -1.66)

    _add_box_beam(
        rear,
        rear_left_top,
        rear_left_bottom,
        width=0.040,
        depth=0.026,
        material=aluminum,
        name="rear_left_rail",
    )
    _add_box_beam(
        rear,
        rear_right_top,
        rear_right_bottom,
        width=0.040,
        depth=0.026,
        material=aluminum,
        name="rear_right_rail",
    )
    rear.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(
            xyz=(0.0, rear_plane_y, -0.040),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=molded_plastic,
        name="rear_top_yoke",
    )
    for index, t in enumerate((0.22, 0.46, 0.69, 0.88), start=1):
        left_pt = _lerp(rear_left_top, rear_left_bottom, t)
        right_pt = _lerp(rear_right_top, rear_right_bottom, t)
        rear.visual(
            Box(((right_pt[0] - left_pt[0]) - 0.010, 0.028, 0.026)),
            origin=Origin(xyz=(0.0, rear_plane_y, left_pt[2])),
            material=aluminum,
            name=f"rear_brace_{index}",
        )
    rear.visual(
        Box((0.290, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, rear_plane_y, -1.615)),
        material=aluminum,
        name="rear_bottom_tie",
    )
    rear.visual(
        Box((0.060, 0.026, 0.032)),
        origin=Origin(xyz=(-0.125, rear_plane_y, -1.660)),
        material=dark_rubber,
        name="rear_left_foot",
    )
    rear.visual(
        Box((0.060, 0.026, 0.032)),
        origin=Origin(xyz=(0.125, rear_plane_y, -1.660)),
        material=dark_rubber,
        name="rear_right_foot",
    )

    rear_pin_surface_local = (0.112, 0.028, -0.960)
    rear_pin_surface_world = _rotate_x(rear_pin_surface_local, open_angle)
    brace_mount_world = (0.147, 0.040, -0.920)
    brace_dir_world = _unit(
        (
            rear_pin_surface_world[0] - brace_mount_world[0],
            rear_pin_surface_world[1] - brace_mount_world[1],
            rear_pin_surface_world[2] - brace_mount_world[2],
        )
    )
    pin_radius = 0.010
    rear_pin_center_world = (
        rear_pin_surface_world[0] + brace_dir_world[0] * pin_radius,
        rear_pin_surface_world[1] + brace_dir_world[1] * pin_radius,
        rear_pin_surface_world[2] + brace_dir_world[2] * pin_radius,
    )
    rear_pin_center_local = _rotate_x(rear_pin_center_world, -open_angle)
    rear.visual(
        Cylinder(radius=pin_radius, length=0.040),
        origin=Origin(
            xyz=rear_pin_center_local,
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="rear_brace_pin",
    )
    rear.visual(
        Box((0.022, 0.018, 0.060)),
        origin=Origin(
            xyz=(
                rear_pin_center_local[0],
                rear_pin_center_local[1] + 0.019,
                rear_pin_center_local[2] - 0.040,
            )
        ),
        material=safety_orange,
        name="rear_brace_lug",
    )

    rear.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="rear_center_hinge_barrel",
    )
    rear.visual(
        Box((0.042, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.016, -0.002)),
        material=molded_plastic,
        name="rear_hinge_web",
    )
    _add_box_beam(
        rear,
        rear_left_top,
        (-0.016, 0.018, -0.004),
        width=0.016,
        depth=0.010,
        material=molded_plastic,
        name="rear_left_hinge_cheek",
    )
    _add_box_beam(
        rear,
        rear_right_top,
        (0.016, 0.018, -0.004),
        width=0.016,
        depth=0.010,
        material=molded_plastic,
        name="rear_right_hinge_cheek",
    )

    fly_left_top = (-0.045, -0.080, 0.62)
    fly_left_bottom = (-0.125, -0.080, -1.72)
    fly_right_top = (0.045, -0.080, 0.62)
    fly_right_bottom = (0.125, -0.080, -1.72)

    fly.visual(
        Box((0.034, 0.020, _distance(fly_left_top, fly_left_bottom))),
        origin=Origin(
            xyz=_midpoint(fly_left_top, fly_left_bottom),
            rpy=_rpy_for_z_axis(fly_left_top, fly_left_bottom),
        ),
        material=aluminum,
        name="fly_left_rail",
    )
    fly.visual(
        Box((0.034, 0.020, _distance(fly_right_top, fly_right_bottom))),
        origin=Origin(
            xyz=_midpoint(fly_right_top, fly_right_bottom),
            rpy=_rpy_for_z_axis(fly_right_top, fly_right_bottom),
        ),
        material=aluminum,
        name="fly_right_rail",
    )
    for index, t in enumerate((0.16, 0.28, 0.40, 0.52, 0.64, 0.76, 0.88), start=1):
        left_pt = _lerp(fly_left_top, fly_left_bottom, t)
        right_pt = _lerp(fly_right_top, fly_right_bottom, t)
        _add_cylinder_member(
            fly,
            left_pt,
            right_pt,
            radius=0.012,
            material=aluminum,
            name=f"fly_rung_{index}",
        )
    fly.visual(
        Box((0.110, 0.022, 0.055)),
        origin=Origin(xyz=(0.0, -0.092, 0.565)),
        material=safety_orange,
        name="fly_top_lock",
    )

    brace_mount_local = (0.0, 0.0, 0.0)
    brace_tip_contact_local = (
        rear_pin_surface_world[0] - brace_mount_world[0],
        rear_pin_surface_world[1] - brace_mount_world[1],
        rear_pin_surface_world[2] - brace_mount_world[2],
    )
    brace_tip_base_local = (
        brace_tip_contact_local[0] - brace_dir_world[0] * 0.050,
        brace_tip_contact_local[1] - brace_dir_world[1] * 0.050,
        brace_tip_contact_local[2] - brace_dir_world[2] * 0.050,
    )
    _add_box_beam(
        brace,
        brace_mount_local,
        brace_tip_base_local,
        width=0.018,
        depth=0.008,
        material=steel,
        name="brace_bar",
    )
    brace.visual(
        Box((0.026, 0.010, _distance(brace_tip_base_local, brace_tip_contact_local))),
        origin=Origin(
            xyz=_midpoint(brace_tip_base_local, brace_tip_contact_local),
            rpy=_rpy_for_z_axis(brace_tip_base_local, brace_tip_contact_local),
        ),
        material=safety_orange,
        name="brace_tip",
    )
    brace.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="brace_hinge_collar",
    )

    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(open_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.18,
        ),
    )
    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=front,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.45,
            lower=-0.10,
            upper=0.48,
        ),
    )
    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=brace,
        origin=Origin(xyz=brace_mount_world),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.12,
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

    front = object_model.get_part("front_section")
    rear = object_model.get_part("rear_section")
    fly = object_model.get_part("fly_section")
    brace = object_model.get_part("spreader_brace")

    apex_hinge = object_model.get_articulation("apex_hinge")
    fly_slide = object_model.get_articulation("fly_slide")
    brace_hinge = object_model.get_articulation("brace_hinge")

    ctx.check(
        "ladder parts are present",
        all(part is not None for part in (front, rear, fly, brace)),
    )
    ctx.check(
        "ladder joints are present",
        all(joint is not None for joint in (apex_hinge, fly_slide, brace_hinge)),
    )

    ctx.expect_within(
        fly,
        front,
        axes="x",
        margin=0.010,
        name="fly stays centered between the front rails",
    )
    ctx.expect_overlap(
        fly,
        front,
        axes="z",
        elem_a="fly_left_rail",
        elem_b="front_left_rail",
        min_overlap=1.20,
        name="fly remains retained in the front guides at rest",
    )
    ctx.expect_contact(
        brace,
        rear,
        elem_a="brace_tip",
        elem_b="rear_brace_pin",
        contact_tol=0.010,
        name="open spreader brace bears on the rear brace pin",
    )

    fly_rest_aabb = ctx.part_element_world_aabb(fly, elem="fly_right_rail")
    with ctx.pose({fly_slide: 0.42}):
        fly_extended_aabb = ctx.part_element_world_aabb(fly, elem="fly_right_rail")
        ctx.expect_overlap(
            fly,
            front,
            axes="z",
            elem_a="fly_right_rail",
            elem_b="front_right_rail",
            min_overlap=0.82,
            name="extended fly still keeps retained insertion",
        )
    ctx.check(
        "fly extends upward from the front section",
        fly_rest_aabb is not None
        and fly_extended_aabb is not None
        and fly_extended_aabb[1][2] > fly_rest_aabb[1][2] + 0.30,
        details=f"rest={fly_rest_aabb}, extended={fly_extended_aabb}",
    )

    rear_open_aabb = ctx.part_element_world_aabb(rear, elem="rear_right_rail")
    with ctx.pose({apex_hinge: -0.30, brace_hinge: -0.95}):
        rear_folded_aabb = ctx.part_element_world_aabb(rear, elem="rear_right_rail")
        ctx.expect_gap(
            brace,
            rear,
            axis="y",
            positive_elem="brace_tip",
            negative_elem="rear_brace_pin",
            min_gap=0.040,
            name="brace tip swings clear of the rear pin when folding",
        )
    ctx.check(
        "rear section folds toward the front section",
        rear_open_aabb is not None
        and rear_folded_aabb is not None
        and rear_folded_aabb[1][1] < rear_open_aabb[1][1] - 0.12,
        details=f"open={rear_open_aabb}, folded={rear_folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
