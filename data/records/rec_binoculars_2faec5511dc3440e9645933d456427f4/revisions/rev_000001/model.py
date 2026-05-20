from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_between(part, a, b, radius: float, material, *, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_barrel_half(
    part,
    *,
    x: float,
    material_body,
    material_brass,
    material_glass,
    material_rubber,
    prefix: str,
) -> None:
    """Add one compact opera-glass optical barrel, aligned front/back on Y."""
    _add_cylinder_between(
        part,
        (x, -0.055, 0.0),
        (x, 0.055, 0.0),
        0.021,
        material_body,
        name=f"{prefix}_main_tube",
    )
    _add_cylinder_between(
        part,
        (x, 0.047, 0.0),
        (x, 0.073, 0.0),
        0.028,
        material_brass,
        name=f"{prefix}_front_bell",
    )
    _add_cylinder_between(
        part,
        (x, -0.069, 0.0),
        (x, -0.050, 0.0),
        0.017,
        material_brass,
        name=f"{prefix}_eyepiece_ring",
    )
    _add_cylinder_between(
        part,
        (x, -0.083, 0.0),
        (x, -0.068, 0.0),
        0.014,
        material_rubber,
        name=f"{prefix}_eye_cup",
    )
    _add_cylinder_between(
        part,
        (x, 0.073, 0.0),
        (x, 0.077, 0.0),
        0.022,
        material_glass,
        name=f"{prefix}_objective_lens",
    )
    _add_cylinder_between(
        part,
        (x, -0.086, 0.0),
        (x, -0.083, 0.0),
        0.011,
        material_glass,
        name=f"{prefix}_ocular_lens",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_opera_glass")

    brass = model.material("polished_brass", rgba=(0.86, 0.62, 0.24, 1.0))
    dark_leather = model.material("black_leatherette", rgba=(0.035, 0.032, 0.030, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.50, 0.75, 0.90, 0.42))
    shadow = model.material("dark_lens_shadow", rgba=(0.02, 0.024, 0.030, 1.0))

    focus_mesh = mesh_from_geometry(
        KnobGeometry(
            0.030,
            0.016,
            body_style="cylindrical",
            grip=KnobGrip(style="knurled", count=34, depth=0.00075, helix_angle_deg=18.0),
            edge_radius=0.0007,
        ),
        "crown_focus_wheel",
    )

    fixed_barrel = model.part("fixed_barrel")
    _add_barrel_half(
        fixed_barrel,
        x=-0.047,
        material_body=dark_leather,
        material_brass=brass,
        material_glass=glass,
        material_rubber=rubber,
        prefix="fixed",
    )

    # Split hinge leaves: the fixed half carries upper/lower knuckles and the
    # through pin; the moving half carries the middle knuckle.
    _add_cylinder_between(
        fixed_barrel,
        (0.0, 0.0, 0.012),
        (0.0, 0.0, 0.028),
        0.010,
        brass,
        name="upper_hinge_knuckle",
    )
    _add_cylinder_between(
        fixed_barrel,
        (0.0, 0.0, -0.028),
        (0.0, 0.0, -0.012),
        0.010,
        brass,
        name="lower_hinge_knuckle",
    )
    _add_cylinder_between(
        fixed_barrel,
        (0.0, 0.0, -0.032),
        (0.0, 0.0, 0.032),
        0.0045,
        brass,
        name="pivot_pin",
    )
    fixed_barrel.visual(
        Box((0.039, 0.012, 0.008)),
        origin=Origin(xyz=(-0.0275, 0.0, 0.020)),
        material=brass,
        name="upper_bridge",
    )
    fixed_barrel.visual(
        Box((0.039, 0.012, 0.008)),
        origin=Origin(xyz=(-0.0275, 0.0, -0.020)),
        material=brass,
        name="lower_bridge",
    )

    # A slim crown-wheel yoke rises from the hinge and reaches back between the
    # eyepieces, as on compact theatre/opera glasses.
    _add_cylinder_between(
        fixed_barrel,
        (0.0, 0.0, 0.028),
        (0.0, 0.0, 0.050),
        0.006,
        brass,
        name="focus_riser",
    )
    fixed_barrel.visual(
        Box((0.011, 0.054, 0.008)),
        origin=Origin(xyz=(0.0, -0.027, 0.047)),
        material=brass,
        name="focus_stem",
    )
    fixed_barrel.visual(
        Box((0.052, 0.011, 0.006)),
        origin=Origin(xyz=(0.0, -0.054, 0.050)),
        material=brass,
        name="focus_yoke_base",
    )
    fixed_barrel.visual(
        Box((0.006, 0.012, 0.030)),
        origin=Origin(xyz=(-0.022, -0.055, 0.065)),
        material=brass,
        name="focus_lug_0",
    )
    fixed_barrel.visual(
        Box((0.006, 0.012, 0.030)),
        origin=Origin(xyz=(0.022, -0.055, 0.065)),
        material=brass,
        name="focus_lug_1",
    )

    folding_barrel = model.part("folding_barrel")
    _add_barrel_half(
        folding_barrel,
        x=0.047,
        material_body=dark_leather,
        material_brass=brass,
        material_glass=glass,
        material_rubber=rubber,
        prefix="folding",
    )
    _add_cylinder_between(
        folding_barrel,
        (0.0, 0.0, -0.008),
        (0.0, 0.0, 0.008),
        0.010,
        brass,
        name="middle_hinge_knuckle",
    )
    folding_barrel.visual(
        Box((0.039, 0.012, 0.008)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
        material=brass,
        name="center_bridge",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.0035, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="focus_axle",
    )
    focus_wheel.visual(
        focus_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="knurled_crown",
    )
    focus_wheel.visual(
        Cylinder(radius=0.009, length=0.0025),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shadow,
        name="crown_recess",
    )

    model.articulation(
        "barrel_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_barrel,
        child=folding_barrel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=fixed_barrel,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, -0.055, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_barrel = object_model.get_part("fixed_barrel")
    folding_barrel = object_model.get_part("folding_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    barrel_hinge = object_model.get_articulation("barrel_hinge")
    focus_rotation = object_model.get_articulation("focus_rotation")

    ctx.allow_overlap(
        fixed_barrel,
        folding_barrel,
        elem_a="pivot_pin",
        elem_b="middle_hinge_knuckle",
        reason="The brass pivot pin is intentionally captured inside the folding barrel's hinge knuckle.",
    )
    ctx.expect_within(
        fixed_barrel,
        folding_barrel,
        axes="xy",
        inner_elem="pivot_pin",
        outer_elem="middle_hinge_knuckle",
        margin=0.0,
        name="pivot pin is centered inside hinge knuckle",
    )
    ctx.expect_overlap(
        fixed_barrel,
        folding_barrel,
        axes="z",
        elem_a="pivot_pin",
        elem_b="middle_hinge_knuckle",
        min_overlap=0.014,
        name="hinge knuckle remains captured on pivot pin",
    )

    ctx.expect_contact(
        focus_wheel,
        fixed_barrel,
        elem_a="focus_axle",
        elem_b="focus_lug_0",
        contact_tol=0.001,
        name="focus axle is seated in one yoke lug",
    )
    ctx.expect_contact(
        focus_wheel,
        fixed_barrel,
        elem_a="focus_axle",
        elem_b="focus_lug_1",
        contact_tol=0.001,
        name="focus axle is seated in opposite yoke lug",
    )

    rest_aabb = ctx.part_world_aabb(folding_barrel)
    with ctx.pose({barrel_hinge: 0.45}):
        folded_aabb = ctx.part_world_aabb(folding_barrel)
        ctx.expect_overlap(
            fixed_barrel,
            folding_barrel,
            axes="z",
            elem_a="pivot_pin",
            elem_b="middle_hinge_knuckle",
            min_overlap=0.014,
            name="folded hinge still retains pin",
        )

    rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    folded_center_y = None if folded_aabb is None else (folded_aabb[0][1] + folded_aabb[1][1]) * 0.5
    ctx.check(
        "folding barrel swings about central pivot",
        rest_center_y is not None
        and folded_center_y is not None
        and folded_center_y > rest_center_y + 0.015,
        details=f"rest_y={rest_center_y}, folded_y={folded_center_y}",
    )

    focus_limits = focus_rotation.motion_limits
    ctx.check(
        "crown focus wheel is rotary",
        focus_rotation.articulation_type == ArticulationType.CONTINUOUS
        and focus_limits is not None
        and focus_limits.velocity >= 1.0,
        details=f"type={focus_rotation.articulation_type}, limits={focus_limits}",
    )

    return ctx.report()


object_model = build_object_model()
