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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEAN_ANGLE = math.radians(10.0)
LADDER_DIR = (-math.sin(LEAN_ANGLE), 0.0, math.cos(LEAN_ANGLE))
RAIL_NORMAL = (math.cos(LEAN_ANGLE), 0.0, math.sin(LEAN_ANGLE))

BASE_SIZE = (0.18, 0.78, 0.018)
BASE_FEET_SIZE = (0.065, 0.082, 0.014)
BASE_TOP_Z = BASE_FEET_SIZE[2] + BASE_SIZE[2]

LOWER_RAIL_CENTER_Y = 0.182
LOWER_RAIL_SIZE = (0.070, 0.032, 1.30)
LOWER_HINGE_STRAP_SIZE = (0.026, 0.010, 0.12)
LOWER_HINGE_STRAP_CENTER_Y = 0.195

UPPER_RAIL_CENTER_Y = 0.176
UPPER_RAIL_SIZE = (0.066, 0.030, 1.02)
UPPER_HINGE_STRAP_SIZE = (0.024, 0.020, 0.12)

FLY_RAIL_CENTER_Y = 0.149
FLY_RAIL_SIZE = (0.050, 0.024, 1.18)
UPPER_GUIDE_BLOCK_SIZE = (0.008, 0.014, 0.18)
UPPER_GUIDE_BLOCK_CENTER_Y = FLY_RAIL_CENTER_Y + FLY_RAIL_SIZE[1] * 0.5 + UPPER_GUIDE_BLOCK_SIZE[1] * 0.5
UPPER_GUIDE_BLOCK_CENTER_DISTANCE = 0.90
UPPER_GUIDE_BLOCK_NORMAL_OFFSET = UPPER_RAIL_SIZE[0] * 0.5 + UPPER_GUIDE_BLOCK_SIZE[0] * 0.5
FLY_STACK_OFFSET_NORMAL = (
    UPPER_RAIL_SIZE[0] * 0.5 + UPPER_GUIDE_BLOCK_SIZE[0] + FLY_RAIL_SIZE[0] * 0.5
)

HINGE_BARREL_RADIUS = 0.015
HINGE_BARREL_LENGTH = 0.020
LOWER_BARREL_CENTER_Y = 0.192
UPPER_BARREL_CENTER_Y = 0.172

HINGE_DISTANCE = LOWER_RAIL_SIZE[2] + LOWER_HINGE_STRAP_SIZE[2]
SLIDE_ORIGIN_DISTANCE = 0.20
MAX_FLY_EXTENSION = 0.45

RAIL_RPY = (0.0, -LEAN_ANGLE, 0.0)
RUNG_RPY = (math.pi / 2.0, 0.0, 0.0)
BARREL_RPY = (math.pi / 2.0, 0.0, 0.0)


def _point_along(
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        start[0] + direction[0] * distance,
        start[1] + direction[1] * distance,
        start[2] + direction[2] * distance,
    )


def _rail_center(
    bottom: tuple[float, float, float],
    direction: tuple[float, float, float],
    length: float,
) -> tuple[float, float, float]:
    return _point_along(bottom, direction, length * 0.5)


def _ladder_space_point(
    *,
    distance: float,
    y: float = 0.0,
    normal_offset: float = 0.0,
) -> tuple[float, float, float]:
    return (
        RAIL_NORMAL[0] * normal_offset + LADDER_DIR[0] * distance,
        y,
        RAIL_NORMAL[2] * normal_offset + LADDER_DIR[2] * distance,
    )


def _add_rungs(
    part,
    *,
    bottom_point: tuple[float, float, float],
    positions: tuple[float, ...],
    rung_length: float,
    rung_radius: float,
    material,
    prefix: str,
) -> None:
    for index, distance in enumerate(positions):
        rung_center = _point_along(bottom_point, LADDER_DIR, distance)
        part.visual(
            Cylinder(radius=rung_radius, length=rung_length),
            origin=Origin(xyz=rung_center, rpy=RUNG_RPY),
            material=material,
            name=f"{prefix}_rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="loft_access_ladder")

    aluminium = model.material("aluminium", rgba=(0.78, 0.80, 0.82, 1.0))
    aluminium_dark = model.material("aluminium_dark", rgba=(0.66, 0.68, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    nylon = model.material("nylon", rgba=(0.17, 0.17, 0.18, 1.0))

    lower_section = model.part("lower_section")
    lower_section.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_FEET_SIZE[2] + BASE_SIZE[2] * 0.5)),
        material=aluminium_dark,
        name="base_plate",
    )
    for x in (-0.045, 0.045):
        for y in (-0.305, 0.305):
            lower_section.visual(
                Box(BASE_FEET_SIZE),
                origin=Origin(xyz=(x, y, BASE_FEET_SIZE[2] * 0.5)),
                material=rubber,
                name=f"foot_{'front' if x > 0 else 'rear'}_{'right' if y > 0 else 'left'}",
            )

    hinge_origin_xyz = _point_along((0.0, 0.0, BASE_TOP_Z), LADDER_DIR, HINGE_DISTANCE)

    for side_name, y_sign in (("right", 1.0), ("left", -1.0)):
        rail_bottom = (0.0, y_sign * LOWER_RAIL_CENTER_Y, BASE_TOP_Z)
        strap_bottom = _point_along(rail_bottom, LADDER_DIR, LOWER_RAIL_SIZE[2])
        barrel_center_y = y_sign * LOWER_BARREL_CENTER_Y

        lower_section.visual(
            Box(LOWER_RAIL_SIZE),
            origin=Origin(
                xyz=_rail_center(rail_bottom, LADDER_DIR, LOWER_RAIL_SIZE[2]),
                rpy=RAIL_RPY,
            ),
            material=aluminium,
            name=f"lower_{side_name}_rail",
        )
        lower_section.visual(
            Box(LOWER_HINGE_STRAP_SIZE),
            origin=Origin(
                xyz=(
                    _rail_center(strap_bottom, LADDER_DIR, LOWER_HINGE_STRAP_SIZE[2])[0],
                    y_sign * LOWER_HINGE_STRAP_CENTER_Y,
                    _rail_center(strap_bottom, LADDER_DIR, LOWER_HINGE_STRAP_SIZE[2])[2],
                ),
                rpy=RAIL_RPY,
            ),
            material=aluminium_dark,
            name=f"lower_{side_name}_hinge_strap",
        )
        lower_section.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(hinge_origin_xyz[0], barrel_center_y, hinge_origin_xyz[2]), rpy=BARREL_RPY),
            material=nylon,
            name=f"lower_{side_name}_barrel",
        )

    _add_rungs(
        lower_section,
        bottom_point=(0.0, 0.0, BASE_TOP_Z),
        positions=(0.22, 0.50, 0.78, 1.06, 1.24),
        rung_length=2.0 * (LOWER_RAIL_CENTER_Y - LOWER_RAIL_SIZE[1] * 0.5),
        rung_radius=0.014,
        material=aluminium_dark,
        prefix="lower",
    )
    lower_section.inertial = Inertial.from_geometry(
        Box((0.50, 0.78, 1.50)),
        mass=8.5,
        origin=Origin(xyz=(-0.10, 0.0, 0.75)),
    )

    upper_section = model.part("upper_section")
    for side_name, y_sign in (("right", 1.0), ("left", -1.0)):
        strap_bottom = (0.0, y_sign * UPPER_BARREL_CENTER_Y, 0.0)
        rail_bottom = _point_along((0.0, y_sign * UPPER_RAIL_CENTER_Y, 0.0), LADDER_DIR, UPPER_HINGE_STRAP_SIZE[2])
        barrel_center_y = y_sign * UPPER_BARREL_CENTER_Y

        upper_section.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(0.0, barrel_center_y, 0.0), rpy=BARREL_RPY),
            material=nylon,
            name=f"upper_{side_name}_barrel",
        )
        upper_section.visual(
            Box(UPPER_HINGE_STRAP_SIZE),
            origin=Origin(
                xyz=_rail_center(strap_bottom, LADDER_DIR, UPPER_HINGE_STRAP_SIZE[2]),
                rpy=RAIL_RPY,
            ),
            material=aluminium_dark,
            name=f"upper_{side_name}_hinge_strap",
        )
        upper_section.visual(
            Box(UPPER_RAIL_SIZE),
            origin=Origin(
                xyz=_rail_center(rail_bottom, LADDER_DIR, UPPER_RAIL_SIZE[2]),
                rpy=RAIL_RPY,
            ),
            material=aluminium,
            name=f"upper_{side_name}_rail",
        )
        upper_section.visual(
            Box(UPPER_GUIDE_BLOCK_SIZE),
            origin=Origin(
                xyz=_ladder_space_point(
                    distance=UPPER_GUIDE_BLOCK_CENTER_DISTANCE,
                    y=y_sign * UPPER_GUIDE_BLOCK_CENTER_Y,
                    normal_offset=UPPER_GUIDE_BLOCK_NORMAL_OFFSET,
                ),
                rpy=RAIL_RPY,
            ),
            material=nylon,
            name=f"upper_{side_name}_guide_block",
        )

    _add_rungs(
        upper_section,
        bottom_point=_point_along((0.0, 0.0, 0.0), LADDER_DIR, UPPER_HINGE_STRAP_SIZE[2]),
        positions=(0.18, 0.46, 0.74, 0.96),
        rung_length=2.0 * (UPPER_RAIL_CENTER_Y - UPPER_RAIL_SIZE[1] * 0.5),
        rung_radius=0.013,
        material=aluminium_dark,
        prefix="upper",
    )
    top_cap_center = _point_along(
        (0.0, 0.0, 0.0),
        LADDER_DIR,
        UPPER_HINGE_STRAP_SIZE[2] + UPPER_RAIL_SIZE[2],
    )
    for side_name, y_sign in (("right", 1.0), ("left", -1.0)):
        upper_section.visual(
            Box((0.030, 0.036, 0.020)),
            origin=Origin(
                xyz=(top_cap_center[0], y_sign * UPPER_RAIL_CENTER_Y, top_cap_center[2]),
                rpy=RAIL_RPY,
            ),
            material=aluminium_dark,
            name=f"upper_{side_name}_top_cap",
        )
    upper_section.inertial = Inertial.from_geometry(
        Box((0.34, 0.38, 1.20)),
        mass=4.2,
        origin=Origin(xyz=(-0.08, 0.0, 0.60)),
    )

    fly_section = model.part("fly_section")
    for side_name, y_sign in (("right", 1.0), ("left", -1.0)):
        fly_bottom = (0.0, y_sign * FLY_RAIL_CENTER_Y, 0.0)
        fly_section.visual(
            Box(FLY_RAIL_SIZE),
            origin=Origin(
                xyz=_rail_center(fly_bottom, LADDER_DIR, FLY_RAIL_SIZE[2]),
                rpy=RAIL_RPY,
            ),
            material=aluminium,
            name=f"fly_{side_name}_rail",
        )
    _add_rungs(
        fly_section,
        bottom_point=(0.0, 0.0, 0.0),
        positions=(0.14, 0.42, 0.70, 0.98),
        rung_length=2.0 * (FLY_RAIL_CENTER_Y - FLY_RAIL_SIZE[1] * 0.5),
        rung_radius=0.012,
        material=aluminium_dark,
        prefix="fly",
    )
    fly_section.visual(
        Box((0.026, 0.286, 0.018)),
        origin=Origin(
            xyz=_point_along((0.0, 0.0, 0.0), LADDER_DIR, FLY_RAIL_SIZE[2]),
            rpy=RAIL_RPY,
        ),
        material=aluminium_dark,
        name="fly_top_cap",
    )
    fly_section.inertial = Inertial.from_geometry(
        Box((0.28, 0.32, 1.24)),
        mass=3.3,
        origin=Origin(xyz=(-0.08, 0.0, 0.62)),
    )

    model.articulation(
        "mid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_section,
        child=upper_section,
        origin=Origin(xyz=hinge_origin_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=2.60,
        ),
    )
    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=upper_section,
        child=fly_section,
        origin=Origin(
            xyz=_ladder_space_point(
                distance=SLIDE_ORIGIN_DISTANCE,
                normal_offset=FLY_STACK_OFFSET_NORMAL,
            )
        ),
        axis=LADDER_DIR,
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.22,
            lower=0.0,
            upper=MAX_FLY_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
    lower_section = object_model.get_part("lower_section")
    upper_section = object_model.get_part("upper_section")
    fly_section = object_model.get_part("fly_section")
    mid_hinge = object_model.get_articulation("mid_hinge")
    fly_slide = object_model.get_articulation("fly_slide")

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mid_hinge_axis_lateral",
        tuple(mid_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"mid_hinge axis was {mid_hinge.axis}",
    )
    ctx.check(
        "fly_slide_axis_aligned_with_rails",
        all(abs(a - b) < 1e-6 for a, b in zip(fly_slide.axis, LADDER_DIR)),
        details=f"fly_slide axis was {fly_slide.axis}, expected {LADDER_DIR}",
    )

    ctx.expect_contact(
        lower_section,
        upper_section,
        elem_a="lower_right_barrel",
        elem_b="upper_right_barrel",
        name="hinge_barrels_touch_in_open_pose",
    )
    ctx.expect_contact(
        upper_section,
        fly_section,
        elem_a="upper_right_guide_block",
        elem_b="fly_right_rail",
        name="fly_rails_are_guided_by_upper_section",
    )
    ctx.expect_overlap(upper_section, fly_section, axes="y", min_overlap=0.28, name="fly_stays_between_upper_rails")

    lower_aabb = ctx.part_world_aabb(lower_section)
    fly_aabb = ctx.part_world_aabb(fly_section)
    ctx.check(
        "ladder_open_height_is_realistic",
        lower_aabb is not None and fly_aabb is not None and (fly_aabb[1][2] - lower_aabb[0][2]) > 2.7,
        details=f"lower_aabb={lower_aabb}, fly_aabb={fly_aabb}",
    )

    fly_rest = ctx.part_world_position(fly_section)
    assert fly_rest is not None
    with ctx.pose({fly_slide: MAX_FLY_EXTENSION}):
        fly_extended = ctx.part_world_position(fly_section)
        assert fly_extended is not None
        ctx.expect_contact(
            upper_section,
            fly_section,
            elem_a="upper_right_guide_block",
            elem_b="fly_right_rail",
            name="fly_stays_guided_when_extended",
        )
        ctx.check(
            "fly_section_extends_upward",
            fly_extended[2] > fly_rest[2] + 0.40,
            details=f"rest={fly_rest}, extended={fly_extended}",
        )
        ctx.check(
            "fly_section_extends_along_ladder_pitch",
            fly_extended[0] < fly_rest[0] - 0.06,
            details=f"rest={fly_rest}, extended={fly_extended}",
        )

    upper_open_aabb = ctx.part_world_aabb(upper_section)
    assert upper_open_aabb is not None
    with ctx.pose({mid_hinge: 1.70}):
        upper_folded_aabb = ctx.part_world_aabb(upper_section)
        assert upper_folded_aabb is not None
        ctx.expect_contact(
            lower_section,
            upper_section,
            elem_a="lower_right_barrel",
            elem_b="upper_right_barrel",
            name="hinge_barrels_stay_seated_when_folded",
        )
        ctx.check(
            "upper_section_folds_forward",
            upper_folded_aabb[1][0] > upper_open_aabb[1][0] + 0.55,
            details=f"open={upper_open_aabb}, folded={upper_folded_aabb}",
        )
        ctx.check(
            "upper_section_drops_toward_stowed_height",
            upper_folded_aabb[1][2] < upper_open_aabb[1][2] - 0.65,
            details=f"open={upper_open_aabb}, folded={upper_folded_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
