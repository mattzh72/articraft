from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.300
BASE_WIDTH = 0.180
BASE_THICKNESS = 0.024

RAIL_LENGTH = 0.220
RAIL_BASE_WIDTH = 0.055
RAIL_BASE_HEIGHT = 0.009
RAIL_TOP_WIDTH = 0.036
RAIL_TOP_HEIGHT = 0.009
RAIL_HEIGHT = RAIL_BASE_HEIGHT + RAIL_TOP_HEIGHT

CARRIAGE_LENGTH = 0.082
CARRIAGE_CORE_WIDTH = 0.060
CARRIAGE_FULL_WIDTH = 0.074
CARRIAGE_HEIGHT = 0.026
GUIDE_SKIRT_WIDTH = 0.010
GUIDE_SKIRT_HEIGHT = 0.018
GUIDE_SKIRT_OFFSET_Y = 0.032

MAST_WIDTH = 0.048
MAST_DEPTH = 0.036
MAST_HEIGHT = 0.270
MAST_WALL = 0.004

BRACKET_WIDTH = 0.074
BRACKET_DEPTH = 0.052
BRACKET_HEIGHT = 0.040
BRACKET_FRONT_WEB = 0.008
BRACKET_SLOT_WIDTH = MAST_WIDTH + 0.003
BRACKET_SLOT_DEPTH = BRACKET_DEPTH - BRACKET_FRONT_WEB
BRACKET_SLOT_CENTER_Y = (-BRACKET_DEPTH / 2.0 + (MAST_DEPTH / 2.0)) / 2.0
TOP_PLATE_WIDTH = 0.070
TOP_PLATE_DEPTH = 0.034
TOP_PLATE_THICKNESS = 0.008
TOP_PLATE_SHIFT_Y = 0.006

LOWER_SLIDE_TRAVEL = 0.066
UPPER_SLIDE_ORIGIN_Z = 0.185
UPPER_SLIDE_LOWER = -0.060
UPPER_SLIDE_UPPER = 0.050


def make_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints([(0.0, 0.050), (0.0, -0.050)])
        .rect(BASE_LENGTH - 0.070, 0.028)
        .cutBlind(-0.003)
    )
    return base


def make_rail_shape() -> cq.Workplane:
    rail_base = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_BASE_WIDTH,
        RAIL_BASE_HEIGHT,
        centered=(True, True, False),
    )
    rail_top = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_TOP_WIDTH,
        RAIL_TOP_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, RAIL_BASE_HEIGHT))
    rail = rail_base.union(rail_top)
    rail = rail.faces(">Z").edges("|X").fillet(0.0015)
    return rail


def make_mast_carriage_shape() -> cq.Workplane:
    carriage_core = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_CORE_WIDTH,
        CARRIAGE_HEIGHT,
        centered=(True, True, False),
    )
    left_skirt = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        GUIDE_SKIRT_WIDTH,
        GUIDE_SKIRT_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, GUIDE_SKIRT_OFFSET_Y, 0.0))
    right_skirt = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        GUIDE_SKIRT_WIDTH,
        GUIDE_SKIRT_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, -GUIDE_SKIRT_OFFSET_Y, 0.0))
    return carriage_core.union(left_skirt).union(right_skirt)


def make_mast_tube_shape() -> cq.Workplane:
    mast_outer = cq.Workplane("XY").box(
        MAST_WIDTH,
        MAST_DEPTH,
        MAST_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, CARRIAGE_HEIGHT))
    mast_inner = cq.Workplane("XY").box(
        MAST_WIDTH - 2.0 * MAST_WALL,
        MAST_DEPTH - 2.0 * MAST_WALL,
        MAST_HEIGHT - MAST_WALL,
        centered=(True, True, False),
    ).translate((0.0, 0.0, CARRIAGE_HEIGHT))
    return mast_outer.cut(mast_inner)


def make_top_bracket_shape() -> cq.Workplane:
    pad_width = 0.052
    pad_depth = 0.008
    pad_height = 0.032
    cheek_width = 0.010
    cheek_depth = 0.018
    cheek_height = BRACKET_HEIGHT
    cheek_center_y = 0.032
    top_plate_center_y = 0.038

    rear_pad = cq.Workplane("XY").box(
        pad_width,
        pad_depth,
        pad_height,
        centered=(True, True, False),
    ).translate((0.0, MAST_DEPTH / 2.0 + pad_depth / 2.0, 0.0))

    cheek_x = MAST_WIDTH / 2.0 + cheek_width / 2.0
    left_cheek = cq.Workplane("XY").box(
        cheek_width,
        cheek_depth,
        cheek_height,
        centered=(True, True, False),
    ).translate((cheek_x, cheek_center_y, 0.0))
    right_cheek = cq.Workplane("XY").box(
        cheek_width,
        cheek_depth,
        cheek_height,
        centered=(True, True, False),
    ).translate((-cheek_x, cheek_center_y, 0.0))

    top_plate = cq.Workplane("XY").box(
        TOP_PLATE_WIDTH,
        TOP_PLATE_DEPTH,
        TOP_PLATE_THICKNESS,
        centered=(True, True, False),
    ).translate((0.0, top_plate_center_y, BRACKET_HEIGHT))
    top_plate = (
        top_plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .hole(0.0068, TOP_PLATE_THICKNESS)
    )

    return rear_pad.union(left_cheek).union(right_cheek).union(top_plate)


def _joint_kind_name(joint_type: object) -> str:
    return getattr(joint_type, "name", str(joint_type)).split(".")[-1]


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        maxs[0] - mins[0],
        maxs[1] - mins[1],
        maxs[2] - mins[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="optics_lift_stage")

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    mast_black = model.material("mast_black", rgba=(0.07, 0.07, 0.08, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "optics_lift_stage_base"),
        origin=Origin(),
        material=base_black,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(make_rail_shape(), "optics_lift_stage_rail"),
        origin=Origin(),
        material=rail_steel,
        name="rail_shell",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_HEIGHT)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
    )

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(make_mast_carriage_shape(), "optics_lift_stage_carriage"),
        origin=Origin(),
        material=mast_black,
        name="carriage_body",
    )
    mast.visual(
        mesh_from_cadquery(make_mast_tube_shape(), "optics_lift_stage_mast_tube"),
        origin=Origin(),
        material=mast_black,
        name="mast_tube",
    )
    mast.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_FULL_WIDTH, CARRIAGE_HEIGHT + MAST_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, (CARRIAGE_HEIGHT + MAST_HEIGHT) / 2.0)),
    )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        mesh_from_cadquery(make_top_bracket_shape(), "optics_lift_stage_top_bracket"),
        origin=Origin(),
        material=bracket_silver,
        name="bracket_shell",
    )
    top_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, BRACKET_HEIGHT + TOP_PLATE_THICKNESS)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (BRACKET_HEIGHT + TOP_PLATE_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_rail",
        ArticulationType.FIXED,
        parent=base,
        child=rail,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "rail_to_mast_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=-LOWER_SLIDE_TRAVEL,
            upper=LOWER_SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "mast_to_bracket_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=UPPER_SLIDE_LOWER,
            upper=UPPER_SLIDE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rail = object_model.get_part("rail")
    mast = object_model.get_part("mast")
    top_bracket = object_model.get_part("top_bracket")
    slide_x = object_model.get_articulation("rail_to_mast_slide")
    slide_z = object_model.get_articulation("mast_to_bracket_slide")

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
        "part_count",
        len(object_model.parts) == 4,
        f"Expected 4 parts but found {len(object_model.parts)}.",
    )
    ctx.check(
        "lower_slide_is_prismatic_x",
        _joint_kind_name(slide_x.joint_type) == "PRISMATIC" and tuple(slide_x.axis) == (1.0, 0.0, 0.0),
        f"Expected X-axis prismatic lower slide, got type={slide_x.joint_type} axis={slide_x.axis}.",
    )
    ctx.check(
        "upper_slide_is_prismatic_z",
        _joint_kind_name(slide_z.joint_type) == "PRISMATIC" and tuple(slide_z.axis) == (0.0, 0.0, 1.0),
        f"Expected Z-axis prismatic upper slide, got type={slide_z.joint_type} axis={slide_z.axis}.",
    )

    ctx.expect_gap(
        rail,
        base,
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
        name="rail_seats_on_base",
    )
    ctx.expect_overlap(
        rail,
        base,
        axes="xy",
        min_overlap=0.050,
        name="rail_sits_within_base_footprint",
    )
    ctx.expect_gap(
        mast,
        rail,
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
        name="mast_carriage_seats_on_rail",
    )
    ctx.expect_contact(
        top_bracket,
        mast,
        contact_tol=0.0006,
        name="top_bracket_bears_on_mast",
    )
    ctx.expect_overlap(
        top_bracket,
        mast,
        axes="xz",
        min_overlap=0.020,
        name="top_bracket_wraps_mast_in_profile",
    )

    base_dims = _aabb_size(ctx.part_world_aabb(base))
    mast_tube_dims = _aabb_size(ctx.part_element_world_aabb(mast, elem="mast_tube"))
    if base_dims is not None and mast_tube_dims is not None:
        ctx.check(
            "stage_proportions_read_correctly",
            base_dims[0] > 0.24
            and base_dims[1] > 0.14
            and base_dims[2] < 0.04
            and mast_tube_dims[2] > 4.5 * max(mast_tube_dims[0], mast_tube_dims[1]),
            f"Base dims={base_dims}, mast tube dims={mast_tube_dims}.",
        )

    x_lower = slide_x.motion_limits.lower
    x_upper = slide_x.motion_limits.upper
    if x_lower is not None and x_upper is not None:
        with ctx.pose({slide_x: x_lower}):
            mast_x_lo = ctx.part_world_position(mast)[0]
            ctx.expect_gap(
                mast,
                rail,
                axis="z",
                max_gap=0.0006,
                max_penetration=0.0,
                name="mast_supported_at_lower_x_limit",
            )
        with ctx.pose({slide_x: x_upper}):
            mast_x_hi = ctx.part_world_position(mast)[0]
            ctx.expect_gap(
                mast,
                rail,
                axis="z",
                max_gap=0.0006,
                max_penetration=0.0,
                name="mast_supported_at_upper_x_limit",
            )
        ctx.check(
            "mast_translates_laterally",
            mast_x_hi - mast_x_lo > 0.10,
            f"Observed X travel {mast_x_hi - mast_x_lo:.4f} m.",
        )

    z_lower = slide_z.motion_limits.lower
    z_upper = slide_z.motion_limits.upper
    if z_lower is not None and z_upper is not None:
        with ctx.pose({slide_z: z_lower}):
            bracket_z_lo = ctx.part_world_position(top_bracket)[2]
            ctx.expect_contact(
                top_bracket,
                mast,
                contact_tol=0.0006,
                name="top_bracket_guided_at_lower_z_limit",
            )
        with ctx.pose({slide_z: z_upper}):
            bracket_z_hi = ctx.part_world_position(top_bracket)[2]
            ctx.expect_contact(
                top_bracket,
                mast,
                contact_tol=0.0006,
                name="top_bracket_guided_at_upper_z_limit",
            )
        ctx.check(
            "top_bracket_translates_vertically",
            bracket_z_hi - bracket_z_lo > 0.09,
            f"Observed Z travel {bracket_z_hi - bracket_z_lo:.4f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
