from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_PLATE_X = 0.34
BASE_PLATE_Y = 0.34
BASE_PLATE_Z = 0.026

PEDESTAL_X = 0.14
PEDESTAL_Y = 0.14
PEDESTAL_Z = 0.050

OUTER_OD = 0.086
OUTER_WALL = 0.006
OUTER_LEN = 0.58
OUTER_Z0 = BASE_PLATE_Z + PEDESTAL_Z
OUTER_ID = OUTER_OD - 2.0 * OUTER_WALL

MID_OD = 0.068
MID_WALL = 0.005
MID_LEN = 0.62
MID_HOME_Z = 0.17
MID_TRAVEL = 0.28
MID_ID = MID_OD - 2.0 * MID_WALL

TOP_OD = 0.052
TOP_WALL = 0.004
TOP_LEN = 0.50
TOP_HOME_Z = 0.16
TOP_TRAVEL = 0.22

PAN_LIMIT = 2.8


def _add_box_visual(
    part,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_cylinder_visual(
    part,
    name: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_square_tube(
    part,
    *,
    prefix: str,
    outer: float,
    wall: float,
    length: float,
    z0: float,
    material: str,
    corner_overlap: float = 0.001,
) -> None:
    inner = outer - 2.0 * wall
    cx = (outer - wall) / 2.0
    cy = cx
    side_span = inner + 2.0 * corner_overlap
    zc = z0 + length / 2.0

    _add_box_visual(
        part,
        f"{prefix}_front",
        (outer, wall, length),
        (0.0, cy, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_back",
        (outer, wall, length),
        (0.0, -cy, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_right",
        (wall, side_span, length),
        (cx, 0.0, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_left",
        (wall, side_span, length),
        (-cx, 0.0, zc),
        material,
    )


def _add_square_ring(
    part,
    *,
    prefix: str,
    outer: float,
    inner: float,
    height: float,
    z0: float,
    material: str,
    corner_overlap: float = 0.001,
) -> None:
    wall = (outer - inner) / 2.0
    cx = inner / 2.0 + wall / 2.0
    cy = cx
    side_span = inner + 2.0 * corner_overlap
    zc = z0 + height / 2.0

    _add_box_visual(
        part,
        f"{prefix}_front",
        (outer, wall, height),
        (0.0, cy, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_back",
        (outer, wall, height),
        (0.0, -cy, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_right",
        (wall, side_span, height),
        (cx, 0.0, zc),
        material,
    )
    _add_box_visual(
        part,
        f"{prefix}_left",
        (wall, side_span, height),
        (-cx, 0.0, zc),
        material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_pan_mast")

    model.material("base_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("mast_outer", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("mast_mid", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("mast_top", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("head_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("plate_gray", rgba=(0.88, 0.89, 0.91, 1.0))

    base = model.part("base_mast")
    _add_box_visual(
        base,
        "base_plate",
        (BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z),
        (0.0, 0.0, BASE_PLATE_Z / 2.0),
        "base_graphite",
    )
    _add_box_visual(
        base,
        "pedestal",
        (PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z),
        (0.0, 0.0, BASE_PLATE_Z + PEDESTAL_Z / 2.0),
        "mast_outer",
    )
    _add_square_tube(
        base,
        prefix="outer_sleeve",
        outer=OUTER_OD,
        wall=OUTER_WALL,
        length=OUTER_LEN,
        z0=OUTER_Z0,
        material="mast_outer",
    )
    rib_x = 0.030
    rib_y = 0.104
    rib_z = 0.060
    rib_center_offset = OUTER_OD / 2.0 + rib_x / 2.0
    rib_center_z = BASE_PLATE_Z + rib_z / 2.0
    _add_box_visual(
        base,
        "rib_pos_x",
        (rib_x, rib_y, rib_z),
        (rib_center_offset, 0.0, rib_center_z),
        "mast_outer",
    )
    _add_box_visual(
        base,
        "rib_neg_x",
        (rib_x, rib_y, rib_z),
        (-rib_center_offset, 0.0, rib_center_z),
        "mast_outer",
    )
    _add_box_visual(
        base,
        "rib_pos_y",
        (rib_y, rib_x, rib_z),
        (0.0, rib_center_offset, rib_center_z),
        "mast_outer",
    )
    _add_box_visual(
        base,
        "rib_neg_y",
        (rib_y, rib_x, rib_z),
        (0.0, -rib_center_offset, rib_center_z),
        "mast_outer",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_X, BASE_PLATE_Y, OUTER_Z0 + OUTER_LEN)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, (OUTER_Z0 + OUTER_LEN) / 2.0)),
    )

    middle = model.part("middle_section")
    _add_square_tube(
        middle,
        prefix="middle_tube",
        outer=MID_OD,
        wall=MID_WALL,
        length=MID_LEN,
        z0=0.0,
        material="mast_mid",
    )
    _add_square_ring(
        middle,
        prefix="middle_guide_low",
        outer=OUTER_ID,
        inner=MID_OD - 0.002,
        height=0.020,
        z0=0.030,
        material="mast_mid",
    )
    _add_square_ring(
        middle,
        prefix="middle_guide_high",
        outer=OUTER_ID,
        inner=MID_OD - 0.002,
        height=0.020,
        z0=0.150,
        material="mast_mid",
    )
    _add_square_ring(
        middle,
        prefix="middle_trim",
        outer=0.080,
        inner=MID_OD - 0.002,
        height=0.035,
        z0=MID_LEN - 0.045,
        material="mast_mid",
    )
    middle.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, MID_LEN)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN / 2.0)),
    )

    top = model.part("top_section")
    _add_square_tube(
        top,
        prefix="top_tube",
        outer=TOP_OD,
        wall=TOP_WALL,
        length=TOP_LEN,
        z0=0.0,
        material="mast_top",
    )
    _add_square_ring(
        top,
        prefix="top_guide_low",
        outer=MID_ID,
        inner=TOP_OD - 0.002,
        height=0.018,
        z0=0.030,
        material="mast_top",
    )
    _add_square_ring(
        top,
        prefix="top_guide_high",
        outer=MID_ID,
        inner=TOP_OD - 0.002,
        height=0.018,
        z0=0.140,
        material="mast_top",
    )
    _add_square_ring(
        top,
        prefix="top_trim",
        outer=0.062,
        inner=TOP_OD - 0.002,
        height=0.025,
        z0=TOP_LEN - 0.040,
        material="mast_top",
    )
    _add_box_visual(
        top,
        "top_cap",
        (TOP_OD, TOP_OD, 0.014),
        (0.0, 0.0, TOP_LEN - 0.007),
        "mast_top",
    )
    top.inertial = Inertial.from_geometry(
        Box((0.062, 0.062, TOP_LEN)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, TOP_LEN / 2.0)),
    )

    head = model.part("pan_head")
    _add_cylinder_visual(
        head,
        "rotary_base",
        radius=0.036,
        length=0.020,
        center=(0.0, 0.0, 0.010),
        material="head_black",
    )
    _add_cylinder_visual(
        head,
        "rotary_cap",
        radius=0.028,
        length=0.020,
        center=(0.0, 0.0, 0.028),
        material="head_black",
    )
    _add_box_visual(
        head,
        "faceplate_neck",
        (0.040, 0.030, 0.034),
        (0.026, 0.0, 0.040),
        "head_black",
    )
    _add_box_visual(
        head,
        "faceplate_spine",
        (0.018, 0.056, 0.060),
        (0.050, 0.0, 0.056),
        "head_black",
    )
    _add_box_visual(
        head,
        "faceplate",
        (0.008, 0.094, 0.104),
        (0.061, 0.0, 0.064),
        "plate_gray",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.080, 0.100, 0.120)),
        mass=0.8,
        origin=Origin(xyz=(0.030, 0.0, 0.060)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, MID_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.35,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, TOP_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=TOP_TRAVEL,
        ),
    )
    model.articulation(
        "top_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=top,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TOP_LEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-PAN_LIMIT,
            upper=PAN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mast")
    middle = object_model.get_part("middle_section")
    top = object_model.get_part("top_section")
    head = object_model.get_part("pan_head")
    base_to_middle = object_model.get_articulation("base_to_middle")
    middle_to_top = object_model.get_articulation("middle_to_top")
    top_to_pan = object_model.get_articulation("top_to_pan_head")

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
        "base_to_middle axis is vertical",
        tuple(base_to_middle.axis) == (0.0, 0.0, 1.0),
        f"unexpected axis: {base_to_middle.axis}",
    )
    ctx.check(
        "middle_to_top axis is vertical",
        tuple(middle_to_top.axis) == (0.0, 0.0, 1.0),
        f"unexpected axis: {middle_to_top.axis}",
    )
    ctx.check(
        "pan axis is vertical",
        tuple(top_to_pan.axis) == (0.0, 0.0, 1.0),
        f"unexpected axis: {top_to_pan.axis}",
    )

    ctx.expect_contact(
        middle,
        base,
        contact_tol=1e-6,
        name="middle section is guided by the outer sleeve",
    )
    ctx.expect_contact(
        top,
        middle,
        contact_tol=1e-6,
        name="top section is guided by the middle sleeve",
    )
    ctx.expect_contact(
        head,
        top,
        contact_tol=1e-6,
        name="pan head is seated on the top section",
    )

    with ctx.pose({base_to_middle: MID_TRAVEL, middle_to_top: TOP_TRAVEL, top_to_pan: 1.4}):
        ctx.expect_contact(
            middle,
            base,
            contact_tol=1e-6,
            name="middle section stays guided at full extension",
        )
        ctx.expect_contact(
            top,
            middle,
            contact_tol=1e-6,
            name="top section stays guided at full extension",
        )
        ctx.expect_contact(
            head,
            top,
            contact_tol=1e-6,
            name="pan head stays mounted while rotated",
        )
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=1.32,
            name="mast reaches a credible extended height",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps when the mast is fully extended and panned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
