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
    section_loft,
)


BODY_WIDTH = 0.30
BODY_DEPTH = 0.46
SIDE_HEIGHT = 0.115
SIDE_THICKNESS = 0.010
ROOF_THICKNESS = 0.010
FLOOR_THICKNESS = 0.010
FRONT_FRAME_THICKNESS = 0.014
BACK_PANEL_THICKNESS = 0.010
FRONT_OUTER_WIDTH = 0.292
MID_OUTER_WIDTH = 0.300
REAR_OUTER_WIDTH = 0.298
OPENING_BOTTOM = 0.006
OPENING_WIDTH = 0.246
OPENING_SIDE_HEIGHT = 0.108
DOOR_THICKNESS = 0.010


def _arch_profile(
    width: float,
    side_height: float,
    *,
    bottom: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float]]:
    radius = width * 0.5
    pts: list[tuple[float, float]] = [
        (-radius, bottom),
        (radius, bottom),
        (radius, bottom + side_height),
    ]
    for step in range(1, segments):
        theta = math.pi * step / segments
        pts.append((radius * math.cos(theta), bottom + side_height + radius * math.sin(theta)))
    pts.append((-radius, bottom + side_height))
    return pts


def _arch_top_height(width: float, side_height: float, *, bottom: float = 0.0) -> float:
    return bottom + side_height + width * 0.5


def _extrude_xz_profile(profile: list[tuple[float, float]], depth: float, name: str):
    geom = ExtrudeGeometry(profile, depth, center=True).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _roof_band_section(y_pos: float, outer_width: float, *, segments: int = 28) -> list[tuple[float, float, float]]:
    outer_radius = outer_width * 0.5
    inner_radius = outer_radius - ROOF_THICKNESS
    if inner_radius <= 0.0:
        raise ValueError("Inner roof radius must remain positive.")

    outer: list[tuple[float, float, float]] = []
    inner: list[tuple[float, float, float]] = []

    for step in range(segments + 1):
        theta = math.pi - (math.pi * step / segments)
        outer.append(
            (
                outer_radius * math.cos(theta),
                y_pos,
                SIDE_HEIGHT + outer_radius * math.sin(theta),
            )
        )

    for step in range(segments + 1):
        theta = math.pi * step / segments
        inner.append(
            (
                inner_radius * math.cos(theta),
                y_pos,
                SIDE_HEIGHT + inner_radius * math.sin(theta),
            )
        )

    return outer + inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decorative_arch_mailbox")

    body_paint = model.material("body_paint", rgba=(0.20, 0.39, 0.26, 1.0))
    trim_paint = model.material("trim_paint", rgba=(0.86, 0.80, 0.66, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.18, 1.0))
    flag_red = model.material("flag_red", rgba=(0.76, 0.10, 0.09, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.67, 0.60, 0.46, 1.0))

    body = model.part("body")
    total_height = _arch_top_height(MID_OUTER_WIDTH, SIDE_HEIGHT)
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, total_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, total_height * 0.5)),
    )

    side_depth = BODY_DEPTH - FRONT_FRAME_THICKNESS - BACK_PANEL_THICKNESS
    floor_depth = side_depth
    side_y_center = FRONT_FRAME_THICKNESS + side_depth * 0.5

    body.visual(
        Box((BODY_WIDTH - (2.0 * SIDE_THICKNESS), floor_depth, FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FRAME_THICKNESS + floor_depth * 0.5,
                FLOOR_THICKNESS * 0.5,
            )
        ),
        material=dark_metal,
        name="floor_pan",
    )
    body.visual(
        Box((SIDE_THICKNESS, side_depth, SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + SIDE_THICKNESS * 0.5,
                side_y_center,
                SIDE_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="left_side",
    )
    body.visual(
        Box((SIDE_THICKNESS, side_depth, SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - SIDE_THICKNESS * 0.5,
                side_y_center,
                SIDE_HEIGHT * 0.5,
            )
        ),
        material=body_paint,
        name="right_side",
    )

    roof_geom = section_loft(
        [
            _roof_band_section(FRONT_FRAME_THICKNESS, FRONT_OUTER_WIDTH),
            _roof_band_section(BODY_DEPTH * 0.43, MID_OUTER_WIDTH),
            _roof_band_section(BODY_DEPTH - BACK_PANEL_THICKNESS, REAR_OUTER_WIDTH),
        ]
    )
    body.visual(
        mesh_from_geometry(roof_geom, "mailbox_roof"),
        material=body_paint,
        name="roof_shell",
    )

    front_frame_outer = _arch_profile(FRONT_OUTER_WIDTH, SIDE_HEIGHT)
    front_frame_inner = _arch_profile(
        OPENING_WIDTH,
        OPENING_SIDE_HEIGHT,
        bottom=OPENING_BOTTOM,
    )
    front_frame_geom = ExtrudeWithHolesGeometry(
        front_frame_outer,
        [front_frame_inner],
        FRONT_FRAME_THICKNESS,
        center=True,
    ).rotate_x(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(front_frame_geom, "mailbox_front_frame"),
        origin=Origin(xyz=(0.0, FRONT_FRAME_THICKNESS * 0.5, 0.0)),
        material=trim_paint,
        name="front_frame",
    )

    back_panel_mesh = _extrude_xz_profile(
        _arch_profile(REAR_OUTER_WIDTH, SIDE_HEIGHT),
        BACK_PANEL_THICKNESS,
        "mailbox_back_panel",
    )
    body.visual(
        back_panel_mesh,
        origin=Origin(xyz=(0.0, BODY_DEPTH - BACK_PANEL_THICKNESS * 0.5, 0.0)),
        material=body_paint,
        name="back_panel",
    )

    body.visual(
        Cylinder(radius=0.0045, length=OPENING_WIDTH - 0.030),
        origin=Origin(
            xyz=(0.0, FRONT_FRAME_THICKNESS * 0.72, OPENING_BOTTOM),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_metal,
        name="hinge_rod",
    )

    door = model.part("door")
    door_height = _arch_top_height(OPENING_WIDTH, OPENING_SIDE_HEIGHT)
    door.inertial = Inertial.from_geometry(
        Box((OPENING_WIDTH, DOOR_THICKNESS, door_height)),
        mass=0.9,
        origin=Origin(xyz=(0.0, DOOR_THICKNESS * 0.5, door_height * 0.5)),
    )

    door_mesh = _extrude_xz_profile(
        _arch_profile(OPENING_WIDTH, OPENING_SIDE_HEIGHT),
        DOOR_THICKNESS,
        "mailbox_door_outer",
    )
    door.visual(
        door_mesh,
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS * 0.5, 0.0)),
        material=trim_paint,
        name="door_outer",
    )

    door_inset_mesh = _extrude_xz_profile(
        _arch_profile(OPENING_WIDTH - 0.040, OPENING_SIDE_HEIGHT - 0.024, bottom=0.018),
        0.004,
        "mailbox_door_inset",
    )
    door.visual(
        door_inset_mesh,
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=body_paint,
        name="door_inset",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.006, 0.118),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=warm_metal,
        name="door_knob",
    )

    flag_bracket = model.part("flag_bracket")
    flag_bracket.inertial = Inertial.from_geometry(
        Box((0.018, 0.060, 0.090)),
        mass=0.12,
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
    )
    bracket_plate_thickness = 0.004
    flag_bracket.visual(
        Box((bracket_plate_thickness, 0.058, 0.078)),
        origin=Origin(xyz=(bracket_plate_thickness * 0.5, 0.0, 0.0)),
        material=dark_metal,
        name="mount_plate",
    )
    flag_bracket.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(
            xyz=(0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_metal,
        name="pivot_boss",
    )
    flag_bracket.visual(
        Box((0.012, 0.018, 0.050)),
        origin=Origin(xyz=(0.008, 0.0, -0.014)),
        material=dark_metal,
        name="lower_gusset",
    )

    flag = model.part("flag")
    flag.inertial = Inertial.from_geometry(
        Box((0.022, 0.125, 0.090)),
        mass=0.15,
        origin=Origin(xyz=(0.012, 0.060, 0.010)),
    )
    flag.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(
            xyz=(0.004, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_metal,
        name="flag_hub",
    )
    flag.visual(
        Box((0.004, 0.090, 0.012)),
        origin=Origin(xyz=(0.010, 0.050, 0.0)),
        material=flag_red,
        name="flag_arm",
    )
    flag.visual(
        Box((0.004, 0.070, 0.032)),
        origin=Origin(xyz=(0.010, 0.090, 0.010)),
        material=flag_red,
        name="flag_blade",
    )
    flag.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(
            xyz=(0.010, 0.123, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=flag_red,
        name="flag_tip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, OPENING_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_flag_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=flag_bracket,
        origin=Origin(xyz=(BODY_WIDTH * 0.5, 0.120, 0.145)),
    )
    model.articulation(
        "bracket_to_flag",
        ArticulationType.REVOLUTE,
        parent=flag_bracket,
        child=flag,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag_bracket = object_model.get_part("flag_bracket")
    flag = object_model.get_part("flag")
    door_hinge = object_model.get_articulation("body_to_door")
    flag_pivot = object_model.get_articulation("bracket_to_flag")

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
        "door hinge axis is horizontal",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected door hinge axis (1, 0, 0), got {door_hinge.axis!r}",
    )
    ctx.check(
        "door opens downward from closed pose",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.0,
        f"Unexpected door hinge limits: {door_hinge.motion_limits!r}",
    )
    ctx.check(
        "flag pivot axis is horizontal",
        tuple(flag_pivot.axis) == (1.0, 0.0, 0.0),
        f"Expected flag pivot axis (1, 0, 0), got {flag_pivot.axis!r}",
    )
    ctx.check(
        "flag raises upward from lowered pose",
        flag_pivot.motion_limits is not None
        and flag_pivot.motion_limits.lower == 0.0
        and flag_pivot.motion_limits.upper is not None
        and flag_pivot.motion_limits.upper > 1.0,
        f"Unexpected flag pivot limits: {flag_pivot.motion_limits!r}",
    )

    ctx.expect_contact(flag_bracket, body)
    ctx.expect_contact(flag, flag_bracket)
    ctx.expect_contact(door, body, contact_tol=1e-4)
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.20)

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_outer")
    closed_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_blade")
    assert closed_door_aabb is not None
    assert closed_flag_aabb is not None

    with ctx.pose({door_hinge: 1.20}):
        ctx.expect_contact(door, body, contact_tol=2e-4)
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_outer")
        assert open_door_aabb is not None
        assert open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.11
        assert open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.06

    with ctx.pose({flag_pivot: 1.30}):
        ctx.expect_contact(flag, flag_bracket)
        raised_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_blade")
        assert raised_flag_aabb is not None
        assert raised_flag_aabb[1][2] > closed_flag_aabb[1][2] + 0.06
        assert raised_flag_aabb[1][1] < closed_flag_aabb[1][1] - 0.02

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
