from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.90
BODY_DEPTH = 0.50
CANOPY_HEIGHT = 0.065
INTAKE_WIDTH = 0.82
INTAKE_DEPTH = 0.42
COVER_TOP_Z = 0.30
CHIMNEY_WIDTH = 0.30
CHIMNEY_DEPTH = 0.18
CHIMNEY_HEIGHT = 0.38
CONTROL_STRIP_WIDTH = 0.70
CONTROL_STRIP_DEPTH = 0.018
CONTROL_STRIP_HEIGHT = 0.048
CONTROL_STRIP_CENTER = (0.0, BODY_DEPTH - (CONTROL_STRIP_DEPTH * 0.5), 0.085)
KNOB_XS = (-0.22, 0.0, 0.22)
KNOB_Z = CONTROL_STRIP_CENTER[2]
KNOB_MOUNT_Y = BODY_DEPTH


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _rect_section(
    width: float,
    depth: float,
    z: float,
    center_y: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, depth, radius=radius, corner_segments=8)
    return [(x, y + center_y, z) for x, y in profile]


def _build_canopy_shell_mesh():
    outer = BoxGeometry((BODY_WIDTH, BODY_DEPTH, CANOPY_HEIGHT)).translate(
        0.0,
        BODY_DEPTH * 0.5,
        CANOPY_HEIGHT * 0.5,
    )
    inner = BoxGeometry((INTAKE_WIDTH, INTAKE_DEPTH, 0.058)).translate(
        0.0,
        BODY_DEPTH * 0.5,
        0.024,
    )
    return boolean_difference(outer, inner)


def _build_taper_cover_mesh():
    lower = _rect_section(0.80, 0.34, CANOPY_HEIGHT, 0.24, 0.030)
    middle = _rect_section(0.56, 0.27, 0.17, 0.17, 0.026)
    upper = _rect_section(CHIMNEY_WIDTH, CHIMNEY_DEPTH, COVER_TOP_Z, 0.09, 0.020)
    return section_loft([lower, middle, upper])


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_filter = model.material("dark_filter", rgba=(0.24, 0.25, 0.27, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.88, 0.90, 0.92, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        _save_mesh(_build_canopy_shell_mesh(), "range_hood_canopy_shell.obj"),
        material=brushed_steel,
        name="canopy_shell",
    )
    hood_body.visual(
        _save_mesh(_build_taper_cover_mesh(), "range_hood_taper_cover.obj"),
        material=brushed_steel,
        name="taper_cover",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_DEPTH * 0.5,
                COVER_TOP_Z + (CHIMNEY_HEIGHT * 0.5),
            )
        ),
        material=brushed_steel,
        name="chimney_stack",
    )
    hood_body.visual(
        Box((BODY_WIDTH + 0.02, 0.012, COVER_TOP_Z + CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.006,
                (COVER_TOP_Z + CHIMNEY_HEIGHT) * 0.5,
            )
        ),
        material=satin_steel,
        name="wall_backplate",
    )
    hood_body.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT)),
        origin=Origin(xyz=CONTROL_STRIP_CENTER),
        material=satin_steel,
        name="control_strip",
    )
    hood_body.visual(
        Box((0.41, INTAKE_DEPTH, 0.008)),
        origin=Origin(xyz=(-0.205, BODY_DEPTH * 0.5, 0.004)),
        material=dark_filter,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.41, INTAKE_DEPTH, 0.008)),
        origin=Origin(xyz=(0.205, BODY_DEPTH * 0.5, 0.004)),
        material=dark_filter,
        name="right_filter",
    )
    hood_body.visual(
        Box((0.014, INTAKE_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, 0.005)),
        material=satin_steel,
        name="filter_divider",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH + 0.02, BODY_DEPTH, COVER_TOP_Z + CHIMNEY_HEIGHT)),
        mass=19.0,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5,
                (COVER_TOP_Z + CHIMNEY_HEIGHT) * 0.5,
            )
        ),
    )

    knob_names = ("left_knob", "center_knob", "right_knob")
    for knob_name in knob_names:
        knob_part = model.part(knob_name)
        knob_part.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=charcoal,
            name="knob_skirt",
        )
        knob_part.visual(
            Cylinder(radius=0.017, length=0.018),
            origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=charcoal,
            name="knob_body",
        )
        knob_part.visual(
            Box((0.003, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.020, 0.011)),
            material=knob_marker,
            name="indicator",
        )
        knob_part.inertial = Inertial.from_geometry(
            Box((0.040, 0.024, 0.040)),
            mass=0.14,
            origin=Origin(xyz=(0.0, 0.011, 0.0)),
        )

    for knob_name, knob_x in zip(knob_names, KNOB_XS):
        model.articulation(
            f"hood_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob_name,
            origin=Origin(xyz=(knob_x, KNOB_MOUNT_Y, KNOB_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")

    left_joint = object_model.get_articulation("hood_to_left_knob")
    center_joint = object_model.get_articulation("hood_to_center_knob")
    right_joint = object_model.get_articulation("hood_to_right_knob")

    control_strip = hood_body.get_visual("control_strip")
    left_knob_body = left_knob.get_visual("knob_body")
    center_knob_body = center_knob.get_visual("knob_body")
    right_knob_body = right_knob.get_visual("knob_body")
    left_indicator = left_knob.get_visual("indicator")
    center_indicator = center_knob.get_visual("indicator")
    right_indicator = right_knob.get_visual("indicator")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    hood_aabb = ctx.part_world_aabb(hood_body)
    assert hood_aabb is not None
    hood_size = tuple(hood_aabb[1][axis] - hood_aabb[0][axis] for axis in range(3))
    ctx.check(
        "hood_body_proportions",
        0.88 <= hood_size[0] <= 0.94
        and 0.49 <= hood_size[1] <= 0.53
        and 0.66 <= hood_size[2] <= 0.70,
        (
            "Expected a wall hood sized roughly 0.9 m wide, 0.5 m deep, and 0.68 m tall; "
            f"got {hood_size!r}."
        ),
    )

    for knob_part, knob_body, label in (
        (left_knob, left_knob_body, "left"),
        (center_knob, center_knob_body, "center"),
        (right_knob, right_knob_body, "right"),
    ):
        ctx.expect_gap(
            knob_part,
            hood_body,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=knob_body,
            negative_elem=control_strip,
            name=f"{label}_knob_seated_on_control_strip",
        )
        ctx.expect_overlap(
            knob_part,
            hood_body,
            axes="xz",
            min_overlap=0.028,
            elem_a=knob_body,
            elem_b=control_strip,
            name=f"{label}_knob_overlaps_control_strip_face",
        )

    ctx.expect_origin_distance(
        left_knob,
        center_knob,
        axes="x",
        min_dist=0.20,
        max_dist=0.24,
        name="left_and_center_knob_spacing",
    )
    ctx.expect_origin_distance(
        center_knob,
        right_knob,
        axes="x",
        min_dist=0.20,
        max_dist=0.24,
        name="center_and_right_knob_spacing",
    )

    left_body_center = _aabb_center(ctx.part_element_world_aabb(left_knob, elem=left_knob_body))
    center_body_center = _aabb_center(
        ctx.part_element_world_aabb(center_knob, elem=center_knob_body)
    )
    right_body_center = _aabb_center(ctx.part_element_world_aabb(right_knob, elem=right_knob_body))

    left_indicator_rest = _aabb_center(ctx.part_element_world_aabb(left_knob, elem=left_indicator))
    center_indicator_rest = _aabb_center(
        ctx.part_element_world_aabb(center_knob, elem=center_indicator)
    )
    right_indicator_rest = _aabb_center(
        ctx.part_element_world_aabb(right_knob, elem=right_indicator)
    )

    ctx.check(
        "left_knob_indicator_starts_above_axis",
        left_indicator_rest[2] > left_body_center[2] + 0.006,
        "Left knob indicator should begin above its rotation axis.",
    )
    ctx.check(
        "center_knob_indicator_starts_above_axis",
        center_indicator_rest[2] > center_body_center[2] + 0.006,
        "Center knob indicator should begin above its rotation axis.",
    )
    ctx.check(
        "right_knob_indicator_starts_above_axis",
        right_indicator_rest[2] > right_body_center[2] + 0.006,
        "Right knob indicator should begin above its rotation axis.",
    )

    with ctx.pose({left_joint: math.pi * 0.5}):
        left_indicator_quarter = _aabb_center(
            ctx.part_element_world_aabb(left_knob, elem=left_indicator)
        )
        ctx.check(
            "left_knob_quarter_turn_moves_indicator_sideways",
            left_indicator_quarter[0] > left_body_center[0] + 0.007
            and abs(left_indicator_quarter[2] - left_body_center[2]) < 0.004
            and abs(left_indicator_quarter[1] - left_indicator_rest[1]) < 0.0015,
            "Quarter-turn should carry the left indicator around the knob about the Y axis.",
        )
        ctx.expect_gap(
            left_knob,
            hood_body,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=left_knob_body,
            negative_elem=control_strip,
            name="left_knob_remains_seated_when_turned",
        )

    with ctx.pose({center_joint: -math.pi * 0.5}):
        center_indicator_quarter = _aabb_center(
            ctx.part_element_world_aabb(center_knob, elem=center_indicator)
        )
        ctx.check(
            "center_knob_negative_quarter_turn_moves_indicator_sideways",
            center_indicator_quarter[0] < center_body_center[0] - 0.007
            and abs(center_indicator_quarter[2] - center_body_center[2]) < 0.004
            and abs(center_indicator_quarter[1] - center_indicator_rest[1]) < 0.0015,
            "Negative quarter-turn should move the center indicator to the opposite side.",
        )
        ctx.expect_gap(
            center_knob,
            hood_body,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=center_knob_body,
            negative_elem=control_strip,
            name="center_knob_remains_seated_when_turned",
        )

    with ctx.pose({right_joint: math.pi}):
        right_indicator_half = _aabb_center(
            ctx.part_element_world_aabb(right_knob, elem=right_indicator)
        )
        ctx.check(
            "right_knob_half_turn_inverts_indicator",
            right_indicator_half[2] < right_body_center[2] - 0.006
            and abs(right_indicator_half[1] - right_indicator_rest[1]) < 0.0015,
            "Half-turn should place the right knob indicator below its axis.",
        )
        ctx.expect_gap(
            right_knob,
            hood_body,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=right_knob_body,
            negative_elem=control_strip,
            name="right_knob_remains_seated_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
