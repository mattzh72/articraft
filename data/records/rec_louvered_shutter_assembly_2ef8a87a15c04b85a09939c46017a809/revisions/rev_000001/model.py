from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LEAF_WIDTH = 0.255
LEAF_HEIGHT = 1.40
LEAF_DEPTH = 0.028
FRAME_DEPTH = 0.060
OPENING_WIDTH = LEAF_WIDTH * 2.0
JAMB_WIDTH = 0.040
HEADER_HEIGHT = 0.040
SILL_HEIGHT = 0.040
TOP_BOTTOM_CLEARANCE = 0.005
FRAME_WIDTH = OPENING_WIDTH + 2.0 * JAMB_WIDTH
FRAME_HEIGHT = LEAF_HEIGHT + 2.0 * (HEADER_HEIGHT + TOP_BOTTOM_CLEARANCE)

STILE_CORE_WIDTH = 0.016
CLIP_LIP_WIDTH = 0.004
CLIP_DEPTH = 0.010
RAIL_HEIGHT = 0.075
CLEAR_WIDTH = LEAF_WIDTH - 2.0 * (STILE_CORE_WIDTH + CLIP_LIP_WIDTH)
SLAT_COUNT = 8
SLAT_DEPTH = 0.020
SLAT_THICKNESS = 0.008

ROD_WIDTH = 0.008
ROD_DEPTH = 0.004
ROD_X = LEAF_WIDTH - (STILE_CORE_WIDTH + CLIP_LIP_WIDTH) - 0.018
ROD_Y = 0.012
ROD_BOTTOM = RAIL_HEIGHT + 0.030
ROD_TOP = LEAF_HEIGHT - RAIL_HEIGHT - 0.030
ROD_LENGTH = ROD_TOP - ROD_BOTTOM
GUIDE_WIDTH = 0.004
GUIDE_DEPTH = 0.008
GUIDE_HEIGHT = 0.060
LOWER_GUIDE_Z = RAIL_HEIGHT + GUIDE_HEIGHT / 2.0
UPPER_GUIDE_Z = LEAF_HEIGHT - RAIL_HEIGHT - GUIDE_HEIGHT / 2.0

HINGE_PLATE_WIDTH = 0.018
HINGE_PLATE_THICKNESS = 0.004
HINGE_PLATE_HEIGHT = 0.110
HINGE_ZS = (0.180, LEAF_HEIGHT * 0.50, LEAF_HEIGHT - 0.180)

SLAT_ZS = tuple(0.175 + 0.150 * index for index in range(SLAT_COUNT))


def _build_louver_mesh():
    profile = rounded_rect_profile(
        SLAT_THICKNESS,
        SLAT_DEPTH,
        radius=SLAT_THICKNESS * 0.44,
        corner_segments=6,
    )
    geom = ExtrudeGeometry.from_z0(profile, CLEAR_WIDTH)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(-CLEAR_WIDTH / 2.0, 0.0, 0.0)
    return mesh_from_geometry(geom, "bifold_shutter_louver")


def _add_leaf_frame_visuals(part, *, paint, hardware, left_hinges: bool, right_hinges: bool) -> None:
    opening_height = LEAF_HEIGHT - 2.0 * RAIL_HEIGHT
    lip_center_z = RAIL_HEIGHT + opening_height / 2.0
    front_back_guide_y = LEAF_DEPTH / 2.0 - GUIDE_DEPTH / 2.0
    lip_offset_y = LEAF_DEPTH / 2.0 - CLIP_DEPTH / 2.0

    part.visual(
        Box((STILE_CORE_WIDTH, LEAF_DEPTH, LEAF_HEIGHT)),
        origin=Origin(xyz=(STILE_CORE_WIDTH / 2.0, 0.0, LEAF_HEIGHT / 2.0)),
        material=paint,
        name="left_stile",
    )
    part.visual(
        Box((STILE_CORE_WIDTH, LEAF_DEPTH, LEAF_HEIGHT)),
        origin=Origin(xyz=(LEAF_WIDTH - STILE_CORE_WIDTH / 2.0, 0.0, LEAF_HEIGHT / 2.0)),
        material=paint,
        name="right_stile",
    )
    part.visual(
        Box((LEAF_WIDTH, LEAF_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(LEAF_WIDTH / 2.0, 0.0, RAIL_HEIGHT / 2.0)),
        material=paint,
        name="bottom_rail",
    )
    part.visual(
        Box((LEAF_WIDTH, LEAF_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(LEAF_WIDTH / 2.0, 0.0, LEAF_HEIGHT - RAIL_HEIGHT / 2.0)),
        material=paint,
        name="top_rail",
    )

    for x_center, prefix in (
        (STILE_CORE_WIDTH + CLIP_LIP_WIDTH / 2.0, "left"),
        (LEAF_WIDTH - STILE_CORE_WIDTH - CLIP_LIP_WIDTH / 2.0, "right"),
    ):
        for y_sign, side_name in ((1.0, "front"), (-1.0, "rear")):
            part.visual(
                Box((CLIP_LIP_WIDTH, CLIP_DEPTH, opening_height)),
                origin=Origin(xyz=(x_center, y_sign * lip_offset_y, lip_center_z)),
                material=paint,
                name=f"{prefix}_{side_name}_clip",
            )

    for guide_z, guide_name in ((LOWER_GUIDE_Z, "lower"), (UPPER_GUIDE_Z, "upper")):
        for guide_x, guide_side in (
            (ROD_X - ROD_WIDTH / 2.0 - GUIDE_WIDTH / 2.0, "left"),
            (ROD_X + ROD_WIDTH / 2.0 + GUIDE_WIDTH / 2.0, "right"),
        ):
            part.visual(
                Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
                origin=Origin(xyz=(guide_x, front_back_guide_y, guide_z)),
                material=paint,
                name=f"{guide_name}_rod_guide_{guide_side}",
            )

    if left_hinges:
        for index, hinge_z in enumerate(HINGE_ZS):
            part.visual(
                Box((HINGE_PLATE_WIDTH, HINGE_PLATE_THICKNESS, HINGE_PLATE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        HINGE_PLATE_WIDTH / 2.0,
                        LEAF_DEPTH / 2.0 + HINGE_PLATE_THICKNESS / 2.0,
                        hinge_z,
                    )
                ),
                material=hardware,
                name=f"left_hinge_plate_{index}",
            )

    if right_hinges:
        for index, hinge_z in enumerate(HINGE_ZS):
            part.visual(
                Box((HINGE_PLATE_WIDTH, HINGE_PLATE_THICKNESS, HINGE_PLATE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        LEAF_WIDTH - HINGE_PLATE_WIDTH / 2.0,
                        LEAF_DEPTH / 2.0 + HINGE_PLATE_THICKNESS / 2.0,
                        hinge_z,
                    )
                ),
                material=hardware,
                name=f"right_hinge_plate_{index}",
            )


def _add_frame_visuals(part, *, paint, hardware) -> None:
    part.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-JAMB_WIDTH / 2.0, 0.0, FRAME_HEIGHT / 2.0)),
        material=paint,
        name="left_jamb",
    )
    part.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH + JAMB_WIDTH / 2.0, 0.0, FRAME_HEIGHT / 2.0)),
        material=paint,
        name="right_jamb",
    )
    part.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, HEADER_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH / 2.0,
                0.0,
                FRAME_HEIGHT - HEADER_HEIGHT / 2.0,
            )
        ),
        material=paint,
        name="header",
    )
    part.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0, 0.0, SILL_HEIGHT / 2.0)),
        material=paint,
        name="sill",
    )

    for index, hinge_z in enumerate(HINGE_ZS):
        part.visual(
            Box((HINGE_PLATE_WIDTH, HINGE_PLATE_THICKNESS, HINGE_PLATE_HEIGHT)),
            origin=Origin(
                xyz=(
                    -HINGE_PLATE_WIDTH / 2.0,
                    LEAF_DEPTH / 2.0 + HINGE_PLATE_THICKNESS / 2.0,
                    SILL_HEIGHT + TOP_BOTTOM_CLEARANCE + hinge_z,
                )
            ),
            material=hardware,
            name=f"jamb_hinge_plate_{index}",
        )


def _add_leaf_louvers_and_rod(model, *, leaf_name: str, louver_mesh, slat_material, rod_material) -> None:
    opening_center_x = STILE_CORE_WIDTH + CLIP_LIP_WIDTH + CLEAR_WIDTH / 2.0

    rod_part = model.part(f"{leaf_name}_control_rod")
    rod_part.visual(
        Box((ROD_WIDTH, ROD_DEPTH, ROD_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rod_material,
        name="control_rod",
    )
    model.articulation(
        f"{leaf_name}_to_{leaf_name}_control_rod",
        ArticulationType.PRISMATIC,
        parent=leaf_name,
        child=rod_part,
        origin=Origin(xyz=(ROD_X, ROD_Y, (ROD_BOTTOM + ROD_TOP) / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=-0.030,
            upper=0.030,
        ),
    )

    for index, slat_z in enumerate(SLAT_ZS):
        slat_part = model.part(f"{leaf_name}_louver_{index}")
        slat_part.visual(
            louver_mesh,
            material=slat_material,
            name="louver_slat",
        )
        model.articulation(
            f"{leaf_name}_to_{leaf_name}_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=leaf_name,
            child=slat_part,
            origin=Origin(xyz=(opening_center_x, 0.0, slat_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.5,
                lower=-0.80,
                upper=0.80,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bifold_louvered_shutter")

    frame_paint = model.material("frame_paint", rgba=(0.93, 0.92, 0.88, 1.0))
    slat_paint = model.material("slat_paint", rgba=(0.89, 0.87, 0.82, 1.0))
    rod_paint = model.material("rod_paint", rgba=(0.84, 0.82, 0.76, 1.0))
    hardware = model.material("hardware", rgba=(0.33, 0.30, 0.24, 1.0))

    louver_mesh = _build_louver_mesh()

    frame = model.part("frame")
    _add_frame_visuals(frame, paint=frame_paint, hardware=hardware)

    outer_leaf = model.part("outer_leaf")
    _add_leaf_frame_visuals(
        outer_leaf,
        paint=frame_paint,
        hardware=hardware,
        left_hinges=True,
        right_hinges=True,
    )

    inner_leaf = model.part("inner_leaf")
    _add_leaf_frame_visuals(
        inner_leaf,
        paint=frame_paint,
        hardware=hardware,
        left_hinges=True,
        right_hinges=False,
    )

    model.articulation(
        "frame_to_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=outer_leaf,
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT + TOP_BOTTOM_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.60,
        ),
    )

    model.articulation(
        "outer_leaf_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=outer_leaf,
        child=inner_leaf,
        origin=Origin(xyz=(LEAF_WIDTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-2.80,
            upper=0.0,
        ),
    )

    _add_leaf_louvers_and_rod(
        model,
        leaf_name="outer_leaf",
        louver_mesh=louver_mesh,
        slat_material=slat_paint,
        rod_material=rod_paint,
    )
    _add_leaf_louvers_and_rod(
        model,
        leaf_name="inner_leaf",
        louver_mesh=louver_mesh,
        slat_material=slat_paint,
        rod_material=rod_paint,
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
    ctx.fail_if_parts_overlap_in_current_pose()

    frame = object_model.get_part("frame")
    outer_leaf = object_model.get_part("outer_leaf")
    inner_leaf = object_model.get_part("inner_leaf")
    outer_rod = object_model.get_part("outer_leaf_control_rod")
    inner_rod = object_model.get_part("inner_leaf_control_rod")

    outer_leaf_hinge = object_model.get_articulation("frame_to_outer_leaf")
    inner_leaf_hinge = object_model.get_articulation("outer_leaf_to_inner_leaf")
    outer_rod_joint = object_model.get_articulation("outer_leaf_to_outer_leaf_control_rod")
    inner_rod_joint = object_model.get_articulation("inner_leaf_to_inner_leaf_control_rod")

    ctx.expect_contact(outer_leaf, frame, name="outer_leaf_seated_on_jamb")
    ctx.expect_contact(inner_leaf, outer_leaf, name="inner_leaf_seated_on_outer_leaf")
    ctx.expect_contact(inner_leaf, frame, name="inner_leaf_closes_to_right_jamb")
    ctx.expect_contact(outer_rod, outer_leaf, name="outer_control_rod_guided_by_leaf")
    ctx.expect_contact(inner_rod, inner_leaf, name="inner_control_rod_guided_by_leaf")
    ctx.expect_within(outer_rod, outer_leaf, axes="xz", name="outer_control_rod_inside_leaf_span")
    ctx.expect_within(inner_rod, inner_leaf, axes="xz", name="inner_control_rod_inside_leaf_span")

    ctx.check(
        "outer_leaf_hinge_axis_is_vertical",
        outer_leaf_hinge.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical z-axis hinge, got {outer_leaf_hinge.axis!r}",
    )
    ctx.check(
        "inner_leaf_hinge_axis_is_vertical",
        inner_leaf_hinge.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical z-axis hinge, got {inner_leaf_hinge.axis!r}",
    )
    ctx.check(
        "outer_control_rod_is_vertical_prismatic",
        outer_rod_joint.articulation_type == ArticulationType.PRISMATIC
        and outer_rod_joint.axis == (0.0, 0.0, 1.0),
        details=(
            "Outer control rod should translate vertically; "
            f"got type={outer_rod_joint.articulation_type!r}, axis={outer_rod_joint.axis!r}"
        ),
    )
    ctx.check(
        "inner_control_rod_is_vertical_prismatic",
        inner_rod_joint.articulation_type == ArticulationType.PRISMATIC
        and inner_rod_joint.axis == (0.0, 0.0, 1.0),
        details=(
            "Inner control rod should translate vertically; "
            f"got type={inner_rod_joint.articulation_type!r}, axis={inner_rod_joint.axis!r}"
        ),
    )

    for leaf_name, leaf_part, rod_part in (
        ("outer_leaf", outer_leaf, outer_rod),
        ("inner_leaf", inner_leaf, inner_rod),
    ):
        representative_louver = object_model.get_part(f"{leaf_name}_louver_3")
        ctx.expect_contact(
            rod_part,
            representative_louver,
            name=f"{leaf_name}_control_rod_touches_louvers",
        )
        for index in range(SLAT_COUNT):
            louver = object_model.get_part(f"{leaf_name}_louver_{index}")
            louver_joint = object_model.get_articulation(f"{leaf_name}_to_{leaf_name}_louver_{index}")
            ctx.expect_contact(
                louver,
                leaf_part,
                name=f"{leaf_name}_louver_{index}_stays_clipped_in_stiles",
            )
            ctx.expect_within(
                louver,
                leaf_part,
                axes="xz",
                name=f"{leaf_name}_louver_{index}_sits_inside_leaf_opening",
            )
            ctx.check(
                f"{leaf_name}_louver_{index}_axis_is_horizontal",
                louver_joint.axis == (1.0, 0.0, 0.0),
                details=f"Expected x-axis louver pivot, got {louver_joint.axis!r}",
            )

        louver_joint = object_model.get_articulation(f"{leaf_name}_to_{leaf_name}_louver_3")
        with ctx.pose({louver_joint: 0.45}):
            ctx.expect_contact(
                representative_louver,
                leaf_part,
                name=f"{leaf_name}_representative_louver_remains_seated_when_tilted",
            )

    outer_leaf_rest_aabb = ctx.part_world_aabb(outer_leaf)
    if outer_leaf_rest_aabb is not None:
        with ctx.pose({outer_leaf_hinge: 0.65}):
            outer_leaf_open_aabb = ctx.part_world_aabb(outer_leaf)
        ctx.check(
            "outer_leaf_swings_about_vertical_hinge",
            outer_leaf_open_aabb is not None
            and outer_leaf_open_aabb[1][1] > outer_leaf_rest_aabb[1][1] + 0.10,
            details=(
                "Expected the outer leaf to sweep in +y when opened; "
                f"rest={outer_leaf_rest_aabb!r}, posed={outer_leaf_open_aabb!r}"
            ),
        )
    else:
        ctx.fail("outer_leaf_aabb_available", "Could not evaluate outer leaf AABB in rest pose.")

    inner_leaf_rest_aabb = ctx.part_world_aabb(inner_leaf)
    if inner_leaf_rest_aabb is not None:
        with ctx.pose({inner_leaf_hinge: -1.20}):
            inner_leaf_folded_aabb = ctx.part_world_aabb(inner_leaf)
        ctx.check(
            "inner_leaf_folds_on_second_vertical_hinge",
            inner_leaf_folded_aabb is not None
            and inner_leaf_folded_aabb[0][1] < inner_leaf_rest_aabb[0][1] - 0.12,
            details=(
                "Expected the inner leaf to fold behind the outer leaf in -y; "
                f"rest={inner_leaf_rest_aabb!r}, posed={inner_leaf_folded_aabb!r}"
            ),
        )
    else:
        ctx.fail("inner_leaf_aabb_available", "Could not evaluate inner leaf AABB in rest pose.")

    outer_rod_rest = ctx.part_world_position(outer_rod)
    if outer_rod_rest is not None:
        with ctx.pose({outer_rod_joint: 0.020}):
            outer_rod_raised = ctx.part_world_position(outer_rod)
        ctx.check(
            "outer_control_rod_translates_upward",
            outer_rod_raised is not None and outer_rod_raised[2] > outer_rod_rest[2] + 0.015,
            details=f"Expected positive z travel; rest={outer_rod_rest!r}, posed={outer_rod_raised!r}",
        )
    else:
        ctx.fail("outer_control_rod_position_available", "Could not evaluate outer control rod position.")

    inner_rod_rest = ctx.part_world_position(inner_rod)
    if inner_rod_rest is not None:
        with ctx.pose({inner_rod_joint: -0.020}):
            inner_rod_lowered = ctx.part_world_position(inner_rod)
        ctx.check(
            "inner_control_rod_translates_downward",
            inner_rod_lowered is not None and inner_rod_lowered[2] < inner_rod_rest[2] - 0.015,
            details=f"Expected negative z travel; rest={inner_rod_rest!r}, posed={inner_rod_lowered!r}",
        )
    else:
        ctx.fail("inner_control_rod_position_available", "Could not evaluate inner control rod position.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
