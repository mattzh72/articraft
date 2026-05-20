from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OPENING_WIDTH = 1.20
OPENING_HEIGHT = 1.38
FRAME_DEPTH = 0.045
JAMB_WIDTH = 0.060
HEAD_HEIGHT = 0.070
SILL_HEIGHT = 0.070

PANEL_WIDTH = 0.598
PANEL_HEIGHT = 1.335
PANEL_DEPTH = 0.030
STILE_WIDTH = 0.055
TOP_RAIL_HEIGHT = 0.090
BOTTOM_RAIL_HEIGHT = 0.110

LOUVER_COUNT = 10
LOUVER_DEPTH = 0.044
LOUVER_THICKNESS = 0.009
LOUVER_PIN_LENGTH = 0.004
LOUVER_PIN_RADIUS = 0.0035
HINGE_LEAF_WIDTH = 0.028
HINGE_LEAF_THICKNESS = 0.003
HINGE_LEAF_HEIGHT = 0.085
HINGE_BARREL_RADIUS = 0.006
HINGE_POSITIONS = (-0.47, 0.0, 0.47)


def _louver_profile(depth: float, thickness: float) -> list[tuple[float, float]]:
    half_depth = depth * 0.5
    half_thickness = thickness * 0.5
    return [
        (0.0, -half_depth),
        (half_thickness * 0.55, -depth * 0.34),
        (half_thickness * 0.90, -depth * 0.12),
        (half_thickness, 0.0),
        (half_thickness * 0.90, depth * 0.12),
        (half_thickness * 0.55, depth * 0.34),
        (0.0, half_depth),
        (-half_thickness * 0.55, depth * 0.34),
        (-half_thickness * 0.90, depth * 0.12),
        (-half_thickness, 0.0),
        (-half_thickness * 0.90, -depth * 0.12),
        (-half_thickness * 0.55, -depth * 0.34),
    ]


def _make_louver_mesh(length: float):
    geometry = ExtrudeGeometry(
        _louver_profile(LOUVER_DEPTH, LOUVER_THICKNESS),
        length,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_y(pi / 2.0)
    return mesh_from_geometry(geometry, "shutter_louver_blade")


def _add_panel(
    model: ArticulatedObject,
    *,
    side: str,
    frame_material,
    louver_material,
    hinge_material,
    louver_mesh,
) -> None:
    panel = model.part(f"{side}_panel")

    clear_span = PANEL_WIDTH - (2.0 * STILE_WIDTH)
    top_rail_center_z = (PANEL_HEIGHT * 0.5) - (TOP_RAIL_HEIGHT * 0.5)
    bottom_rail_center_z = -(PANEL_HEIGHT * 0.5) + (BOTTOM_RAIL_HEIGHT * 0.5)

    panel.visual(
        Box((STILE_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(STILE_WIDTH * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="outer_stile",
    )
    panel.visual(
        Box((STILE_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(PANEL_WIDTH - (STILE_WIDTH * 0.5), 0.0, 0.0)),
        material=frame_material,
        name="inner_stile",
    )
    panel.visual(
        Box((clear_span, PANEL_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, 0.0, top_rail_center_z)),
        material=frame_material,
        name="top_rail",
    )
    panel.visual(
        Box((clear_span, PANEL_DEPTH, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, 0.0, bottom_rail_center_z)),
        material=frame_material,
        name="bottom_rail",
    )
    for hinge_index, hinge_z in enumerate(HINGE_POSITIONS):
        panel.visual(
            Box((HINGE_LEAF_WIDTH, HINGE_LEAF_THICKNESS, HINGE_LEAF_HEIGHT)),
            origin=Origin(
                xyz=(
                    HINGE_LEAF_WIDTH * 0.5,
                    (PANEL_DEPTH * 0.5) + (HINGE_LEAF_THICKNESS * 0.5),
                    hinge_z,
                )
            ),
            material=hinge_material,
            name=f"hinge_leaf_{hinge_index}",
        )

    panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        mass=8.5,
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, 0.0, 0.0)),
    )

    louver_length = clear_span - (2.0 * LOUVER_PIN_LENGTH)
    louver_zone_bottom = -(PANEL_HEIGHT * 0.5) + BOTTOM_RAIL_HEIGHT
    louver_pitch = (PANEL_HEIGHT - TOP_RAIL_HEIGHT - BOTTOM_RAIL_HEIGHT) / LOUVER_COUNT
    louver_center_x = PANEL_WIDTH * 0.5

    for index in range(LOUVER_COUNT):
        z_center = louver_zone_bottom + (index + 0.5) * louver_pitch
        louver = model.part(f"{side}_louver_{index}")
        louver.visual(
            louver_mesh,
            material=louver_material,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(-(louver_length * 0.5) - (LOUVER_PIN_LENGTH * 0.5), 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=louver_material,
            name="outer_pivot",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=((louver_length * 0.5) + (LOUVER_PIN_LENGTH * 0.5), 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=louver_material,
            name="inner_pivot",
        )
        louver.inertial = Inertial.from_geometry(
            Box((louver_length + (2.0 * LOUVER_PIN_LENGTH), LOUVER_DEPTH, LOUVER_THICKNESS)),
            mass=0.18,
            origin=Origin(),
        )

        model.articulation(
            f"{side}_panel_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(louver_center_x, 0.0, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=1.2,
                lower=-0.55,
                upper=0.55,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_louvered_shutter")

    painted_frame = model.material("painted_frame", rgba=(0.94, 0.93, 0.90, 1.0))
    painted_louver = model.material("painted_louver", rgba=(0.90, 0.89, 0.86, 1.0))
    trim_shadow = model.material("trim_shadow", rgba=(0.78, 0.76, 0.72, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.54, 0.56, 0.58, 1.0))

    opening_frame = model.part("opening_frame")
    outer_width = OPENING_WIDTH + (2.0 * JAMB_WIDTH)
    outer_height = OPENING_HEIGHT + HEAD_HEIGHT + SILL_HEIGHT

    opening_frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, outer_height)),
        origin=Origin(xyz=(-(OPENING_WIDTH * 0.5) - (JAMB_WIDTH * 0.5), 0.0, 0.0)),
        material=painted_frame,
        name="left_jamb",
    )
    opening_frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, outer_height)),
        origin=Origin(xyz=((OPENING_WIDTH * 0.5) + (JAMB_WIDTH * 0.5), 0.0, 0.0)),
        material=painted_frame,
        name="right_jamb",
    )
    opening_frame.visual(
        Box((outer_width, FRAME_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, (OPENING_HEIGHT * 0.5) + (HEAD_HEIGHT * 0.5))),
        material=painted_frame,
        name="head",
    )
    opening_frame.visual(
        Box((outer_width, FRAME_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -((OPENING_HEIGHT * 0.5) + (SILL_HEIGHT * 0.5)))),
        material=painted_frame,
        name="sill",
    )
    opening_frame.visual(
        Box((outer_width + 0.050, 0.012, HEAD_HEIGHT + 0.040)),
        origin=Origin(xyz=(0.0, (FRAME_DEPTH * 0.5) + 0.006, (OPENING_HEIGHT * 0.5) + (HEAD_HEIGHT * 0.5))),
        material=trim_shadow,
        name="head_casing",
    )
    opening_frame.visual(
        Box((JAMB_WIDTH + 0.040, 0.012, outer_height + 0.040)),
        origin=Origin(
            xyz=(-(OPENING_WIDTH * 0.5) - (JAMB_WIDTH * 0.5), (FRAME_DEPTH * 0.5) + 0.006, 0.0)
        ),
        material=trim_shadow,
        name="left_casing",
    )
    opening_frame.visual(
        Box((JAMB_WIDTH + 0.040, 0.012, outer_height + 0.040)),
        origin=Origin(
            xyz=((OPENING_WIDTH * 0.5) + (JAMB_WIDTH * 0.5), (FRAME_DEPTH * 0.5) + 0.006, 0.0)
        ),
        material=trim_shadow,
        name="right_casing",
    )
    opening_frame.visual(
        Box((outer_width + 0.060, 0.016, SILL_HEIGHT + 0.050)),
        origin=Origin(xyz=(0.0, (FRAME_DEPTH * 0.5) + 0.008, -((OPENING_HEIGHT * 0.5) + (SILL_HEIGHT * 0.5)))),
        material=trim_shadow,
        name="sill_casing",
    )
    for hinge_index, hinge_z in enumerate(HINGE_POSITIONS):
        opening_frame.visual(
            Box((HINGE_LEAF_WIDTH, HINGE_LEAF_THICKNESS, HINGE_LEAF_HEIGHT)),
            origin=Origin(
                xyz=(
                    -(OPENING_WIDTH * 0.5) - (HINGE_LEAF_WIDTH * 0.5),
                    (FRAME_DEPTH * 0.5) + (HINGE_LEAF_THICKNESS * 0.5),
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"left_hinge_leaf_{hinge_index}",
        )
        opening_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_LEAF_HEIGHT),
            origin=Origin(
                xyz=(
                    -(OPENING_WIDTH * 0.5) - HINGE_BARREL_RADIUS,
                    (FRAME_DEPTH * 0.5) - 0.0035,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"left_hinge_barrel_{hinge_index}",
        )
        opening_frame.visual(
            Box((HINGE_LEAF_WIDTH, HINGE_LEAF_THICKNESS, HINGE_LEAF_HEIGHT)),
            origin=Origin(
                xyz=(
                    (OPENING_WIDTH * 0.5) + (HINGE_LEAF_WIDTH * 0.5),
                    (FRAME_DEPTH * 0.5) + (HINGE_LEAF_THICKNESS * 0.5),
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"right_hinge_leaf_{hinge_index}",
        )
        opening_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_LEAF_HEIGHT),
            origin=Origin(
                xyz=(
                    (OPENING_WIDTH * 0.5) + HINGE_BARREL_RADIUS,
                    (FRAME_DEPTH * 0.5) - 0.0035,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"right_hinge_barrel_{hinge_index}",
        )
    opening_frame.inertial = Inertial.from_geometry(
        Box((outer_width, FRAME_DEPTH, outer_height)),
        mass=18.0,
        origin=Origin(),
    )

    louver_mesh = _make_louver_mesh(PANEL_WIDTH - (2.0 * STILE_WIDTH) - (2.0 * LOUVER_PIN_LENGTH))
    _add_panel(
        model,
        side="left",
        frame_material=painted_frame,
        louver_material=painted_louver,
        hinge_material=hinge_metal,
        louver_mesh=louver_mesh,
    )
    _add_panel(
        model,
        side="right",
        frame_material=painted_frame,
        louver_material=painted_louver,
        hinge_material=hinge_metal,
        louver_mesh=louver_mesh,
    )

    model.articulation(
        "frame_to_left_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child="left_panel",
        origin=Origin(xyz=(-(OPENING_WIDTH * 0.5), 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.1,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "frame_to_right_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child="right_panel",
        origin=Origin(xyz=((OPENING_WIDTH * 0.5), 0.0, 0.0), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.1,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    opening_frame = object_model.get_part("opening_frame")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    left_hinge = object_model.get_articulation("frame_to_left_panel")
    right_hinge = object_model.get_articulation("frame_to_right_panel")
    left_mid_louver = object_model.get_part(f"left_louver_{LOUVER_COUNT // 2}")
    right_mid_louver = object_model.get_part(f"right_louver_{LOUVER_COUNT // 2}")
    left_mid_louver_joint = object_model.get_articulation(f"left_panel_to_louver_{LOUVER_COUNT // 2}")
    right_mid_louver_joint = object_model.get_articulation(f"right_panel_to_louver_{LOUVER_COUNT // 2}")

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
        "panel_hinges_are_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0) and right_hinge.axis == (0.0, 0.0, 1.0),
        details=f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "representative_louver_axes_are_longitudinal",
        left_mid_louver_joint.axis == (1.0, 0.0, 0.0) and right_mid_louver_joint.axis == (1.0, 0.0, 0.0),
        details=f"left={left_mid_louver_joint.axis}, right={right_mid_louver_joint.axis}",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(left_panel, opening_frame, name="left_panel_seats_on_left_jamb")
        ctx.expect_contact(right_panel, opening_frame, name="right_panel_seats_on_right_jamb")
        ctx.expect_gap(
            right_panel,
            left_panel,
            axis="x",
            min_gap=0.002,
            max_gap=0.006,
            name="panel_meeting_gap_is_narrow",
        )
        ctx.expect_contact(left_mid_louver, left_panel, name="left_mid_louver_mounts_into_panel")
        ctx.expect_contact(right_mid_louver, right_panel, name="right_mid_louver_mounts_into_panel")
        ctx.expect_within(
            left_mid_louver,
            left_panel,
            axes="xz",
            margin=0.0,
            name="left_mid_louver_stays_within_panel_opening",
        )
        ctx.expect_within(
            right_mid_louver,
            right_panel,
            axes="xz",
            margin=0.0,
            name="right_mid_louver_stays_within_panel_opening",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
