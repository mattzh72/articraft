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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


OUTER_FRAME_WIDTH = 0.924
OUTER_FRAME_HEIGHT = 1.344
OPENING_WIDTH = 0.828
OPENING_HEIGHT = 1.248
FRAME_DEPTH = 0.050
FRAME_BORDER = (OUTER_FRAME_WIDTH - OPENING_WIDTH) / 2.0

PANEL_WIDTH = 0.820
PANEL_HEIGHT = 1.240
PANEL_THICKNESS = 0.030
PANEL_STILE = 0.055
PANEL_RAIL = 0.075
CENTER_DIVIDER = 0.055
PANEL_GAP = 0.004

LOUVER_COUNT = 11
LOUVER_DEPTH = 0.065
LOUVER_THICKNESS = 0.010
LOUVER_MARGIN_Z = 0.050
LOUVER_PIN_RADIUS = 0.004
LOUVER_PIN_LENGTH = 0.009
LOUVER_PIN_OVERLAP = 0.001

HINGE_BARREL_RADIUS = 0.008
HINGE_FRAME_SEGMENT = 0.140
HINGE_PANEL_SEGMENT = 0.110


def _make_louver_mesh(length: float):
    half_depth = LOUVER_DEPTH / 2.0
    half_thickness = LOUVER_THICKNESS / 2.0
    profile = sample_catmull_rom_spline_2d(
        [
            (0.0, half_depth),
            (half_thickness * 0.95, half_depth * 0.55),
            (half_thickness, 0.0),
            (half_thickness * 0.90, -half_depth * 0.55),
            (0.0, -half_depth),
            (-half_thickness * 0.70, -half_depth * 0.58),
            (-half_thickness, 0.0),
            (-half_thickness * 0.70, half_depth * 0.58),
        ],
        samples_per_segment=8,
        closed=True,
    )
    blade = ExtrudeGeometry.centered(profile, length, cap=True).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(blade, "plantation_louver_blade")


def _spans(aabb):
    if aabb is None:
        return None
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _add_louver_bank(
    model: ArticulatedObject,
    panel,
    *,
    bank_name: str,
    bank_center_x: float,
    slat_z_positions: list[float],
    blade_length: float,
    blade_mesh,
    blade_material: Material,
    pin_material: Material,
):
    pin_center_offset = (
        blade_length / 2.0 + LOUVER_PIN_LENGTH / 2.0 - LOUVER_PIN_OVERLAP
    )
    for index, z_pos in enumerate(slat_z_positions, start=1):
        louver = model.part(f"{bank_name}_louver_{index:02d}")
        louver.visual(blade_mesh, material=blade_material, name="blade")
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(-pin_center_offset, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pin_material,
            name="left_pin",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(pin_center_offset, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pin_material,
            name="right_pin",
        )

        model.articulation(
            f"panel_to_{bank_name}_louver_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(bank_center_x, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=-1.10,
                upper=1.10,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_bank_plantation_shutter")

    frame_paint = model.material("frame_paint", color=(0.94, 0.94, 0.91))
    panel_paint = model.material("panel_paint", color=(0.97, 0.97, 0.94))
    hinge_metal = model.material("hinge_metal", color=(0.63, 0.58, 0.42))

    opening_frame = model.part("opening_frame")
    opening_frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, OUTER_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(-OUTER_FRAME_WIDTH / 2.0 + FRAME_BORDER / 2.0, 0.0, 0.0)
        ),
        material=frame_paint,
        name="left_jamb",
    )
    opening_frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, OUTER_FRAME_HEIGHT)),
        origin=Origin(
            xyz=(OUTER_FRAME_WIDTH / 2.0 - FRAME_BORDER / 2.0, 0.0, 0.0)
        ),
        material=frame_paint,
        name="right_jamb",
    )
    opening_frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(
            xyz=(0.0, 0.0, OUTER_FRAME_HEIGHT / 2.0 - FRAME_BORDER / 2.0)
        ),
        material=frame_paint,
        name="top_head",
    )
    opening_frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(
            xyz=(0.0, 0.0, -OUTER_FRAME_HEIGHT / 2.0 + FRAME_BORDER / 2.0)
        ),
        material=frame_paint,
        name="bottom_sill",
    )

    hinge_x = -OPENING_WIDTH / 2.0 + PANEL_GAP
    for idx, z_pos in enumerate((-0.420, 0.0, 0.420), start=1):
        opening_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_FRAME_SEGMENT),
            origin=Origin(xyz=(hinge_x - HINGE_BARREL_RADIUS, 0.0, z_pos)),
            material=hinge_metal,
            name=f"frame_hinge_knuckle_{idx}",
        )

    shutter_panel = model.part("shutter_panel")
    shutter_panel.visual(
        Box((PANEL_STILE, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(PANEL_STILE / 2.0, 0.0, 0.0)),
        material=panel_paint,
        name="left_stile",
    )
    shutter_panel.visual(
        Box((PANEL_STILE, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(PANEL_WIDTH - PANEL_STILE / 2.0, 0.0, 0.0)),
        material=panel_paint,
        name="right_stile",
    )
    shutter_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_RAIL)),
        origin=Origin(
            xyz=(PANEL_WIDTH / 2.0, 0.0, PANEL_HEIGHT / 2.0 - PANEL_RAIL / 2.0)
        ),
        material=panel_paint,
        name="top_rail",
    )
    shutter_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_RAIL)),
        origin=Origin(
            xyz=(PANEL_WIDTH / 2.0, 0.0, -PANEL_HEIGHT / 2.0 + PANEL_RAIL / 2.0)
        ),
        material=panel_paint,
        name="bottom_rail",
    )

    bank_width = (PANEL_WIDTH - 2.0 * PANEL_STILE - CENTER_DIVIDER) / 2.0
    shutter_panel.visual(
        Box((CENTER_DIVIDER, PANEL_THICKNESS, PANEL_HEIGHT - 2.0 * PANEL_RAIL)),
        origin=Origin(xyz=(PANEL_WIDTH / 2.0, 0.0, 0.0)),
        material=panel_paint,
        name="divider",
    )

    for idx, z_pos in enumerate((-0.210, 0.210), start=1):
        shutter_panel.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_PANEL_SEGMENT),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=hinge_metal,
            name=f"panel_hinge_knuckle_{idx}",
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=shutter_panel,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    left_bank_center_x = PANEL_STILE + bank_width / 2.0
    right_bank_center_x = PANEL_STILE + bank_width + CENTER_DIVIDER + bank_width / 2.0
    louver_opening_height = PANEL_HEIGHT - 2.0 * PANEL_RAIL
    louver_spacing = (louver_opening_height - 2.0 * LOUVER_MARGIN_Z) / (LOUVER_COUNT - 1)
    slat_z_positions = [
        -louver_opening_height / 2.0 + LOUVER_MARGIN_Z + idx * louver_spacing
        for idx in range(LOUVER_COUNT)
    ]

    blade_length = bank_width - 2.0 * 0.007
    blade_mesh = _make_louver_mesh(length=blade_length)
    _add_louver_bank(
        model,
        shutter_panel,
        bank_name="left",
        bank_center_x=left_bank_center_x,
        slat_z_positions=slat_z_positions,
        blade_length=blade_length,
        blade_mesh=blade_mesh,
        blade_material=panel_paint,
        pin_material=hinge_metal,
    )
    _add_louver_bank(
        model,
        shutter_panel,
        bank_name="right",
        bank_center_x=right_bank_center_x,
        slat_z_positions=slat_z_positions,
        blade_length=blade_length,
        blade_mesh=blade_mesh,
        blade_material=panel_paint,
        pin_material=hinge_metal,
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

    frame = object_model.get_part("opening_frame")
    panel = object_model.get_part("shutter_panel")
    hinge = object_model.get_articulation("frame_to_panel")
    left_mid = object_model.get_part("left_louver_06")
    right_mid = object_model.get_part("right_louver_06")
    left_mid_joint = object_model.get_articulation("panel_to_left_louver_06")
    right_mid_joint = object_model.get_articulation("panel_to_right_louver_06")

    def _has_part(name: str) -> bool:
        try:
            object_model.get_part(name)
            return True
        except Exception:
            return False

    def _has_joint(name: str) -> bool:
        try:
            object_model.get_articulation(name)
            return True
        except Exception:
            return False

    expected_part_names = ["opening_frame", "shutter_panel"] + [
        f"{bank}_louver_{index:02d}"
        for bank in ("left", "right")
        for index in range(1, LOUVER_COUNT + 1)
    ]
    expected_joint_names = ["frame_to_panel"] + [
        f"panel_to_{bank}_louver_{index:02d}"
        for bank in ("left", "right")
        for index in range(1, LOUVER_COUNT + 1)
    ]
    ctx.check(
        "all expected shutter parts exist",
        all(_has_part(name) for name in expected_part_names),
        details=f"missing={[name for name in expected_part_names if not _has_part(name)]}",
    )
    ctx.check(
        "all expected shutter articulations exist",
        all(_has_joint(name) for name in expected_joint_names),
        details=f"missing={[name for name in expected_joint_names if not _has_joint(name)]}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem="left_stile",
            negative_elem="left_jamb",
            min_gap=0.003,
            max_gap=0.006,
            name="left stile clears hinge jamb in closed pose",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem="right_jamb",
            negative_elem="right_stile",
            min_gap=0.003,
            max_gap=0.006,
            name="right stile clears latch jamb in closed pose",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="z",
            positive_elem="top_head",
            negative_elem="top_rail",
            min_gap=0.003,
            max_gap=0.006,
            name="top rail clears head in closed pose",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="bottom_sill",
            min_gap=0.003,
            max_gap=0.006,
            name="bottom rail clears sill in closed pose",
        )
        ctx.expect_contact(
            left_mid,
            panel,
            elem_a="left_pin",
            elem_b="left_stile",
            contact_tol=0.0005,
            name="left-bank louver mounts on the left stile",
        )
        ctx.expect_contact(
            left_mid,
            panel,
            elem_a="right_pin",
            elem_b="divider",
            contact_tol=0.0005,
            name="left-bank louver mounts on the divider",
        )
        ctx.expect_contact(
            right_mid,
            panel,
            elem_a="left_pin",
            elem_b="divider",
            contact_tol=0.0005,
            name="right-bank louver mounts on the divider",
        )
        ctx.expect_contact(
            right_mid,
            panel,
            elem_a="right_pin",
            elem_b="right_stile",
            contact_tol=0.0005,
            name="right-bank louver mounts on the right stile",
        )

    rest_right_stile = ctx.part_element_world_aabb(panel, elem="right_stile")
    with ctx.pose({hinge: 0.95}):
        open_right_stile = ctx.part_element_world_aabb(panel, elem="right_stile")
    ctx.check(
        "panel swings outward on the vertical hinge axis",
        rest_right_stile is not None
        and open_right_stile is not None
        and open_right_stile[1][1] > rest_right_stile[1][1] + 0.45,
        details=f"rest={rest_right_stile}, open={open_right_stile}",
    )

    with ctx.pose({hinge: 0.0, left_mid_joint: 0.0, right_mid_joint: 0.0}):
        left_rest_blade = ctx.part_element_world_aabb(left_mid, elem="blade")
        right_rest_blade = ctx.part_element_world_aabb(right_mid, elem="blade")
    with ctx.pose({hinge: 0.0, left_mid_joint: 0.95, right_mid_joint: -0.95}):
        left_tilt_blade = ctx.part_element_world_aabb(left_mid, elem="blade")
        right_tilt_blade = ctx.part_element_world_aabb(right_mid, elem="blade")

    left_rest_spans = _spans(left_rest_blade)
    left_tilt_spans = _spans(left_tilt_blade)
    right_rest_spans = _spans(right_rest_blade)
    right_tilt_spans = _spans(right_tilt_blade)

    ctx.check(
        "left-bank louver pitches about its long axis",
        left_rest_spans is not None
        and left_tilt_spans is not None
        and left_tilt_spans[2] > left_rest_spans[2] + 0.030
        and left_tilt_spans[1] < left_rest_spans[1] - 0.015,
        details=f"rest={left_rest_spans}, tilted={left_tilt_spans}",
    )
    ctx.check(
        "right-bank louver pitches about its long axis",
        right_rest_spans is not None
        and right_tilt_spans is not None
        and right_tilt_spans[2] > right_rest_spans[2] + 0.030
        and right_tilt_spans[1] < right_rest_spans[1] - 0.015,
        details=f"rest={right_rest_spans}, tilted={right_tilt_spans}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
