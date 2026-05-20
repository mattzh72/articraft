from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_X = 0.34
BASE_Y = 0.48
BASE_PLATE_T = 0.018
BASE_LANDING_H = 0.017
BASE_TOP_Z = BASE_PLATE_T + BASE_LANDING_H

Y_RAIL_X = 0.10
Y_RAIL_W = 0.026
Y_RAIL_L = 0.39
Y_RAIL_H = 0.018
Y_RAIL_TOP_Z = BASE_TOP_Z + Y_RAIL_H

Y_STRIP_W = 0.012
Y_STRIP_L = 0.42
Y_STRIP_H = 0.006
Y_STRIP_X = 0.132

Y_TRAVEL_LOWER = -0.06
Y_TRAVEL_UPPER = 0.06

Y_STAGE_X = 0.27
Y_STAGE_Y = 0.17
Y_STAGE_BLOCK_X = 0.044
Y_STAGE_BLOCK_Y = 0.048
Y_STAGE_BLOCK_H = 0.022
Y_STAGE_PLATE_T = 0.018
Y_STAGE_TOP_Z = Y_STAGE_BLOCK_H + Y_STAGE_PLATE_T
Y_STAGE_MOUNT_PAD_T = 0.012
Y_STAGE_MOUNT_TOP_Z = Y_STAGE_BLOCK_H + Y_STAGE_PLATE_T + Y_STAGE_MOUNT_PAD_T

COLUMN_FRONT_FACE_Y = 0.042
COLUMN_DEPTH = 0.10
COLUMN_FRAME_X = 0.16
COLUMN_CHEEK_T = 0.022
COLUMN_WEB_T = 0.022
COLUMN_H = 0.32
COLUMN_BASE_Z = Y_STAGE_TOP_Z
COLUMN_BACK_FACE_Y = COLUMN_FRONT_FACE_Y - COLUMN_DEPTH

Z_RAIL_X = 0.055
Z_RAIL_W = 0.024
Z_RAIL_D = 0.018
Z_RAIL_L = 0.25
Z_RAIL_BOTTOM_Z = 0.07
Z_RAIL_FRONT_Y = COLUMN_FRONT_FACE_Y + Z_RAIL_D

Z_STRIP_W = 0.010
Z_STRIP_D = 0.010
Z_STRIP_L = Z_RAIL_L + 0.02
Z_STRIP_X = 0.084

Z_TRAVEL_LOWER = 0.0
Z_TRAVEL_UPPER = 0.10

Z_STAGE_HEIGHT = 0.16
Z_STAGE_PLATE_T = 0.014
Z_STAGE_BACKBONE_D = 0.022
Z_STAGE_BLOCK_X = 0.034
Z_STAGE_BLOCK_D = 0.022
Z_STAGE_BLOCK_H = 0.035


def _base_frame_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_X,
        BASE_Y,
        BASE_PLATE_T,
        centered=(True, True, False),
    )
    landings = (
        cq.Workplane("XY")
        .pushPoints([(-Y_RAIL_X, 0.0), (Y_RAIL_X, 0.0)])
        .box(
            Y_RAIL_W + 0.032,
            Y_RAIL_L + 0.036,
            BASE_LANDING_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_PLATE_T))
        .combine()
    )
    rear_riser = cq.Workplane("XY").box(
        0.18,
        0.09,
        0.012,
        centered=(True, True, False),
    ).translate((0.0, -0.12, BASE_PLATE_T))
    side_ribs = (
        cq.Workplane("XY")
        .pushPoints([(-0.145, 0.0), (0.145, 0.0)])
        .box(0.02, 0.42, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_PLATE_T))
        .combine()
    )
    front_bridge = cq.Workplane("XY").box(
        0.22,
        0.028,
        0.012,
        centered=(True, True, False),
    ).translate((0.0, 0.19, BASE_PLATE_T))
    body = plate.union(landings).union(rear_riser).union(side_ribs).union(front_bridge)

    central_pocket = cq.Workplane("XY").box(
        0.16,
        0.24,
        0.028,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_PLATE_T - 0.002))
    cable_window = cq.Workplane("XY").box(
        0.10,
        0.05,
        0.05,
        centered=(True, True, False),
    ).translate((0.0, -0.145, BASE_PLATE_T + 0.016))
    body = body.cut(central_pocket).cut(cable_window)

    foot_pads = (
        cq.Workplane("XY")
        .pushPoints([(-0.125, -0.18), (0.125, -0.18), (-0.125, 0.18), (0.125, 0.18)])
        .cylinder(0.006, 0.016, centered=(True, True, False))
        .combine()
    )
    return body.union(foot_pads)


def _y_stage_shape() -> cq.Workplane:
    saddle = cq.Workplane("XY").box(
        Y_STAGE_X,
        Y_STAGE_Y,
        Y_STAGE_PLATE_T,
        centered=(True, True, False),
    ).translate((0.0, 0.0, Y_STAGE_BLOCK_H))
    saddle_window = cq.Workplane("XY").box(
        0.15,
        0.07,
        0.020,
        centered=(True, True, False),
    ).translate((0.0, 0.0, Y_STAGE_BLOCK_H + 0.001))
    saddle = saddle.cut(saddle_window)

    bearing_blocks = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-Y_RAIL_X, -0.045),
                (-Y_RAIL_X, 0.045),
                (Y_RAIL_X, -0.045),
                (Y_RAIL_X, 0.045),
            ]
        )
        .box(
            Y_STAGE_BLOCK_X,
            Y_STAGE_BLOCK_Y,
            Y_STAGE_BLOCK_H,
            centered=(True, True, False),
        )
        .combine()
    )

    side_stiffeners = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.056), (0.0, 0.056)])
        .box(
            0.22,
            0.016,
            0.014,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, Y_STAGE_BLOCK_H))
        .combine()
    )
    center_mount = cq.Workplane("XY").box(
        0.12,
        0.08,
        Y_STAGE_MOUNT_PAD_T,
        centered=(True, True, False),
    ).translate((0.0, 0.0, Y_STAGE_TOP_Z))

    return (
        bearing_blocks.union(saddle)
        .union(side_stiffeners)
        .union(center_mount)
    )


def _column_frame_shape() -> cq.Workplane:
    front_web = cq.Workplane("XY").box(
        COLUMN_FRAME_X,
        COLUMN_WEB_T,
        COLUMN_H,
        centered=(True, True, False),
    ).translate((0.0, -COLUMN_WEB_T / 2.0, 0.0))
    side_cheeks = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-(COLUMN_FRAME_X / 2.0 - COLUMN_CHEEK_T / 2.0), 0.0),
                ((COLUMN_FRAME_X / 2.0 - COLUMN_CHEEK_T / 2.0), 0.0),
            ]
        )
        .box(
            COLUMN_CHEEK_T,
            COLUMN_DEPTH,
            COLUMN_H - 0.03,
            centered=(True, True, False),
        )
        .translate((0.0, -COLUMN_DEPTH / 2.0, 0.0))
        .combine()
    )
    lower_bridge = cq.Workplane("XY").box(
        COLUMN_FRAME_X - 0.02,
        0.055,
        0.018,
        centered=(True, True, False),
    ).translate((0.0, -0.060, 0.06))
    top_cap = cq.Workplane("XY").box(
        COLUMN_FRAME_X,
        0.065,
        0.018,
        centered=(True, True, False),
    ).translate((0.0, -0.050, COLUMN_H))
    base_pad = cq.Workplane("XY").box(
        0.12,
        0.08,
        0.014,
        centered=(True, True, False),
    )

    left_gusset = (
        cq.Workplane("XY")
        .box(0.012, 0.072, 0.108, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -28.0)
        .translate((-0.054, -0.020, 0.050))
    )
    right_gusset = (
        cq.Workplane("XY")
        .box(0.012, 0.072, 0.108, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -28.0)
        .translate((0.054, -0.020, 0.050))
    )

    return (
        base_pad.union(front_web)
        .union(side_cheeks)
        .union(lower_bridge)
        .union(top_cap)
        .union(left_gusset)
        .union(right_gusset)
    )


def _column_cover_shape() -> cq.Workplane:
    cover = cq.Workplane("XY").box(
        0.10,
        0.004,
        0.20,
        centered=(True, True, False),
    )
    flanges = (
        cq.Workplane("XY")
        .pushPoints([(-0.045, 0.0), (0.045, 0.0)])
        .box(0.010, 0.008, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.091))
        .combine()
    )
    return cover.union(flanges)


def _z_stage_shape() -> cq.Workplane:
    backbone = cq.Workplane("XY").box(
        0.09,
        Z_STAGE_BACKBONE_D,
        Z_STAGE_HEIGHT,
        centered=(True, True, False),
    )
    carriage_plate = cq.Workplane("XY").box(
        0.19,
        Z_STAGE_PLATE_T,
        Z_STAGE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, Z_STAGE_BACKBONE_D + Z_STAGE_PLATE_T / 2.0, 0.0))
    carriage_window = cq.Workplane("XY").box(
        0.09,
        0.02,
        0.08,
        centered=(True, True, False),
    ).translate((0.0, Z_STAGE_BACKBONE_D + Z_STAGE_PLATE_T / 2.0 - 0.002, 0.035))
    carriage_plate = carriage_plate.cut(carriage_window)

    tooling_plate = cq.Workplane("XY").box(
        0.22,
        0.012,
        0.12,
        centered=(True, True, False),
    ).translate((0.0, Z_STAGE_BACKBONE_D + Z_STAGE_PLATE_T + 0.006, 0.02))

    rail_blocks = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-Z_RAIL_X, 0.0),
                (Z_RAIL_X, 0.0),
            ]
        )
        .box(
            Z_STAGE_BLOCK_X,
            Z_STAGE_BLOCK_D,
            Z_STAGE_BLOCK_H,
            centered=(True, True, False),
        )
        .combine()
    )
    upper_blocks = rail_blocks.translate((0.0, 0.0, 0.100))

    lower_brace = cq.Workplane("XY").box(
        0.15,
        0.018,
        0.014,
        centered=(True, True, False),
    ).translate((0.0, Z_STAGE_BACKBONE_D + 0.014, 0.018))
    top_bridge = cq.Workplane("XY").box(
        0.13,
        0.018,
        0.014,
        centered=(True, True, False),
    ).translate((0.0, Z_STAGE_BACKBONE_D + 0.014, Z_STAGE_HEIGHT - 0.018))

    return (
        backbone.union(carriage_plate)
        .union(tooling_plate)
        .union(rail_blocks)
        .union(upper_blocks)
        .union(lower_brace)
        .union(top_bridge)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_yz_stage", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    plate_steel = model.material("plate_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    stop_red = model.material("stop_red", rgba=(0.62, 0.18, 0.16, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame.obj", assets=ASSETS),
        material=dark_steel,
        name="frame_shell",
    )

    y_left_rail = model.part("y_left_rail")
    y_left_rail.visual(Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)), material=rail_steel, name="rail_body")

    y_right_rail = model.part("y_right_rail")
    y_right_rail.visual(Box((Y_RAIL_W, Y_RAIL_L, Y_RAIL_H)), material=rail_steel, name="rail_body")

    y_left_strip = model.part("y_left_strip")
    y_left_strip.visual(
        Box((Y_STRIP_W, Y_STRIP_L, Y_STRIP_H)),
        material=plate_steel,
        name="strip_body",
    )

    y_right_strip = model.part("y_right_strip")
    y_right_strip.visual(
        Box((Y_STRIP_W, Y_STRIP_L, Y_STRIP_H)),
        material=plate_steel,
        name="strip_body",
    )

    y_front_stop = model.part("y_front_stop")
    y_front_stop.visual(Box((0.08, 0.014, 0.026)), material=stop_red, name="stop_body")
    y_front_stop.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021), rpy=(1.57079632679, 0.0, 0.0)),
        material=rail_steel,
        name="bumper",
    )

    y_rear_stop = model.part("y_rear_stop")
    y_rear_stop.visual(Box((0.08, 0.014, 0.026)), material=stop_red, name="stop_body")
    y_rear_stop.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021), rpy=(1.57079632679, 0.0, 0.0)),
        material=rail_steel,
        name="bumper",
    )

    y_stage = model.part("y_stage")
    y_stage.visual(
        mesh_from_cadquery(_y_stage_shape(), "y_stage.obj", assets=ASSETS),
        material=plate_steel,
        name="stage_shell",
    )
    z_left_rail = model.part("z_left_rail")
    z_left_rail.visual(Box((Z_RAIL_W, Z_RAIL_D, Z_RAIL_L)), material=rail_steel, name="rail_body")

    z_right_rail = model.part("z_right_rail")
    z_right_rail.visual(Box((Z_RAIL_W, Z_RAIL_D, Z_RAIL_L)), material=rail_steel, name="rail_body")

    z_left_strip = model.part("z_left_strip")
    z_left_strip.visual(
        Box((Z_STRIP_W, Z_STRIP_D, Z_STRIP_L)),
        material=plate_steel,
        name="strip_body",
    )

    z_right_strip = model.part("z_right_strip")
    z_right_strip.visual(
        Box((Z_STRIP_W, Z_STRIP_D, Z_STRIP_L)),
        material=plate_steel,
        name="strip_body",
    )

    column_cover = model.part("column_cover")
    column_cover.visual(
        mesh_from_cadquery(_column_cover_shape(), "column_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="cover_panel",
    )

    z_bottom_stop = model.part("z_bottom_stop")
    z_bottom_stop.visual(Box((0.13, 0.014, 0.018)), material=stop_red, name="stop_body")
    z_bottom_stop.visual(
        Box((0.07, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, 0.013)),
        material=rail_steel,
        name="bumper",
    )

    z_top_stop = model.part("z_top_stop")
    z_top_stop.visual(Box((0.13, 0.014, 0.018)), material=stop_red, name="stop_body")
    z_top_stop.visual(
        Box((0.07, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, 0.005)),
        material=rail_steel,
        name="bumper",
    )

    z_stage = model.part("z_stage")
    z_stage.visual(
        mesh_from_cadquery(_z_stage_shape(), "z_stage.obj", assets=ASSETS),
        material=plate_steel,
        name="carriage_shell",
    )
    model.articulation(
        "base_to_y_left_rail",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_left_rail,
        origin=Origin(xyz=(-Y_RAIL_X, 0.0, BASE_TOP_Z + Y_RAIL_H / 2.0)),
    )
    model.articulation(
        "base_to_y_right_rail",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_right_rail,
        origin=Origin(xyz=(Y_RAIL_X, 0.0, BASE_TOP_Z + Y_RAIL_H / 2.0)),
    )
    model.articulation(
        "base_to_y_left_strip",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_left_strip,
        origin=Origin(xyz=(-Y_STRIP_X, 0.0, BASE_PLATE_T + Y_STRIP_H / 2.0)),
    )
    model.articulation(
        "base_to_y_right_strip",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_right_strip,
        origin=Origin(xyz=(Y_STRIP_X, 0.0, BASE_PLATE_T + Y_STRIP_H / 2.0)),
    )
    model.articulation(
        "base_to_y_front_stop",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_front_stop,
        origin=Origin(xyz=(0.0, 0.1635, BASE_PLATE_T + 0.013)),
    )
    model.articulation(
        "base_to_y_rear_stop",
        ArticulationType.FIXED,
        parent=base_frame,
        child=y_rear_stop,
        origin=Origin(xyz=(0.0, -0.1635, BASE_PLATE_T + 0.013)),
    )
    model.articulation(
        "base_to_y_stage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.35,
            lower=Y_TRAVEL_LOWER,
            upper=Y_TRAVEL_UPPER,
        ),
    )
    model.articulation(
        "y_stage_to_z_left_rail",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_left_rail,
        origin=Origin(
            xyz=(-Z_RAIL_X, COLUMN_FRONT_FACE_Y + Z_RAIL_D / 2.0, Z_RAIL_BOTTOM_Z + Z_RAIL_L / 2.0)
        ),
    )
    model.articulation(
        "y_stage_to_z_right_rail",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_right_rail,
        origin=Origin(
            xyz=(Z_RAIL_X, COLUMN_FRONT_FACE_Y + Z_RAIL_D / 2.0, Z_RAIL_BOTTOM_Z + Z_RAIL_L / 2.0)
        ),
    )
    model.articulation(
        "y_stage_to_z_left_strip",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_left_strip,
        origin=Origin(
            xyz=(-Z_STRIP_X, COLUMN_FRONT_FACE_Y + Z_STRIP_D / 2.0, Z_RAIL_BOTTOM_Z + Z_RAIL_L / 2.0)
        ),
    )
    model.articulation(
        "y_stage_to_z_right_strip",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_right_strip,
        origin=Origin(
            xyz=(Z_STRIP_X, COLUMN_FRONT_FACE_Y + Z_STRIP_D / 2.0, Z_RAIL_BOTTOM_Z + Z_RAIL_L / 2.0)
        ),
    )
    model.articulation(
        "y_stage_to_column_cover",
        ArticulationType.FIXED,
        parent=y_stage,
        child=column_cover,
        origin=Origin(xyz=(0.0, COLUMN_BACK_FACE_Y - 0.002, 0.10)),
    )
    model.articulation(
        "y_stage_to_z_bottom_stop",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_bottom_stop,
        origin=Origin(xyz=(0.0, COLUMN_FRONT_FACE_Y + 0.007, 0.0685)),
    )
    model.articulation(
        "y_stage_to_z_top_stop",
        ArticulationType.FIXED,
        parent=y_stage,
        child=z_top_stop,
        origin=Origin(xyz=(0.0, COLUMN_FRONT_FACE_Y + 0.007, 0.3405)),
    )
    model.articulation(
        "y_stage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, Z_RAIL_FRONT_Y, Z_RAIL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.25,
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    y_left_rail = object_model.get_part("y_left_rail")
    y_right_rail = object_model.get_part("y_right_rail")
    y_left_strip = object_model.get_part("y_left_strip")
    y_right_strip = object_model.get_part("y_right_strip")
    y_front_stop = object_model.get_part("y_front_stop")
    y_rear_stop = object_model.get_part("y_rear_stop")
    y_stage = object_model.get_part("y_stage")
    z_left_rail = object_model.get_part("z_left_rail")
    z_right_rail = object_model.get_part("z_right_rail")
    z_left_strip = object_model.get_part("z_left_strip")
    z_right_strip = object_model.get_part("z_right_strip")
    column_cover = object_model.get_part("column_cover")
    z_bottom_stop = object_model.get_part("z_bottom_stop")
    z_top_stop = object_model.get_part("z_top_stop")
    z_stage = object_model.get_part("z_stage")

    y_axis = object_model.get_articulation("base_to_y_stage")
    z_axis = object_model.get_articulation("y_stage_to_z_stage")

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

    for part_name in (
        "base_frame",
        "y_left_rail",
        "y_right_rail",
        "y_left_strip",
        "y_right_strip",
        "y_front_stop",
        "y_rear_stop",
        "y_stage",
        "z_left_rail",
        "z_right_rail",
        "z_left_strip",
        "z_right_strip",
        "column_cover",
        "z_bottom_stop",
        "z_top_stop",
        "z_stage",
    ):
        ctx.check(
            f"{part_name}_present",
            object_model.get_part(part_name) is not None,
            details=f"Missing required part {part_name}",
        )

    ctx.expect_contact(y_left_rail, base_frame, name="y_left_rail_mounts_to_base")
    ctx.expect_contact(y_right_rail, base_frame, name="y_right_rail_mounts_to_base")
    ctx.expect_contact(y_left_strip, base_frame, name="y_left_strip_mounts_to_base")
    ctx.expect_contact(y_right_strip, base_frame, name="y_right_strip_mounts_to_base")
    ctx.expect_contact(y_front_stop, base_frame, name="y_front_stop_mounts_to_base")
    ctx.expect_contact(y_rear_stop, base_frame, name="y_rear_stop_mounts_to_base")

    ctx.expect_contact(y_stage, y_left_rail, name="y_stage_contacts_left_y_rail")
    ctx.expect_contact(y_stage, y_right_rail, name="y_stage_contacts_right_y_rail")
    ctx.expect_overlap(y_stage, y_left_rail, axes="y", min_overlap=0.09, name="y_stage_spans_left_rail")
    ctx.expect_overlap(y_stage, y_right_rail, axes="y", min_overlap=0.09, name="y_stage_spans_right_rail")

    ctx.expect_contact(z_left_rail, y_stage, name="left_z_rail_mounts_to_column")
    ctx.expect_contact(z_right_rail, y_stage, name="right_z_rail_mounts_to_column")
    ctx.expect_contact(z_left_strip, y_stage, name="left_z_strip_mounts_to_column")
    ctx.expect_contact(z_right_strip, y_stage, name="right_z_strip_mounts_to_column")
    ctx.expect_contact(column_cover, y_stage, name="column_cover_mounts_to_stage")
    ctx.expect_contact(z_bottom_stop, y_stage, name="z_bottom_stop_mounts_to_stage")
    ctx.expect_contact(z_top_stop, y_stage, name="z_top_stop_mounts_to_stage")

    ctx.expect_contact(z_stage, z_left_rail, name="z_stage_contacts_left_z_rail")
    ctx.expect_contact(z_stage, z_right_rail, name="z_stage_contacts_right_z_rail")
    ctx.expect_overlap(z_stage, z_left_rail, axes="z", min_overlap=0.10, name="z_stage_runs_on_left_z_rail")
    ctx.expect_overlap(z_stage, z_right_rail, axes="z", min_overlap=0.10, name="z_stage_runs_on_right_z_rail")
    ctx.expect_overlap(z_stage, y_stage, axes="x", min_overlap=0.12, name="z_stage_aligned_over_column")

    with ctx.pose({y_axis: Y_TRAVEL_UPPER}):
        ctx.expect_gap(
            y_front_stop,
            y_stage,
            axis="y",
            min_gap=0.0005,
            max_gap=0.010,
            name="front_y_stop_clearance_at_upper_limit",
        )
    with ctx.pose({y_axis: Y_TRAVEL_LOWER}):
        ctx.expect_gap(
            y_stage,
            y_rear_stop,
            axis="y",
            min_gap=0.0005,
            max_gap=0.010,
            name="rear_y_stop_clearance_at_lower_limit",
        )
    with ctx.pose({z_axis: Z_TRAVEL_LOWER}):
        ctx.expect_gap(
            z_stage,
            z_bottom_stop,
            axis="z",
            min_gap=0.0005,
            max_gap=0.010,
            name="bottom_z_stop_clearance_at_lower_limit",
        )
    with ctx.pose({z_axis: Z_TRAVEL_UPPER}):
        ctx.expect_gap(
            z_top_stop,
            z_stage,
            axis="z",
            min_gap=0.0005,
            max_gap=0.010,
            name="top_z_stop_clearance_at_upper_limit",
        )
        ctx.expect_contact(z_stage, z_left_rail, name="z_stage_left_contact_at_upper_limit")
        ctx.expect_contact(z_stage, z_right_rail, name="z_stage_right_contact_at_upper_limit")

    y_rest = ctx.part_world_position(y_stage)
    with ctx.pose({y_axis: Y_TRAVEL_UPPER}):
        y_shifted = ctx.part_world_position(y_stage)
    ctx.check(
        "y_axis_moves_only_along_y",
        y_rest is not None
        and y_shifted is not None
        and abs((y_shifted[1] - y_rest[1]) - Y_TRAVEL_UPPER) < 1e-6
        and abs(y_shifted[0] - y_rest[0]) < 1e-6
        and abs(y_shifted[2] - y_rest[2]) < 1e-6,
        details=f"Expected pure +Y translation of {Y_TRAVEL_UPPER:.3f} m for y_stage",
    )

    with ctx.pose({y_axis: 0.03, z_axis: 0.0}):
        z_low = ctx.part_world_position(z_stage)
    with ctx.pose({y_axis: 0.03, z_axis: Z_TRAVEL_UPPER}):
        z_high = ctx.part_world_position(z_stage)
    ctx.check(
        "z_axis_moves_only_along_z",
        z_low is not None
        and z_high is not None
        and abs((z_high[2] - z_low[2]) - Z_TRAVEL_UPPER) < 1e-6
        and abs(z_high[0] - z_low[0]) < 1e-6
        and abs(z_high[1] - z_low[1]) < 1e-6,
        details=f"Expected pure +Z translation of {Z_TRAVEL_UPPER:.3f} m for z_stage",
    )

    with ctx.pose({y_axis: Y_TRAVEL_LOWER, z_axis: 0.04}):
        z_stage_y_low = ctx.part_world_position(z_stage)
    with ctx.pose({y_axis: Y_TRAVEL_UPPER, z_axis: 0.04}):
        z_stage_y_high = ctx.part_world_position(z_stage)
    ctx.check(
        "z_stage_follows_y_stage_translation",
        z_stage_y_low is not None
        and z_stage_y_high is not None
        and abs((z_stage_y_high[1] - z_stage_y_low[1]) - (Y_TRAVEL_UPPER - Y_TRAVEL_LOWER)) < 1e-6
        and abs(z_stage_y_high[2] - z_stage_y_low[2]) < 1e-6,
        details="Vertical carriage should inherit the full horizontal Y-stage translation",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
