from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.39
BODY_WIDTH = 0.44
BODY_HEIGHT = 0.18

LID_DEPTH = 0.30
LID_WIDTH = 0.34

CASSETTE_TRAVEL = 0.12
PANEL_PITCH = math.atan2(0.058 - 0.030, 0.082)


def _panel_shape():
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, 0.000),
                (0.082, 0.012),
                (0.082, 0.058),
                (0.000, 0.030),
            ]
        )
        .close()
        .extrude(0.065, both=True)
    )


def _panel_surface_z(x_pos: float) -> float:
    return 0.030 + (0.058 - 0.030) * (x_pos / 0.082)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_all_in_one_printer")

    body_mat = model.material("body_mat", rgba=(0.86, 0.87, 0.89, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.17, 0.18, 0.20, 1.0))
    glass_mat = model.material("glass_mat", rgba=(0.40, 0.55, 0.62, 0.35))
    panel_mat = model.material("panel_mat", rgba=(0.20, 0.21, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=body_mat,
        name="base_floor",
    )
    body.visual(
        Box((BODY_DEPTH, 0.030, 0.160)),
        origin=Origin(xyz=(0.000, -0.205, 0.100)),
        material=body_mat,
        name="left_side",
    )
    body.visual(
        Box((BODY_DEPTH, 0.030, 0.160)),
        origin=Origin(xyz=(0.000, 0.205, 0.100)),
        material=body_mat,
        name="right_side",
    )
    body.visual(
        Box((0.030, 0.380, 0.160)),
        origin=Origin(xyz=(-0.180, 0.000, 0.100)),
        material=body_mat,
        name="rear_wall",
    )
    body.visual(
        Box((0.030, 0.053, 0.160)),
        origin=Origin(xyz=(0.180, -0.1935, 0.100)),
        material=body_mat,
        name="front_cheek_0",
    )
    body.visual(
        Box((0.030, 0.053, 0.160)),
        origin=Origin(xyz=(0.180, 0.1935, 0.100)),
        material=body_mat,
        name="front_cheek_1",
    )
    body.visual(
        Box((0.030, 0.334, 0.060)),
        origin=Origin(xyz=(0.180, 0.000, 0.150)),
        material=body_mat,
        name="front_lintel",
    )
    body.visual(
        Box((0.090, BODY_WIDTH, 0.020)),
        origin=Origin(xyz=(-0.150, 0.000, 0.170)),
        material=body_mat,
        name="top_rear_strip",
    )
    body.visual(
        Box((0.080, BODY_WIDTH, 0.020)),
        origin=Origin(xyz=(0.155, 0.000, 0.170)),
        material=body_mat,
        name="top_front_strip",
    )
    body.visual(
        Box((0.285, 0.075, 0.020)),
        origin=Origin(xyz=(0.005, -0.1825, 0.170)),
        material=body_mat,
        name="top_side_strip_0",
    )
    body.visual(
        Box((0.285, 0.075, 0.020)),
        origin=Origin(xyz=(0.005, 0.1825, 0.170)),
        material=body_mat,
        name="top_side_strip_1",
    )
    body.visual(
        Box((0.282, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, -0.140, 0.159)),
        material=body_mat,
        name="glass_ledge_0",
    )
    body.visual(
        Box((0.282, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, 0.140, 0.159)),
        material=body_mat,
        name="glass_ledge_1",
    )
    body.visual(
        Box((0.010, 0.290, 0.002)),
        origin=Origin(xyz=(-0.141, 0.000, 0.159)),
        material=body_mat,
        name="glass_ledge_2",
    )
    body.visual(
        Box((0.010, 0.290, 0.002)),
        origin=Origin(xyz=(0.141, 0.000, 0.159)),
        material=body_mat,
        name="glass_ledge_3",
    )
    body.visual(
        Box((0.040, 0.120, 0.036)),
        origin=Origin(xyz=(0.175, 0.140, 0.198)),
        material=body_mat,
        name="panel_bracket",
    )

    scanner_glass = model.part("scanner_glass")
    scanner_glass.visual(
        Box((0.272, 0.290, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.001)),
        material=glass_mat,
        name="glass",
    )
    model.articulation(
        "body_to_scanner_glass",
        ArticulationType.FIXED,
        parent=body,
        child=scanner_glass,
        origin=Origin(xyz=(0.000, 0.000, 0.160)),
    )

    flatbed_lid = model.part("flatbed_lid")
    flatbed_lid.visual(
        Box((LID_DEPTH, LID_WIDTH, 0.004)),
        origin=Origin(xyz=(0.150, 0.000, 0.022)),
        material=body_mat,
        name="lid_top",
    )
    flatbed_lid.visual(
        Box((LID_DEPTH, 0.014, 0.020)),
        origin=Origin(xyz=(0.150, -0.163, 0.010)),
        material=body_mat,
        name="lid_side_0",
    )
    flatbed_lid.visual(
        Box((LID_DEPTH, 0.014, 0.020)),
        origin=Origin(xyz=(0.150, 0.163, 0.010)),
        material=body_mat,
        name="lid_side_1",
    )
    flatbed_lid.visual(
        Box((0.014, LID_WIDTH, 0.020)),
        origin=Origin(xyz=(0.293, 0.000, 0.010)),
        material=body_mat,
        name="lid_front",
    )
    flatbed_lid.visual(
        Box((0.012, LID_WIDTH, 0.018)),
        origin=Origin(xyz=(0.006, 0.000, 0.009)),
        material=body_mat,
        name="lid_rear",
    )
    flatbed_lid.visual(
        Box((0.180, 0.010, 0.050)),
        origin=Origin(xyz=(0.120, -0.153, 0.049)),
        material=body_mat,
        name="adf_side_0",
    )
    flatbed_lid.visual(
        Box((0.180, 0.010, 0.050)),
        origin=Origin(xyz=(0.120, 0.153, 0.049)),
        material=body_mat,
        name="adf_side_1",
    )
    flatbed_lid.visual(
        Box((0.012, 0.306, 0.050)),
        origin=Origin(xyz=(0.036, 0.000, 0.049)),
        material=body_mat,
        name="adf_rear",
    )
    flatbed_lid.visual(
        Box((0.180, 0.306, 0.004)),
        origin=Origin(xyz=(0.120, 0.000, 0.072)),
        material=body_mat,
        name="adf_roof",
    )
    model.articulation(
        "body_to_flatbed_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flatbed_lid,
        origin=Origin(xyz=(-0.150, 0.000, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    adf_cover = model.part("adf_cover")
    adf_cover.visual(
        Box((0.166, 0.298, 0.004)),
        origin=Origin(xyz=(0.083, 0.000, 0.016)),
        material=body_mat,
        name="cover_top",
    )
    adf_cover.visual(
        Box((0.166, 0.010, 0.014)),
        origin=Origin(xyz=(0.083, -0.144, 0.007)),
        material=body_mat,
        name="cover_side_0",
    )
    adf_cover.visual(
        Box((0.166, 0.010, 0.014)),
        origin=Origin(xyz=(0.083, 0.144, 0.007)),
        material=body_mat,
        name="cover_side_1",
    )
    adf_cover.visual(
        Box((0.010, 0.298, 0.014)),
        origin=Origin(xyz=(0.161, 0.000, 0.007)),
        material=body_mat,
        name="cover_front",
    )
    adf_cover.visual(
        Box((0.010, 0.298, 0.014)),
        origin=Origin(xyz=(0.005, 0.000, 0.007)),
        material=body_mat,
        name="cover_rear",
    )
    model.articulation(
        "lid_to_adf_cover",
        ArticulationType.REVOLUTE,
        parent=flatbed_lid,
        child=adf_cover,
        origin=Origin(xyz=(0.034, 0.000, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    cassette = model.part("cassette")
    cassette.visual(
        Box((0.240, 0.320, 0.004)),
        origin=Origin(xyz=(-0.095, 0.000, 0.002)),
        material=trim_mat,
        name="tray_floor",
    )
    cassette.visual(
        Box((0.210, 0.010, 0.050)),
        origin=Origin(xyz=(-0.105, -0.155, 0.025)),
        material=trim_mat,
        name="tray_side_0",
    )
    cassette.visual(
        Box((0.210, 0.010, 0.050)),
        origin=Origin(xyz=(-0.105, 0.155, 0.025)),
        material=trim_mat,
        name="tray_side_1",
    )
    cassette.visual(
        Box((0.020, 0.320, 0.066)),
        origin=Origin(xyz=(0.015, 0.000, 0.033)),
        material=trim_mat,
        name="tray_front",
    )
    cassette.visual(
        Box((0.012, 0.320, 0.032)),
        origin=Origin(xyz=(-0.214, 0.000, 0.016)),
        material=trim_mat,
        name="tray_rear",
    )
    model.articulation(
        "body_to_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(0.155, 0.000, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=CASSETTE_TRAVEL,
        ),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_panel_shape(), "control_panel"),
        material=panel_mat,
        name="panel_shell",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.175, 0.140, 0.216)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.012,
                edge_radius=0.001,
                center=False,
            ),
            "selector_dial",
        ),
        material=trim_mat,
        name="dial_knob",
    )
    model.articulation(
        "control_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=selector_dial,
        origin=Origin(
            xyz=(0.024, 0.028, _panel_surface_z(0.024)),
            rpy=(0.0, -PANEL_PITCH, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.10,
            velocity=6.0,
        ),
    )

    button_mat = model.material("button_mat", rgba=(0.92, 0.93, 0.95, 1.0))
    button_positions = [
        (0.046, -0.024),
        (0.062, -0.024),
        (0.078, -0.024),
        (0.046, 0.000),
        (0.062, 0.000),
        (0.078, 0.000),
    ]
    for index, (x_pos, y_pos) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.014, 0.003)),
            origin=Origin(xyz=(0.000, 0.000, 0.0035)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.008, 0.002)),
            origin=Origin(xyz=(0.000, 0.000, 0.0010)),
            material=button_mat,
            name="button_stem",
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(
                xyz=(x_pos, y_pos, _panel_surface_z(x_pos)),
                rpy=(0.0, -PANEL_PITCH, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.001,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("flatbed_lid")
    body = object_model.get_part("body")
    adf_cover = object_model.get_part("adf_cover")
    cassette = object_model.get_part("cassette")
    control_panel = object_model.get_part("control_panel")
    selector_dial = object_model.get_part("selector_dial")
    button_0 = object_model.get_part("button_0")

    lid_hinge = object_model.get_articulation("body_to_flatbed_lid")
    adf_hinge = object_model.get_articulation("lid_to_adf_cover")
    tray_slide = object_model.get_articulation("body_to_cassette")
    panel_hinge = object_model.get_articulation("body_to_control_panel")
    dial_joint = object_model.get_articulation("control_panel_to_selector_dial")
    button_joint = object_model.get_articulation("control_panel_to_button_0")

    lid_rest = ctx.part_world_aabb(lid)
    ctx.check(
        "flatbed lid rests over the scanner bed",
        lid_rest is not None and 0.179 <= lid_rest[0][2] <= 0.181,
        details=f"rest={lid_rest}",
    )

    with ctx.pose({lid_hinge: math.radians(62.0)}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "flatbed lid opens upward",
        lid_rest is not None
        and lid_open is not None
        and lid_open[1][2] > lid_rest[1][2] + 0.10,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    adf_rest = ctx.part_world_aabb(adf_cover)
    with ctx.pose({adf_hinge: math.radians(45.0)}):
        adf_open = ctx.part_world_aabb(adf_cover)
    ctx.check(
        "adf cover lifts upward",
        adf_rest is not None
        and adf_open is not None
        and adf_open[1][2] > adf_rest[1][2] + 0.05,
        details=f"rest={adf_rest}, open={adf_open}",
    )

    cassette_rest = ctx.part_world_position(cassette)
    with ctx.pose({tray_slide: CASSETTE_TRAVEL}):
        cassette_open = ctx.part_world_position(cassette)
        ctx.expect_overlap(
            cassette,
            body,
            axes="yz",
            min_overlap=0.05,
            name="cassette stays aligned within the printer body",
        )
    ctx.check(
        "cassette slides outward from the front",
        cassette_rest is not None
        and cassette_open is not None
        and cassette_open[0] > cassette_rest[0] + 0.09,
        details=f"rest={cassette_rest}, open={cassette_open}",
    )

    panel_rest = ctx.part_world_aabb(control_panel)
    with ctx.pose({panel_hinge: math.radians(28.0)}):
        panel_open = ctx.part_world_aabb(control_panel)
    ctx.check(
        "control panel tilts upward",
        panel_rest is not None
        and panel_open is not None
        and panel_open[1][2] > panel_rest[1][2] + 0.03,
        details=f"rest={panel_rest}, open={panel_open}",
    )

    ctx.check(
        "selector dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"joint={dial_joint}",
    )

    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: 0.001}):
        button_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "keypad button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] > button_rest[0] + 0.0002
        and button_pressed[2] < button_rest[2] - 0.0008,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    button_joints = [object_model.get_articulation(f"control_panel_to_button_{index}") for index in range(6)]
    ctx.check(
        "all visible keypad buttons are independently articulated",
        len(button_joints) == 6
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints),
        details=f"button_joints={[joint.name for joint in button_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
