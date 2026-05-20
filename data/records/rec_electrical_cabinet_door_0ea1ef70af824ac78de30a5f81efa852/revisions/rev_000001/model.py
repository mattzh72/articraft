from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cabinet_geometry() -> cq.Workplane:
    width = 0.72
    depth = 0.46
    height = 1.05
    wall = 0.032

    # A welded MCC enclosure: side/top/bottom/rear shell plus a proud front
    # flange around the compartment opening.
    outer = _cq_box((width, depth, height), (0.0, depth / 2.0, height / 2.0))
    cavity = _cq_box(
        (width - 2.0 * wall, depth - wall + 0.025, height - 2.0 * wall),
        (0.0, (depth - wall - 0.025) / 2.0, height / 2.0),
    )
    shell = outer.cut(cavity)

    front_depth = 0.034
    parts = [
        shell,
        _cq_box((width, front_depth, 0.135), (0.0, front_depth / 2.0, 0.0675)),
        _cq_box((width, front_depth, 0.135), (0.0, front_depth / 2.0, height - 0.0675)),
        _cq_box((0.070, front_depth, 0.780), (-0.325, front_depth / 2.0, 0.520)),
        _cq_box((0.070, front_depth, 0.780), (0.325, front_depth / 2.0, 0.520)),
        # Shallow top nameplate channel and lower kick strip.
        _cq_box((0.60, 0.012, 0.040), (0.0, -0.006, 0.975)),
        _cq_box((0.64, 0.016, 0.035), (0.0, -0.008, 0.075)),
        # Interior rear back-pan mounting rails, visible through the door opening.
        _cq_box((0.52, 0.014, 0.030), (0.0, 0.405, 0.255)),
        _cq_box((0.52, 0.014, 0.030), (0.0, 0.405, 0.775)),
    ]
    body = parts[0]
    for item in parts[1:]:
        body = body.union(item)
    return body


def _bucket_geometry() -> cq.Workplane:
    # Child frame is at the front lower seating plane of the bucket.
    body = _cq_box((0.47, 0.020, 0.60), (0.0, 0.012, 0.300))  # front control pan
    additions = [
        _cq_box((0.045, 0.315, 0.560), (-0.235, 0.180, 0.300)),  # left tray cheek
        _cq_box((0.045, 0.315, 0.560), (0.235, 0.180, 0.300)),  # right tray cheek
        _cq_box((0.47, 0.315, 0.035), (0.0, 0.180, 0.0175)),  # bottom tray
        _cq_box((0.47, 0.030, 0.56), (0.0, 0.335, 0.300)),  # rear back pan
        _cq_box((0.34, 0.038, 0.110), (0.0, 0.304, 0.420)),  # contactor block
        _cq_box((0.26, 0.035, 0.090), (0.0, 0.300, 0.230)),  # control transformer
        _cq_box((0.030, 0.070, 0.180), (-0.120, 0.375, 0.435)),  # copper stab carrier
        _cq_box((0.030, 0.070, 0.180), (0.000, 0.375, 0.435)),
        _cq_box((0.030, 0.070, 0.180), (0.120, 0.375, 0.435)),
        _cq_box((0.022, 0.300, 0.022), (-0.255, 0.175, 0.072)),  # sliding runner
        _cq_box((0.022, 0.300, 0.022), (0.255, 0.175, 0.072)),
    ]
    for item in additions:
        body = body.union(item)
    return body

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motor_control_center_section")

    steel = model.material("powder_coated_steel", rgba=(0.63, 0.68, 0.70, 1.0))
    door_blue = model.material("mcc_door_blue_gray", rgba=(0.38, 0.46, 0.50, 1.0))
    dark = model.material("dark_gasket", rgba=(0.015, 0.017, 0.018, 1.0))
    rail_mat = model.material("zinc_rail", rgba=(0.72, 0.73, 0.70, 1.0))
    bucket_mat = model.material("bucket_white_steel", rgba=(0.82, 0.84, 0.80, 1.0))
    copper = model.material("copper_bus", rgba=(0.80, 0.42, 0.16, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.022, 1.0))
    red = model.material("red_button", rgba=(0.75, 0.04, 0.035, 1.0))
    green = model.material("green_button", rgba=(0.05, 0.45, 0.12, 1.0))
    amber = model.material("amber_lens", rgba=(1.0, 0.55, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet_depth = 0.520
    cabinet.visual(Box((0.038, cabinet_depth, 1.050)), origin=Origin(xyz=(-0.341, cabinet_depth / 2.0, 0.525)), material=steel, name="side_wall_0")
    cabinet.visual(Box((0.038, cabinet_depth, 1.050)), origin=Origin(xyz=(0.341, cabinet_depth / 2.0, 0.525)), material=steel, name="side_wall_1")
    cabinet.visual(Box((0.720, 0.030, 1.050)), origin=Origin(xyz=(0.0, cabinet_depth - 0.015, 0.525)), material=steel, name="back_panel")
    cabinet.visual(Box((0.720, cabinet_depth, 0.038)), origin=Origin(xyz=(0.0, cabinet_depth / 2.0, 1.031)), material=steel, name="top_panel")
    cabinet.visual(Box((0.720, cabinet_depth, 0.038)), origin=Origin(xyz=(0.0, cabinet_depth / 2.0, 0.019)), material=steel, name="bottom_panel")
    cabinet.visual(Box((0.720, 0.035, 0.135)), origin=Origin(xyz=(0.0, 0.0175, 0.0675)), material=steel, name="front_bottom_band")
    cabinet.visual(Box((0.720, 0.035, 0.135)), origin=Origin(xyz=(0.0, 0.0175, 0.9825)), material=steel, name="front_top_band")
    cabinet.visual(Box((0.076, 0.035, 0.780)), origin=Origin(xyz=(-0.322, 0.0175, 0.520)), material=steel, name="front_stile_0")
    cabinet.visual(Box((0.076, 0.035, 0.780)), origin=Origin(xyz=(0.322, 0.0175, 0.520)), material=steel, name="front_stile_1")
    cabinet.visual(Box((0.600, 0.012, 0.040)), origin=Origin(xyz=(0.0, -0.006, 0.975)), material=steel, name="nameplate_channel")
    cabinet.visual(Box((0.640, 0.016, 0.035)), origin=Origin(xyz=(0.0, -0.008, 0.075)), material=steel, name="kick_strip")
    cabinet.visual(Box((0.520, 0.014, 0.030)), origin=Origin(xyz=(0.0, 0.483, 0.255)), material=rail_mat, name="rear_mount_0")
    cabinet.visual(Box((0.520, 0.014, 0.030)), origin=Origin(xyz=(0.0, 0.483, 0.775)), material=rail_mat, name="rear_mount_1")

    # Stationary guide rails inside the compartment opening.
    cabinet.visual(Box((0.056, 0.360, 0.026)), origin=Origin(xyz=(-0.294, 0.195, 0.272)), material=rail_mat, name="guide_rail_0")
    cabinet.visual(Box((0.056, 0.360, 0.018)), origin=Origin(xyz=(-0.294, 0.195, 0.730)), material=rail_mat, name="upper_rail_0")
    cabinet.visual(Box((0.056, 0.360, 0.026)), origin=Origin(xyz=(0.294, 0.195, 0.272)), material=rail_mat, name="guide_rail_1")
    cabinet.visual(Box((0.056, 0.360, 0.018)), origin=Origin(xyz=(0.294, 0.195, 0.730)), material=rail_mat, name="upper_rail_1")

    # Fixed hinge leaves and alternating stationary knuckles on the left edge.
    hinge_x = -0.310
    hinge_y = -0.014
    door_bottom = 0.140
    fixed_segments = ((0.020, 0.145), (0.315, 0.445), (0.615, 0.740))
    for idx, (z0, z1) in enumerate(fixed_segments):
        length = z1 - z0
        zc = door_bottom + (z0 + z1) / 2.0
        cabinet.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=rail_mat,
            name=f"fixed_knuckle_{idx}",
        )
        cabinet.visual(
            Box((0.040, 0.008, length)),
            origin=Origin(xyz=(hinge_x + 0.014, 0.004, zc)),
            material=rail_mat,
            name=f"fixed_leaf_{idx}",
        )

    door = model.part("door")
    door_width = 0.560
    door_height = 0.760
    door_thickness = 0.025
    hinge_offset = 0.030
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(hinge_offset + door_width / 2.0, 0.0015, door_height / 2.0)),
        material=door_blue,
        name="door_panel",
    )
    # Raised perimeter ribs and a dark inner gasket make the panel read as a
    # real sheet-metal compartment door instead of a plain slab.
    door.visual(
        Box((door_width - 0.060, 0.008, 0.030)),
        origin=Origin(xyz=(hinge_offset + door_width / 2.0, -0.010, door_height - 0.050)),
        material=steel,
        name="top_rib",
    )
    door.visual(
        Box((door_width - 0.060, 0.008, 0.030)),
        origin=Origin(xyz=(hinge_offset + door_width / 2.0, -0.010, 0.050)),
        material=steel,
        name="bottom_rib",
    )
    for side, x in (("0", hinge_offset + 0.030), ("1", hinge_offset + door_width - 0.030)):
        door.visual(
            Box((0.030, 0.008, door_height - 0.060)),
            origin=Origin(xyz=(x, -0.010, door_height / 2.0)),
            material=steel,
            name=f"side_rib_{side}",
        )
    door.visual(
        Box((door_width - 0.110, 0.004, door_height - 0.120)),
        origin=Origin(xyz=(hinge_offset + door_width / 2.0, 0.016, door_height / 2.0)),
        material=dark,
        name="door_gasket",
    )
    door.visual(
        Box((0.130, 0.008, 0.045)),
        origin=Origin(xyz=(hinge_offset + door_width - 0.115, -0.010, 0.430)),
        material=black,
        name="latch_paddle",
    )
    door.visual(
        Box((0.150, 0.008, 0.040)),
        origin=Origin(xyz=(hinge_offset + 0.175, -0.010, 0.660)),
        material=amber,
        name="warning_label",
    )

    moving_segments = ((0.155, 0.300), (0.460, 0.605))
    for idx, (z0, z1) in enumerate(moving_segments):
        length = z1 - z0
        zc = (z0 + z1) / 2.0
        door.visual(
            Cylinder(radius=0.013, length=length),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=rail_mat,
            name=f"door_knuckle_{idx}",
        )
        door.visual(
            Box((0.048, 0.008, length)),
            origin=Origin(xyz=(0.027, -0.006, zc)),
            material=rail_mat,
            name=f"door_leaf_{idx}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    bucket = model.part("bucket")
    bucket.visual(Box((0.470, 0.020, 0.600)), origin=Origin(xyz=(0.0, 0.012, 0.300)), material=bucket_mat, name="front_panel")
    bucket.visual(Box((0.045, 0.330, 0.560)), origin=Origin(xyz=(-0.235, 0.180, 0.300)), material=bucket_mat, name="tray_side_0")
    bucket.visual(Box((0.045, 0.330, 0.560)), origin=Origin(xyz=(0.235, 0.180, 0.300)), material=bucket_mat, name="tray_side_1")
    bucket.visual(Box((0.470, 0.330, 0.035)), origin=Origin(xyz=(0.0, 0.180, 0.0175)), material=bucket_mat, name="bottom_tray")
    bucket.visual(Box((0.470, 0.030, 0.560)), origin=Origin(xyz=(0.0, 0.350, 0.300)), material=bucket_mat, name="rear_pan")
    bucket.visual(Box((0.340, 0.045, 0.110)), origin=Origin(xyz=(0.0, 0.322, 0.420)), material=black, name="contactor_block")
    bucket.visual(Box((0.260, 0.045, 0.090)), origin=Origin(xyz=(0.0, 0.322, 0.230)), material=black, name="control_transformer")
    for idx, x in enumerate((-0.120, 0.0, 0.120)):
        bucket.visual(Box((0.030, 0.070, 0.180)), origin=Origin(xyz=(x, 0.385, 0.435)), material=black, name=f"stab_carrier_{idx}")
    bucket.visual(Box((0.022, 0.300, 0.022)), origin=Origin(xyz=(-0.255, 0.175, 0.072)), material=rail_mat, name="bucket_runner_0")
    bucket.visual(Box((0.022, 0.300, 0.022)), origin=Origin(xyz=(0.255, 0.175, 0.072)), material=rail_mat, name="bucket_runner_1")
    # Separate copper stabs are flush with the molded carriers and remain part of
    # the sliding bucket assembly.
    for idx, x in enumerate((-0.120, 0.0, 0.120)):
        bucket.visual(
            Box((0.018, 0.070, 0.110)),
            origin=Origin(xyz=(x, 0.395, 0.435)),
            material=copper,
            name=f"bus_stab_{idx}",
        )

    model.articulation(
        "bucket_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.045, 0.200)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.300),
    )

    selector = model.part("selector_knob")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.030,
                body_style="faceted",
                top_diameter=0.040,
                grip=KnobGrip(style="ribbed", count=16, depth=0.001),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    model.articulation(
        "selector_turn",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=selector,
        origin=Origin(xyz=(0.110, 0.002, 0.462)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.57, upper=1.57),
    )

    for name, x, z, mat in (
        ("start_button", -0.100, 0.462, green),
        ("stop_button", -0.020, 0.462, red),
    ):
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=bucket,
            child=button,
            origin=Origin(xyz=(x, 0.002, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    bucket = object_model.get_part("bucket")
    selector = object_model.get_part("selector_knob")
    start = object_model.get_part("start_button")
    stop = object_model.get_part("stop_button")
    hinge = object_model.get_articulation("door_hinge")
    slide = object_model.get_articulation("bucket_slide")
    selector_joint = object_model.get_articulation("selector_turn")

    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.45,
        elem_a="door_panel",
        elem_b="back_panel",
        name="closed door covers the compartment opening",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="front_stile_0",
        negative_elem="door_panel",
        name="closed door seats on the front frame without deep overlap",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.30}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the left hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.22,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_within(
        bucket,
        cabinet,
        axes="xz",
        margin=0.012,
        elem_a="front_panel",
        elem_b="back_panel",
        name="inserted bucket is centered in the compartment rails",
    )

    rest_pos = ctx.part_world_position(bucket)
    with ctx.pose({hinge: 1.30, slide: 0.300}):
        extended_pos = ctx.part_world_position(bucket)
        ctx.expect_within(
            bucket,
            cabinet,
            axes="xz",
            margin=0.012,
            elem_a="front_panel",
            elem_b="back_panel",
            name="extended bucket stays aligned to the guide rails",
        )
        ctx.expect_overlap(
            bucket,
            cabinet,
            axes="y",
            min_overlap=0.050,
            elem_a="bucket_runner_0",
            elem_b="guide_rail_0",
            name="extended bucket retains insertion in the enclosure",
        )
    ctx.check(
        "bucket slides outward through the door opening",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        selector,
        bucket,
        elem_a="knob_cap",
        elem_b="front_panel",
        contact_tol=0.003,
        name="selector knob is mounted on the bucket face",
    )
    ctx.expect_contact(
        start,
        bucket,
        elem_a="button_cap",
        elem_b="front_panel",
        contact_tol=0.003,
        name="start button is mounted on the bucket face",
    )
    ctx.expect_contact(
        stop,
        bucket,
        elem_a="button_cap",
        elem_b="front_panel",
        contact_tol=0.003,
        name="stop button is mounted on the bucket face",
    )
    with ctx.pose({selector_joint: 1.0}):
        ctx.expect_contact(
            selector,
            bucket,
            elem_a="knob_cap",
            elem_b="front_panel",
            contact_tol=0.004,
            name="selector remains seated while turned",
        )

    return ctx.report()


object_model = build_object_model()
