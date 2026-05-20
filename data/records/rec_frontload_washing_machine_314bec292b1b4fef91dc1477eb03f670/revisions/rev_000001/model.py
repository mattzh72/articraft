from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 1.16
BODY_DEPTH = 0.78
BODY_HEIGHT = 1.15
FRONT_Y = -BODY_DEPTH / 2.0
DRUM_Z = 0.67
HINGE_X = -0.43
DOOR_HINGE_Y = -0.48
DOOR_CENTER_X = -HINGE_X


def _annular_y(center_x: float, center_z: float, outer_r: float, inner_r: float, depth: float):
    """Annular disk/tube with its axis along CadQuery global Y."""
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(depth / 2.0, both=True)
    )


def _disk_y(center_x: float, center_z: float, radius: float, depth: float):
    """Solid circular disk with its axis along CadQuery global Y."""
    return cq.Workplane("XZ").center(center_x, center_z).circle(radius).extrude(depth / 2.0, both=True)


def _body_cadquery():
    body = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, 0.0, BODY_HEIGHT / 2.0)
    )
    porthole_cutter = (
        cq.Workplane("XZ")
        .center(0.0, DRUM_Z)
        .circle(0.35)
        .extrude(BODY_DEPTH + 0.30, both=True)
    )
    return body.cut(porthole_cutter)


def _drum_cadquery():
    shell = _annular_y(0.0, 0.0, 0.295, 0.265, 0.50)
    rear_plate = _disk_y(0.0, 0.0, 0.274, 0.018).translate((0.0, 0.247, 0.0))
    front_rolled_lip = _annular_y(0.0, 0.0, 0.312, 0.238, 0.055).translate(
        (0.0, -0.238, 0.0)
    )

    drum = shell.union(rear_plate).union(front_rolled_lip)
    base_lifter = cq.Workplane("XY").box(0.060, 0.36, 0.040).translate((0.0, 0.0, 0.247))
    for angle in (0.0, 120.0, 240.0):
        drum = drum.union(base_lifter.rotate((0, 0, 0), (0, 1, 0), angle))
    return drum


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_front_load_laundry_machine")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    darker_stainless = model.material("dark_stainless", rgba=(0.44, 0.46, 0.46, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.83, 0.86, 0.86, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.06, 0.10, 0.13, 0.45))
    shadow = model.material("deep_shadow", rgba=(0.01, 0.011, 0.012, 1.0))
    display_blue = model.material("status_glass", rgba=(0.02, 0.07, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_cadquery(), "stainless_body", tolerance=0.003, angular_tolerance=0.08),
        material=stainless,
        name="stainless_body",
    )
    body.visual(
        mesh_from_cadquery(
            _annular_y(0.0, DRUM_Z, 0.405, 0.370, 0.018),
            "front_trim",
            tolerance=0.0015,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, 0.0)),
        material=chrome,
        name="front_trim",
    )
    body.visual(
        mesh_from_cadquery(
            _annular_y(0.0, DRUM_Z, 0.370, 0.318, 0.035),
            "front_gasket",
            tolerance=0.0015,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.017, 0.0)),
        material=rubber,
        name="front_gasket",
    )
    body.visual(
        Box((0.98, 0.014, 0.085)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.005, 1.035)),
        material=darker_stainless,
        name="control_strip",
    )
    body.visual(
        Box((0.22, 0.016, 0.045)),
        origin=Origin(xyz=(0.30, FRONT_Y - 0.014, 1.036)),
        material=display_blue,
        name="status_window",
    )
    body.visual(
        Box((0.30, 0.018, 0.055)),
        origin=Origin(xyz=(-0.25, FRONT_Y - 0.013, 1.035)),
        material=shadow,
        name="brand_plate",
    )
    for i, x in enumerate((-0.2425, 0.2425)):
        body.visual(
            Box((0.335, 0.040, 0.034)),
            origin=Origin(xyz=(x, 0.310, DRUM_Z)),
            material=darker_stainless,
            name=f"bearing_strut_{i}",
        )
    for i, z in enumerate((DRUM_Z - 0.230, DRUM_Z + 0.230), start=2):
        body.visual(
            Box((0.034, 0.040, 0.310)),
            origin=Origin(xyz=(0.0, 0.310, z)),
            material=darker_stainless,
            name=f"bearing_strut_{i}",
        )
    body.visual(
        mesh_from_cadquery(
            _annular_y(0.0, DRUM_Z, 0.075, 0.043, 0.052),
            "rear_bearing",
            tolerance=0.001,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.310, 0.0)),
        material=chrome,
        name="rear_bearing",
    )

    for i, z in enumerate((DRUM_Z + 0.245, DRUM_Z - 0.245)):
        body.visual(
            Box((0.115, 0.052, 0.078)),
            origin=Origin(xyz=(HINGE_X - 0.090, FRONT_Y - 0.026, z)),
            material=chrome,
            name=f"door_hinge_mount_{i}",
        )

    filter_x = 0.33
    filter_hinge_y = FRONT_Y - 0.058
    filter_hinge_z = 0.230
    body.visual(
        Box((0.235, 0.012, 0.105)),
        origin=Origin(xyz=(filter_x, FRONT_Y - 0.006, filter_hinge_z - 0.065)),
        material=shadow,
        name="filter_recess",
    )
    for i, x_offset in enumerate((-0.141, 0.141)):
        body.visual(
            Box((0.032, 0.072, 0.048)),
            origin=Origin(xyz=(filter_x + x_offset, (FRONT_Y + filter_hinge_y) / 2.0, filter_hinge_z)),
            material=chrome,
            name=f"filter_hinge_support_{i}",
        )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_cadquery(), "drum_shell", tolerance=0.002, angular_tolerance=0.08),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.046, length=0.180),
        origin=Origin(xyz=(0.0, 0.310, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="drum_axle",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(
            _annular_y(DOOR_CENTER_X, 0.0, 0.420, 0.292, 0.075),
            "door_steel_ring",
            tolerance=0.0015,
            angular_tolerance=0.06,
        ),
        material=chrome,
        name="door_steel_ring",
    )
    door.visual(
        mesh_from_cadquery(
            _annular_y(DOOR_CENTER_X, 0.0, 0.335, 0.275, 0.040),
            "door_rubber_lip",
            tolerance=0.0015,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.0355, 0.0)),
        material=rubber,
        name="door_rubber_lip",
    )
    door.visual(
        mesh_from_cadquery(
            _disk_y(DOOR_CENTER_X, 0.0, 0.302, 0.022),
            "door_glass",
            tolerance=0.0015,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=dark_glass,
        name="door_glass",
    )
    for i, z in enumerate((0.245, -0.245)):
        door.visual(
            Cylinder(radius=0.027, length=0.112),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"door_hinge_barrel_{i}",
        )
        door.visual(
            Box((0.180, 0.052, 0.074)),
            origin=Origin(xyz=(0.075, 0.0, z)),
            material=chrome,
            name=f"door_hinge_leaf_{i}",
        )
    door.visual(
        Box((0.048, 0.085, 0.245)),
        origin=Origin(xyz=(DOOR_CENTER_X + 0.355, -0.025, 0.0)),
        material=rubber,
        name="door_pull",
    )

    filter_cap = model.part("filter_cap")
    filter_cap.visual(
        Box((0.250, 0.035, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=darker_stainless,
        name="filter_panel",
    )
    filter_cap.visual(
        Cylinder(radius=0.014, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="filter_hinge_barrel",
    )
    filter_cap.visual(
        Box((0.120, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.021, -0.105)),
        material=rubber,
        name="filter_grip",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=18.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, DOOR_HINGE_Y, DRUM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_filter_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_cap,
        origin=Origin(xyz=(filter_x, filter_hinge_y, filter_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    filter_cap = object_model.get_part("filter_cap")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    filter_hinge = object_model.get_articulation("body_to_filter_cap")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing",
        elem_b="drum_axle",
        reason="The rotating drum axle is intentionally captured inside the rear bearing sleeve.",
    )

    ctx.check(
        "drum axle rotates about front-to-back axis",
        tuple(round(v, 3) for v in drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drum_joint.axis}",
    )
    ctx.check(
        "door and lint cap are revolute hinges",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and filter_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"door={door_hinge.articulation_type}, filter={filter_hinge.articulation_type}",
    )

    ctx.expect_within(
        drum,
        body,
        axes="xz",
        margin=0.010,
        inner_elem="drum_shell",
        outer_elem="stainless_body",
        name="drum sits inside the circular cabinet opening",
    )
    ctx.expect_within(
        drum,
        body,
        axes="xz",
        margin=0.002,
        inner_elem="drum_axle",
        outer_elem="rear_bearing",
        name="drum axle remains centered in the rear bearing",
    )
    ctx.expect_overlap(
        drum,
        body,
        axes="y",
        elem_a="drum_axle",
        elem_b="rear_bearing",
        min_overlap=0.040,
        name="drum axle is retained inside the bearing sleeve",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_gasket",
        negative_elem="door_rubber_lip",
        max_gap=0.018,
        max_penetration=0.001,
        name="closed door compresses near the rubber gasket without broad collision",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_glass",
        elem_b="front_gasket",
        min_overlap=0.15,
        name="porthole glass is centered over the front gasket",
    )
    ctx.expect_gap(
        body,
        filter_cap,
        axis="y",
        positive_elem="filter_recess",
        negative_elem="filter_panel",
        min_gap=0.025,
        max_gap=0.060,
        name="lint filter cap stands proud of recessed filter opening",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "left hinged porthole door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.30,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_cap_aabb = ctx.part_world_aabb(filter_cap)
    with ctx.pose({filter_hinge: filter_hinge.motion_limits.upper}):
        open_cap_aabb = ctx.part_world_aabb(filter_cap)
    ctx.check(
        "lint filter cap hinges outward from the front face",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[0][1] < closed_cap_aabb[0][1] - 0.045,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
