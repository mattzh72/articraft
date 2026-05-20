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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery cuboid with rounded vertical corners, centered locally."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _body_shell() -> cq.Workplane:
    """Deep battery-housing body with a softly radiused professional shell."""
    body = cq.Workplane("XY").box(0.072, 0.078, 0.135).edges("|Z").fillet(0.008)
    # A very shallow rear control-panel recess keeps the body from reading as a
    # plain box while staying part of the molded shell.
    rear_boss = (
        cq.Workplane("XY")
        .box(0.058, 0.004, 0.070)
        .edges("|Z")
        .fillet(0.001)
        .translate((0.0, -0.041, 0.010))
    )
    return body.union(rear_boss).translate((0.0, 0.0, 0.0675))


def _head_shell() -> cq.Workplane:
    """Wide flash head: central block plus rounded side cheeks."""
    main = _rounded_box((0.098, 0.064, 0.048), 0.006)
    cheek_l = _rounded_box((0.020, 0.068, 0.052), 0.008).translate((-0.054, 0.0, 0.0))
    cheek_r = _rounded_box((0.020, 0.068, 0.052), 0.008).translate((0.054, 0.0, 0.0))
    return main.union(cheek_l).union(cheek_r)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_speedlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.052, 0.048, 1.0))
    dark_groove = model.material("dark_groove", rgba=(0.002, 0.002, 0.002, 1.0))
    warm_lens = model.material("warm_lens", rgba=(1.0, 0.88, 0.58, 0.58))
    glass = model.material("smoked_glass", rgba=(0.04, 0.06, 0.07, 0.72))
    metal = model.material("brushed_metal", rgba=(0.55, 0.55, 0.50, 1.0))
    red_window = model.material("red_window", rgba=(0.55, 0.03, 0.02, 0.82))

    body_w = 0.072
    body_d = 0.078
    body_h = 0.135
    side_x = body_w / 2.0

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0008),
        material=matte_black,
        name="body_shell",
    )

    # Professional flash foot and locking collar, connected to the molded body.
    body.visual(
        Box((0.050, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=satin_black,
        name="locking_collar",
    )
    body.visual(
        Box((0.044, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.008, 0.030, 0.004)),
        origin=Origin(xyz=(-0.021, 0.0, -0.012)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.008, 0.030, 0.004)),
        origin=Origin(xyz=(0.021, 0.0, -0.012)),
        material=metal,
        name="shoe_rail_1",
    )

    # Fixed side seam/gasket showing the battery door's cut line in the body shell.
    hinge_y = -0.027
    door_width_y = 0.050
    door_center_y = hinge_y + door_width_y / 2.0
    door_center_z = 0.070
    door_height_z = 0.088
    for name, y in (("door_seam_rear", hinge_y), ("door_seam_front", hinge_y + door_width_y)):
        body.visual(
            Box((0.0008, 0.0012, door_height_z)),
            origin=Origin(xyz=(side_x + 0.0002, y, door_center_z)),
            material=dark_groove,
            name=name,
        )
    for name, z in (("door_seam_lower", door_center_z - door_height_z / 2.0), ("door_seam_upper", door_center_z + door_height_z / 2.0)):
        body.visual(
            Box((0.0008, door_width_y, 0.0012)),
            origin=Origin(xyz=(side_x + 0.0002, door_center_y, z)),
            material=dark_groove,
            name=name,
        )

    # Rear display and front assist window are surface treatments on the body, not moving controls.
    body.visual(
        Box((0.046, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 - 0.005, 0.088)),
        material=glass,
        name="rear_display",
    )
    body.visual(
        Box((0.028, 0.002, 0.014)),
        origin=Origin(xyz=(0.0, body_d / 2.0 + 0.001, 0.047)),
        material=red_window,
        name="assist_window",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.030, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=matte_black,
        name="neck_column",
    )
    yoke = TrunnionYokeGeometry(
        (0.086, 0.040, 0.062),
        span_width=0.054,
        trunnion_diameter=0.018,
        trunnion_center_z=0.046,
        base_thickness=0.014,
        corner_radius=0.004,
        center=False,
    )
    neck.visual(
        mesh_from_geometry(yoke, "head_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_black,
        name="head_yoke",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "flash_shell", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.025, 0.044)),
        material=satin_black,
        name="flash_shell",
    )
    head.visual(
        Box((0.086, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, 0.059, 0.046)),
        material=warm_lens,
        name="flash_lens",
    )
    head.visual(
        Box((0.042, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.008, 0.012)),
        material=satin_black,
        name="tilt_saddle",
    )
    head.visual(
        Cylinder(radius=0.0088, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_pin",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="side_cheek_0",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="side_cheek_1",
    )

    door = model.part("battery_door")
    door.visual(
        Box((0.003, door_width_y, door_height_z)),
        origin=Origin(xyz=(-0.0015, door_width_y / 2.0, 0.0)),
        material=satin_black,
        name="door_panel",
    )
    door.visual(
        Box((0.0010, door_width_y - 0.008, door_height_z - 0.010)),
        origin=Origin(xyz=(0.0004, door_width_y / 2.0, 0.0)),
        material=dark_groove,
        name="door_recess",
    )
    for idx, z in enumerate((-0.030, 0.0, 0.030)):
        door.visual(
            Cylinder(radius=0.0030, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_black,
            name=f"hinge_knuckle_{idx}",
        )

    model.articulation(
        "body_to_neck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, body_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.25, upper=1.75),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(side_x + 0.0035, hinge_y, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    door = object_model.get_part("battery_door")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="swivel_collar",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0005,
        name="swivel collar sits on body top",
    )
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    head_aabb = ctx.part_element_world_aabb(head, elem="flash_shell")
    ctx.check(
        "flash head is wider than body",
        body_aabb is not None
        and head_aabb is not None
        and (head_aabb[1][0] - head_aabb[0][0]) > (body_aabb[1][0] - body_aabb[0][0]) + 0.025,
        details=f"body={body_aabb}, head={head_aabb}",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.004,
        name="battery door is carried on side shell",
    )

    rest_head_aabb = ctx.part_element_world_aabb(head, elem="flash_shell")
    with ctx.pose({tilt: 1.2}):
        tilted_head_aabb = ctx.part_element_world_aabb(head, elem="flash_shell")
    ctx.check(
        "head tilts upward on horizontal hinge",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.010,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    rest_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "battery door opens outward from side",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.020,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_origin_distance(
            neck,
            body,
            axes="xy",
            max_dist=0.001,
            name="continuous swivel stays on vertical axis",
        )

    return ctx.report()


object_model = build_object_model()
