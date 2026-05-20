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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.55
CABINET_D = 0.50
CABINET_H = 0.45

DOOR_W = 0.41
DOOR_H = 0.372
DOOR_T = 0.028
DOOR_X = -0.045
DOOR_HINGE_Z = 0.028

CHAMBER_W = 0.382
CHAMBER_D = 0.470
CHAMBER_H = 0.340
CHAMBER_X = DOOR_X
CHAMBER_Y = -0.003
CHAMBER_Z = 0.052

POD_W = 0.110
POD_D = 0.030
POD_H = 0.332
POD_X = 0.218
POD_Z = 0.062

RACK_RUNNER_X = 0.157
RACK_ORIGIN_Y = -0.208
RACK_ORIGIN_Z = 0.180


def _translated_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    centered: tuple[bool, bool, bool] = (True, True, True),
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=centered).translate(center)


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(CABINET_W, CABINET_D, CABINET_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
        .edges(">Z")
        .fillet(0.010)
    )

    chamber = _translated_box(
        (CHAMBER_W, CHAMBER_D, CHAMBER_H),
        (CHAMBER_X, CHAMBER_Y, CHAMBER_Z),
        centered=(True, True, False),
    )
    front_opening = _translated_box(
        (0.392, 0.090, 0.344),
        (DOOR_X, -CABINET_D / 2.0 + 0.045, 0.044),
        centered=(True, True, False),
    )
    return shell.cut(chamber.val()).cut(front_opening.val())


def _control_pod_shape() -> cq.Workplane:
    pod = (
        cq.Workplane("XY")
        .box(POD_W, POD_D, POD_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((POD_X, -CABINET_D / 2.0 - POD_D / 2.0 + 0.001, POD_Z))
    )

    button_pocket_size = (0.040, 0.011, 0.030)
    button_x = POD_X
    button_y = -CABINET_D / 2.0 - POD_D + 0.0055
    for z in (0.152, 0.205):
        pod = pod.cut(
            _translated_box(
                button_pocket_size,
                (button_x, button_y, z),
                centered=(True, True, True),
            )
        )

    dial_seat = (
        cq.Workplane("XZ")
        .circle(0.030)
        .extrude(0.004)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((POD_X, -CABINET_D / 2.0 - POD_D + 0.004, 0.296))
    )
    return pod.cut(dial_seat)


def _door_shape() -> cq.Workplane:
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )

    liner_cut = _translated_box(
        (DOOR_W - 0.048, DOOR_T - 0.008, DOOR_H - 0.050),
        (0.0, DOOR_T - (DOOR_T - 0.008) / 2.0, 0.022 + (DOOR_H - 0.050) / 2.0),
        centered=(True, True, True),
    )
    return door.cut(liner_cut.val())


def _union_shapes(shapes: list[cq.Workplane]) -> cq.Workplane:
    merged = shapes[0].val()
    for shape in shapes[1:]:
        merged = merged.fuse(shape.val())
    return cq.Workplane(obj=merged)


def _rack_shape() -> cq.Workplane:
    shapes = [
        _translated_box((0.014, 0.400, 0.006), (-RACK_RUNNER_X, 0.200, 0.000)),
        _translated_box((0.014, 0.400, 0.006), (RACK_RUNNER_X, 0.200, 0.000)),
        _translated_box((0.012, 0.022, 0.060), (-RACK_RUNNER_X, 0.048, -0.022)),
        _translated_box((0.012, 0.022, 0.060), (RACK_RUNNER_X, 0.048, -0.022)),
        _translated_box((0.012, 0.022, 0.060), (-RACK_RUNNER_X, 0.314, -0.022)),
        _translated_box((0.012, 0.022, 0.060), (RACK_RUNNER_X, 0.314, -0.022)),
        _translated_box((0.292, 0.006, 0.006), (0.000, 0.042, -0.050)),
        _translated_box((0.292, 0.006, 0.006), (0.000, 0.330, -0.050)),
        _translated_box((0.006, 0.288, 0.006), (-0.146, 0.186, -0.050)),
        _translated_box((0.006, 0.288, 0.006), (0.146, 0.186, -0.050)),
        _translated_box((0.292, 0.006, 0.006), (0.000, 0.042, 0.044)),
        _translated_box((0.292, 0.006, 0.006), (0.000, 0.330, 0.044)),
        _translated_box((0.006, 0.288, 0.006), (-0.146, 0.186, 0.044)),
        _translated_box((0.006, 0.288, 0.006), (0.146, 0.186, 0.044)),
        _translated_box((0.006, 0.006, 0.094), (-0.146, 0.042, -0.003)),
        _translated_box((0.006, 0.006, 0.094), (0.146, 0.042, -0.003)),
        _translated_box((0.006, 0.006, 0.094), (-0.146, 0.330, -0.003)),
        _translated_box((0.006, 0.006, 0.094), (0.146, 0.330, -0.003)),
        _translated_box((0.006, 0.220, 0.006), (-0.075, 0.186, -0.050)),
        _translated_box((0.006, 0.220, 0.006), (0.000, 0.186, -0.050)),
        _translated_box((0.006, 0.220, 0.006), (0.075, 0.186, -0.050)),
        _translated_box((0.150, 0.010, 0.016), (0.000, -0.008, 0.030)),
        _translated_box((0.006, 0.020, 0.050), (-0.060, 0.120, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.000, 0.120, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.060, 0.120, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (-0.060, 0.200, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.000, 0.200, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.060, 0.200, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (-0.060, 0.280, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.000, 0.280, -0.025)),
        _translated_box((0.006, 0.020, 0.050), (0.060, 0.280, -0.025)),
    ]
    return _union_shapes(shapes)


def _cup_shelf_shape() -> cq.Workplane:
    shapes = [
        _translated_box((0.024, 0.010, 0.010), (-0.159, 0.000, 0.000)),
        _translated_box((0.024, 0.010, 0.010), (0.159, 0.000, 0.000)),
        _translated_box((0.010, 0.016, 0.018), (-0.150, -0.010, -0.010)),
        _translated_box((0.010, 0.016, 0.018), (0.150, -0.010, -0.010)),
        _translated_box((0.300, 0.006, 0.006), (0.000, -0.010, -0.016)),
        _translated_box((0.300, 0.006, 0.006), (0.000, -0.104, -0.016)),
        _translated_box((0.006, 0.094, 0.006), (-0.147, -0.057, -0.016)),
        _translated_box((0.006, 0.094, 0.006), (0.147, -0.057, -0.016)),
        _translated_box((0.260, 0.006, 0.006), (0.000, -0.040, -0.016)),
        _translated_box((0.260, 0.006, 0.006), (0.000, -0.068, -0.016)),
        _translated_box((0.260, 0.006, 0.006), (0.000, -0.086, -0.016)),
    ]
    return _union_shapes(shapes)


def _spray_arm_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.016)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.005))
    )
    arm_long = (
        cq.Workplane("XY")
        .box(0.150, 0.018, 0.006)
        .translate((0.026, 0.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 12.0)
    )
    arm_short = (
        cq.Workplane("XY")
        .box(0.120, 0.018, 0.006)
        .translate((-0.020, 0.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -18.0)
    )
    tip_long = _translated_box((0.020, 0.020, 0.008), (0.091, 0.016, 0.0))
    tip_short = _translated_box((0.018, 0.018, 0.008), (-0.075, -0.024, 0.0))
    return _union_shapes([hub, arm_long, arm_short, tip_long, tip_short])


def _button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.038, 0.012, 0.028)
        .edges("|Y")
        .fillet(0.004)
        .edges("<Y")
        .fillet(0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_dishwasher")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    pod_dark = model.material("pod_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    liner_grey = model.material("liner_grey", rgba=(0.79, 0.81, 0.83, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    rack_white = model.material("rack_white", rgba=(0.91, 0.92, 0.93, 1.0))
    spray_dark = model.material("spray_dark", rgba=(0.29, 0.31, 0.34, 1.0))
    button_light = model.material("button_light", rgba=(0.88, 0.89, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "dishwasher_body_shell"),
        material=body_white,
        name="cabinet_shell",
    )
    body.visual(
        mesh_from_cadquery(_control_pod_shape(), "dishwasher_control_pod"),
        material=pod_dark,
        name="control_pod",
    )
    for side_name, side_sign, plate_x, strip_x in (
        ("left", -1.0, CHAMBER_X - 0.167, CHAMBER_X - 0.142),
        ("right", 1.0, CHAMBER_X + 0.167, CHAMBER_X + 0.142),
    ):
        body.visual(
            Box((0.050, 0.378, 0.004)),
            origin=Origin(xyz=(plate_x, 0.002, 0.175)),
            material=liner_grey,
            name=f"{side_name}_runner_lower",
        )
        body.visual(
            Box((0.050, 0.378, 0.004)),
            origin=Origin(xyz=(plate_x, 0.002, 0.185)),
            material=liner_grey,
            name=f"{side_name}_runner_upper",
        )
        body.visual(
            Box((0.004, 0.378, 0.018)),
            origin=Origin(xyz=(strip_x, 0.002, 0.180)),
            material=liner_grey,
            name=f"{side_name}_runner_lip",
        )
        body.visual(
            Box((0.020, 0.050, 0.028)),
            origin=Origin(
                xyz=(CHAMBER_X + side_sign * (CHAMBER_W / 2.0 - 0.010), 0.118, 0.307)
            ),
            material=liner_grey,
            name=f"{side_name}_shelf_bracket",
        )
    body.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(CHAMBER_X, 0.025, 0.062)),
        material=liner_grey,
        name="spray_pedestal",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(CHAMBER_X, 0.025, 0.077)),
        material=trim_silver,
        name="spray_hub",
    )
    for index, z in enumerate((0.152, 0.205)):
        body.visual(
            Box((0.022, 0.024, 0.018)),
            origin=Origin(xyz=(POD_X, -CABINET_D / 2.0 - POD_D + 0.019, z)),
            material=pod_dark,
            name=f"button_stop_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "dishwasher_door"),
        material=body_white,
        name="door_shell",
    )
    door.visual(
        Box((0.220, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, DOOR_H - 0.044)),
        material=trim_silver,
        name="door_handle",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_X, -CABINET_D / 2.0 - DOOR_T, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.1,
            lower=0.0,
            upper=1.35,
        ),
    )

    rack = model.part("rack")
    rack_specs = [
        ((0.014, 0.402, 0.006), (-RACK_RUNNER_X, 0.201, 0.000)),
        ((0.014, 0.402, 0.006), (RACK_RUNNER_X, 0.201, 0.000)),
        ((0.014, 0.026, 0.064), (-RACK_RUNNER_X, 0.048, -0.022)),
        ((0.014, 0.026, 0.064), (RACK_RUNNER_X, 0.048, -0.022)),
        ((0.014, 0.026, 0.064), (-RACK_RUNNER_X, 0.314, -0.022)),
        ((0.014, 0.026, 0.064), (RACK_RUNNER_X, 0.314, -0.022)),
        ((0.296, 0.008, 0.008), (0.000, 0.042, -0.050)),
        ((0.296, 0.008, 0.008), (0.000, 0.330, -0.050)),
        ((0.008, 0.292, 0.008), (-0.146, 0.186, -0.050)),
        ((0.008, 0.292, 0.008), (0.146, 0.186, -0.050)),
        ((0.296, 0.008, 0.008), (0.000, 0.042, 0.044)),
        ((0.296, 0.008, 0.008), (0.000, 0.330, 0.044)),
        ((0.008, 0.292, 0.008), (-0.146, 0.186, 0.044)),
        ((0.008, 0.292, 0.008), (0.146, 0.186, 0.044)),
        ((0.008, 0.008, 0.098), (-0.146, 0.042, -0.003)),
        ((0.008, 0.008, 0.098), (0.146, 0.042, -0.003)),
        ((0.008, 0.008, 0.098), (-0.146, 0.330, -0.003)),
        ((0.008, 0.008, 0.098), (0.146, 0.330, -0.003)),
        ((0.008, 0.296, 0.008), (-0.075, 0.186, -0.050)),
        ((0.008, 0.296, 0.008), (0.000, 0.186, -0.050)),
        ((0.008, 0.296, 0.008), (0.075, 0.186, -0.050)),
    ]
    for index, (size, xyz) in enumerate(rack_specs):
        rack.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=rack_white,
            name=f"rack_piece_{index}",
        )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(CHAMBER_X, RACK_ORIGIN_Y, RACK_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=0.220,
        ),
    )

    cup_shelf = model.part("cup_shelf")
    shelf_specs = [
        ((0.024, 0.010, 0.010), (-0.159, 0.000, 0.000)),
        ((0.024, 0.010, 0.010), (0.159, 0.000, 0.000)),
        ((0.012, 0.018, 0.020), (-0.150, -0.010, -0.010)),
        ((0.012, 0.018, 0.020), (0.150, -0.010, -0.010)),
        ((0.304, 0.008, 0.008), (0.000, -0.010, -0.016)),
        ((0.304, 0.008, 0.008), (0.000, -0.104, -0.016)),
        ((0.008, 0.098, 0.008), (-0.147, -0.057, -0.016)),
        ((0.008, 0.098, 0.008), (0.147, -0.057, -0.016)),
        ((0.294, 0.008, 0.008), (0.000, -0.040, -0.016)),
        ((0.294, 0.008, 0.008), (0.000, -0.068, -0.016)),
        ((0.294, 0.008, 0.008), (0.000, -0.086, -0.016)),
    ]
    for index, (size, xyz) in enumerate(shelf_specs):
        cup_shelf.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=rack_white,
            name=f"shelf_piece_{index}",
        )
    model.articulation(
        "cup_shelf_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cup_shelf,
        origin=Origin(xyz=(CHAMBER_X, 0.118, 0.307)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.4,
            lower=0.0,
            upper=1.20,
        ),
    )

    spray_arm = model.part("spray_arm")
    spray_arm.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(),
        material=spray_dark,
        name="hub",
    )
    spray_arm.visual(
        Box((0.152, 0.018, 0.006)),
        origin=Origin(xyz=(0.026, 0.000, 0.000), rpy=(0.0, 0.0, math.radians(12))),
        material=spray_dark,
        name="arm_long",
    )
    spray_arm.visual(
        Box((0.122, 0.018, 0.006)),
        origin=Origin(xyz=(-0.020, 0.000, 0.000), rpy=(0.0, 0.0, math.radians(-18))),
        material=spray_dark,
        name="arm_short",
    )
    spray_arm.visual(
        Box((0.020, 0.020, 0.008)),
        origin=Origin(xyz=(0.091, 0.016, 0.000)),
        material=spray_dark,
        name="tip_long",
    )
    model.articulation(
        "spray_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spray_arm,
        origin=Origin(xyz=(CHAMBER_X, 0.025, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=12.0,
        ),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.018,
                body_style="skirted",
                top_diameter=0.044,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "dishwasher_timer_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=button_light,
        name="dial_cap",
    )
    model.articulation(
        "timer_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(POD_X, -CABINET_D / 2.0 - POD_D - 0.008, 0.296)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    for index, z in enumerate((0.152, 0.205)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.038, 0.012, 0.028)),
            material=button_light,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.008, 0.016)),
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
            material=button_light,
            name="button_stem",
        )
        model.articulation(
            f"mode_button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(POD_X, -CABINET_D / 2.0 - POD_D - 0.005, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    cup_shelf = object_model.get_part("cup_shelf")
    spray_arm = object_model.get_part("spray_arm")
    button_0 = object_model.get_part("mode_button_0")
    door_hinge = object_model.get_articulation("door_hinge")
    rack_slide = object_model.get_articulation("rack_slide")
    cup_shelf_fold = object_model.get_articulation("cup_shelf_fold")
    button_0_press = object_model.get_articulation("mode_button_0_press")

    body_aabb = ctx.part_world_aabb(body)
    door_closed_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door covers the front opening width",
        body_aabb is not None
        and door_closed_aabb is not None
        and door_closed_aabb[0][0] < CHAMBER_X - CHAMBER_W / 2.0 + 0.01
        and door_closed_aabb[1][0] > CHAMBER_X + CHAMBER_W / 2.0 - 0.01,
        details=f"body={body_aabb}, door={door_closed_aabb}",
    )

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops downward when opened",
            door_closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] < door_closed_aabb[1][2] - 0.12
            and open_aabb[0][1] < body_aabb[0][1] - 0.05,
            details=f"closed={door_closed_aabb}, open={open_aabb}, body={body_aabb}",
        )

    rack_rest_aabb = ctx.part_world_aabb(rack)
    spray_aabb = ctx.part_world_aabb(spray_arm)
    ctx.check(
        "rack sits inside the chamber width at rest",
        rack_rest_aabb is not None
        and rack_rest_aabb[0][0] > CHAMBER_X - CHAMBER_W / 2.0 + 0.02
        and rack_rest_aabb[1][0] < CHAMBER_X + CHAMBER_W / 2.0 - 0.02,
        details=f"rack={rack_rest_aabb}",
    )
    ctx.check(
        "spray arm clears below the rack",
        rack_rest_aabb is not None
        and spray_aabb is not None
        and spray_aabb[1][2] < rack_rest_aabb[0][2] - 0.02,
        details=f"spray={spray_aabb}, rack={rack_rest_aabb}",
    )

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper, rack_slide: rack_slide.motion_limits.upper}):
        rack_open_aabb = ctx.part_world_aabb(rack)
        ctx.check(
            "rack extends outward on the cabinet depth axis",
            body_aabb is not None
            and rack_open_aabb is not None
            and rack_open_aabb[0][1] < body_aabb[0][1] - 0.14
            and rack_open_aabb[1][1] < body_aabb[1][1] - 0.02,
            details=f"body={body_aabb}, rack_open={rack_open_aabb}",
        )

    shelf_rest_aabb = ctx.part_world_aabb(cup_shelf)
    with ctx.pose({cup_shelf_fold: cup_shelf_fold.motion_limits.upper}):
        shelf_folded_aabb = ctx.part_world_aabb(cup_shelf)
        ctx.check(
            "cup shelf folds upward",
            shelf_rest_aabb is not None
            and shelf_folded_aabb is not None
            and shelf_folded_aabb[1][2] > shelf_rest_aabb[1][2] + 0.06,
            details=f"rest={shelf_rest_aabb}, folded={shelf_folded_aabb}",
        )

    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_press: button_0_press.motion_limits.upper}):
        button_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "mode button presses inward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] > button_rest[1] + 0.003,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
