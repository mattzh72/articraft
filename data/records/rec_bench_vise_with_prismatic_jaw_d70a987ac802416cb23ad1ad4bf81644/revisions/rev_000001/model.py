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

JAW_WIDTH = 0.56
JAW_HEIGHT = 0.18
REAR_PLATE_THICKNESS = 0.040
FRONT_PLATE_THICKNESS = 0.045

GUIDE_WAY_RADIUS = 0.014
GUIDE_WAY_BORE_RADIUS = 0.0175
GUIDE_WAY_Z = 0.055
GUIDE_WAY_LENGTH = 0.40
GUIDE_WAY_START_X = 0.015

LEAD_SCREW_RADIUS = 0.0155
LEAD_SCREW_BORE_RADIUS = 0.0205
NUT_BORE_RADIUS = 0.0185
LEAD_SCREW_LENGTH = 0.345

FRONT_JAW_CLOSED_X = 0.050
FRONT_JAW_TRAVEL = 0.185


def _jaw_plate_shape(thickness: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(thickness, JAW_WIDTH, JAW_HEIGHT).translate((thickness / 2.0, 0.0, 0.0))
    screw_bore = cq.Workplane("YZ").circle(LEAD_SCREW_BORE_RADIUS + 0.005).extrude(thickness + 0.010).translate((-0.005, 0.0, 0.0))
    upper_bore = (
        cq.Workplane("YZ")
        .center(0.0, GUIDE_WAY_Z)
        .circle(GUIDE_WAY_BORE_RADIUS)
        .extrude(thickness + 0.010)
        .translate((-0.005, 0.0, 0.0))
    )
    lower_bore = (
        cq.Workplane("YZ")
        .center(0.0, -GUIDE_WAY_Z)
        .circle(GUIDE_WAY_BORE_RADIUS)
        .extrude(thickness + 0.010)
        .translate((-0.005, 0.0, 0.0))
    )
    return plate.cut(screw_bore).cut(upper_bore).cut(lower_bore)


def _rear_casting_shape() -> cq.Workplane:
    face_block = cq.Workplane("XY").box(0.040, 0.250, 0.150).translate((0.020, 0.0, 0.0))
    back_block = cq.Workplane("XY").box(0.100, 0.160, 0.120).translate((-0.010, 0.0, 0.0))
    lower_mount = cq.Workplane("XY").box(0.060, 0.110, 0.045).translate((0.000, 0.0, -0.070))
    casting = face_block.union(back_block).union(lower_mount)
    screw_bore = cq.Workplane("YZ").circle(0.024).extrude(0.170).translate((-0.065, 0.0, 0.0))
    return casting.cut(screw_bore)


def _nut_housing_shape() -> cq.Workplane:
    sleeve = cq.Workplane("YZ").circle(0.036).extrude(0.060).translate((-0.010, 0.0, 0.0))
    front_flange = cq.Workplane("YZ").circle(0.046).extrude(0.015).translate((0.035, 0.0, 0.0))
    housing = sleeve.union(front_flange)
    bore = cq.Workplane("YZ").circle(NUT_BORE_RADIUS).extrude(0.090).translate((-0.020, 0.0, 0.0))
    return housing.cut(bore)


def _front_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.082, 0.160, 0.135).translate((0.041, 0.0, 0.0))
    upper_sleeve = cq.Workplane("YZ").center(0.0, GUIDE_WAY_Z).circle(0.027).extrude(0.082)
    lower_sleeve = cq.Workplane("YZ").center(0.0, -GUIDE_WAY_Z).circle(0.027).extrude(0.082)
    screw_hub = cq.Workplane("YZ").circle(0.040).extrude(0.090)
    side_boss = cq.Workplane("XZ").center(0.030, 0.122).circle(0.018).extrude(0.028).translate((0.0, 0.082, 0.0))
    side_rib = cq.Workplane("XY").box(0.032, 0.032, 0.112).translate((0.030, 0.094, 0.088))
    carriage = body.union(upper_sleeve).union(lower_sleeve).union(screw_hub).union(side_boss).union(side_rib)
    screw_bore = cq.Workplane("YZ").circle(LEAD_SCREW_BORE_RADIUS).extrude(0.096).translate((-0.006, 0.0, 0.0))
    upper_bore = (
        cq.Workplane("YZ")
        .center(0.0, GUIDE_WAY_Z)
        .circle(GUIDE_WAY_BORE_RADIUS)
        .extrude(0.096)
        .translate((-0.006, 0.0, 0.0))
    )
    lower_bore = (
        cq.Workplane("YZ")
        .center(0.0, -GUIDE_WAY_Z)
        .circle(GUIDE_WAY_BORE_RADIUS)
        .extrude(0.096)
        .translate((-0.006, 0.0, 0.0))
    )
    return carriage.cut(screw_bore).cut(upper_bore).cut(lower_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_vise")

    wood = model.material("wood", rgba=(0.66, 0.52, 0.33, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.24, 0.24, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    blackened = model.material("blackened", rgba=(0.16, 0.16, 0.17, 1.0))

    rear_jaw = model.part("rear_jaw")
    rear_jaw.visual(
        mesh_from_cadquery(_rear_casting_shape(), "rear_casting"),
        material=cast_iron,
        name="rear_casting",
    )
    rear_jaw.visual(
        mesh_from_cadquery(_nut_housing_shape(), "nut_housing"),
        material=cast_iron,
        name="nut_housing",
    )
    rear_jaw.visual(
        Cylinder(radius=GUIDE_WAY_RADIUS, length=GUIDE_WAY_LENGTH),
        origin=Origin(
            xyz=(GUIDE_WAY_START_X + GUIDE_WAY_LENGTH / 2.0, 0.0, GUIDE_WAY_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="upper_way",
    )
    rear_jaw.visual(
        Cylinder(radius=GUIDE_WAY_RADIUS, length=GUIDE_WAY_LENGTH),
        origin=Origin(
            xyz=(GUIDE_WAY_START_X + GUIDE_WAY_LENGTH / 2.0, 0.0, -GUIDE_WAY_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lower_way",
    )
    rear_jaw.visual(
        mesh_from_cadquery(_jaw_plate_shape(REAR_PLATE_THICKNESS), "rear_plate"),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=wood,
        name="rear_plate",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(_front_carriage_shape(), "front_carriage"),
        material=cast_iron,
        name="carriage",
    )
    front_jaw.visual(
        mesh_from_cadquery(_jaw_plate_shape(FRONT_PLATE_THICKNESS), "front_plate"),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=wood,
        name="front_plate",
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=LEAD_SCREW_RADIUS, length=LEAD_SCREW_LENGTH),
        origin=Origin(xyz=(0.036 - LEAD_SCREW_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lead_screw",
    )
    screw_handle.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="thrust_collar",
    )
    screw_handle.visual(
        Cylinder(radius=0.007, length=0.320),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    screw_handle.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.058, 0.160, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="grip_0",
    )
    screw_handle.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.058, -0.160, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="grip_1",
    )

    guide_lock = model.part("guide_lock")
    guide_lock.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_shaft",
    )
    guide_lock.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened,
        name="knob_body",
    )
    guide_lock.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(0.022, 0.052, 0.0)),
        material=blackened,
        name="grip_tab",
    )

    model.articulation(
        "rear_to_front",
        ArticulationType.PRISMATIC,
        parent=rear_jaw,
        child=front_jaw,
        origin=Origin(xyz=(FRONT_JAW_CLOSED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.12,
            lower=0.0,
            upper=FRONT_JAW_TRAVEL,
        ),
    )
    model.articulation(
        "front_to_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=screw_handle,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "front_to_guide_lock",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=guide_lock,
        origin=Origin(xyz=(0.030, 0.110, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_jaw = object_model.get_part("rear_jaw")
    front_jaw = object_model.get_part("front_jaw")
    screw_handle = object_model.get_part("screw_handle")
    guide_lock = object_model.get_part("guide_lock")

    slide = object_model.get_articulation("rear_to_front")
    handle_spin = object_model.get_articulation("front_to_screw_handle")
    knob_spin = object_model.get_articulation("front_to_guide_lock")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    def _aabb_extents(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple(hi[i] - lo[i] for i in range(3))

    ctx.expect_gap(
        front_jaw,
        rear_jaw,
        axis="x",
        positive_elem="front_plate",
        negative_elem="rear_plate",
        min_gap=0.009,
        max_gap=0.025,
        name="jaw faces start nearly closed",
    )
    ctx.expect_overlap(
        front_jaw,
        rear_jaw,
        axes="yz",
        elem_a="front_plate",
        elem_b="rear_plate",
        min_overlap=0.160,
        name="jaw faces stay aligned for clamping",
    )
    ctx.expect_overlap(
        front_jaw,
        rear_jaw,
        axes="x",
        elem_a="carriage",
        elem_b="upper_way",
        min_overlap=0.060,
        name="upper guide way remains engaged at rest",
    )
    ctx.expect_overlap(
        front_jaw,
        rear_jaw,
        axes="x",
        elem_a="carriage",
        elem_b="lower_way",
        min_overlap=0.060,
        name="lower guide way remains engaged at rest",
    )
    ctx.expect_overlap(
        screw_handle,
        rear_jaw,
        axes="x",
        elem_a="lead_screw",
        elem_b="nut_housing",
        min_overlap=0.060,
        name="lead screw remains threaded into the rear nut at rest",
    )

    rest_front_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({slide: FRONT_JAW_TRAVEL}):
        open_front_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            front_jaw,
            rear_jaw,
            axis="x",
            positive_elem="front_plate",
            negative_elem="rear_plate",
            min_gap=0.190,
            max_gap=0.210,
            name="front jaw opens to a bench-vise working gap",
        )
        ctx.expect_overlap(
            front_jaw,
            rear_jaw,
            axes="x",
            elem_a="carriage",
            elem_b="upper_way",
            min_overlap=0.060,
            name="upper guide way retains insertion at full opening",
        )
        ctx.expect_overlap(
            front_jaw,
            rear_jaw,
            axes="x",
            elem_a="carriage",
            elem_b="lower_way",
            min_overlap=0.060,
            name="lower guide way retains insertion at full opening",
        )
        ctx.expect_overlap(
            screw_handle,
            rear_jaw,
            axes="x",
            elem_a="lead_screw",
            elem_b="nut_housing",
            min_overlap=0.035,
            name="lead screw remains engaged at full opening",
        )

    ctx.check(
        "front jaw slides outward",
        rest_front_pos is not None
        and open_front_pos is not None
        and open_front_pos[0] > rest_front_pos[0] + 0.15,
        details=f"rest={rest_front_pos}, open={open_front_pos}",
    )

    handle_rest = ctx.part_element_world_aabb(screw_handle, elem="handle_bar")
    with ctx.pose({handle_spin: math.pi / 2.0}):
        handle_turned = ctx.part_element_world_aabb(screw_handle, elem="handle_bar")
    handle_rest_dims = _aabb_extents(handle_rest)
    handle_turned_dims = _aabb_extents(handle_turned)
    ctx.check(
        "screw handle rotation changes the crossbar orientation",
        handle_rest_dims is not None
        and handle_turned_dims is not None
        and handle_rest_dims[1] > 0.28
        and handle_rest_dims[2] < 0.03
        and handle_turned_dims[2] > 0.28
        and handle_turned_dims[1] < 0.03,
        details=f"rest_dims={handle_rest_dims}, turned_dims={handle_turned_dims}",
    )

    knob_rest = ctx.part_element_world_aabb(guide_lock, elem="grip_tab")
    with ctx.pose({knob_spin: math.pi / 2.0}):
        knob_turned = ctx.part_element_world_aabb(guide_lock, elem="grip_tab")
    knob_rest_center = _aabb_center(knob_rest)
    knob_turned_center = _aabb_center(knob_turned)
    ctx.check(
        "guide lock knob visibly turns on its side shaft",
        knob_rest_center is not None
        and knob_turned_center is not None
        and abs(knob_rest_center[0] - knob_turned_center[0]) > 0.015
        and abs(knob_rest_center[2] - knob_turned_center[2]) > 0.015,
        details=f"rest_center={knob_rest_center}, turned_center={knob_turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
