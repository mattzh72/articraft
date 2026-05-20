from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length * 0.5, both=True).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length * 0.5, both=True).translate(center)


def _build_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").circle(0.114).extrude(0.162)
    body = body.faces(">Z").workplane().circle(0.106).extrude(0.060)
    body = body.faces(">Z").workplane().circle(0.111).extrude(0.010)
    body = body.faces(">Z").shell(-0.012)
    body = body.union(_box((0.024, 0.086, 0.076), (0.102, 0.0, 0.114)))
    body = body.union(_box((0.034, 0.118, 0.022), (-0.103, 0.0, 0.218)))
    return body


def _build_lid_shape() -> cq.Workplane:
    lid_center_x = 0.116
    shell = cq.Workplane("XY").circle(0.116).extrude(0.010)
    shell = shell.faces(">Z").workplane().circle(0.111).extrude(0.014)
    shell = shell.faces(">Z").workplane().circle(0.102).extrude(0.022)
    shell = shell.faces("<Z").shell(-0.008)
    shell = shell.translate((lid_center_x, 0.0, -0.008))
    barrel = _cylinder_x(0.009, 0.122, (0.0, 0.0, 0.002))
    front_lip = _box((0.018, 0.060, 0.016), (0.214, 0.0, -0.001))
    steam_cap = _cylinder_z(0.016, 0.008, (0.136, 0.0, 0.026))
    return shell.union(barrel).union(front_lip).union(steam_cap)


def _build_knob_shape() -> cq.Workplane:
    back_plate = _cylinder_x(0.019, 0.006, (0.003, 0.0, 0.0))
    main_dial = _cylinder_x(0.030, 0.022, (0.017, 0.0, 0.0))
    grip_ring = _cylinder_x(0.026, 0.010, (0.028, 0.0, 0.0))
    pointer = _box((0.020, 0.010, 0.014), (0.020, 0.0, 0.025))
    return back_plate.union(main_dial).union(grip_ring).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    knob_black = model.material("knob_black", rgba=(0.15, 0.15, 0.16, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.94, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shape(), "rice_cooker_body"), material=body_white, name="shell")

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid_shape(), "rice_cooker_lid"), material=lid_white, name="lid_shell")

    knob = model.part("selector_knob")
    knob.visual(mesh_from_cadquery(_build_knob_shape(), "rice_cooker_selector_knob"), material=knob_black, name="knob")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.116, 0.0, 0.238)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=0.0,
            upper=1.42,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.114, 0.0, 0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("selector_knob")
    lid_hinge = object_model.get_articulation("body_to_lid")
    knob_spin = object_model.get_articulation("body_to_selector_knob")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.012,
        max_penetration=0.004,
        name="closed lid seats onto the body top",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.16,
        name="closed lid covers the body opening",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        name="selector knob mounts flush to the front panel",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    pointer_rest = ctx.part_element_world_aabb(knob, elem="knob")
    with ctx.pose({knob_spin: 1.2}):
        pointer_turned = ctx.part_element_world_aabb(knob, elem="knob")

    def _aabb_center_yz(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return ((lower[1] + upper[1]) * 0.5, (lower[2] + upper[2]) * 0.5)

    rest_center = _aabb_center_yz(pointer_rest)
    turned_center = _aabb_center_yz(pointer_turned)
    ctx.check(
        "selector knob rotates about its front axis",
        rest_center is not None
        and turned_center is not None
        and math.hypot(turned_center[0] - rest_center[0], turned_center[1] - rest_center[1]) > 0.0008,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
