from __future__ import annotations

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
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, True)).translate(center)


def build_body_housing_shape() -> cq.Workplane:
    body = _box((0.115, 0.006, 0.004), (0.0, 0.017, 0.002))
    body = body.union(_box((0.115, 0.006, 0.004), (0.0, -0.017, 0.002)))
    body = body.union(_box((0.028, 0.040, 0.004), (-0.043, 0.0, 0.002)))
    body = body.union(_box((0.022, 0.040, 0.004), (0.046, 0.0, 0.002)))

    body = body.union(_box((0.074, 0.034, 0.014), (0.000, 0.0, 0.011)))
    body = body.union(_box((0.028, 0.036, 0.020), (-0.033, 0.0, 0.021)))
    body = body.union(
        _box((0.058, 0.034, 0.009), (0.008, 0.0, 0.027)).rotate(
            (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -11.0
        )
    )
    body = body.union(_box((0.020, 0.036, 0.022), (0.041, 0.0, 0.024)))
    body = body.union(_box((0.010, 0.032, 0.016), (0.050, 0.0, 0.024)))

    body = body.cut(_box((0.032, 0.026, 0.018), (0.026, 0.0, 0.011)))
    body = body.cut(_box((0.012, 0.020, 0.010), (0.050, 0.0, 0.008)))

    return body


def build_body_handle_lugs_shape() -> cq.Workplane:
    lugs = _box((0.008, 0.010, 0.024), (-0.033, 0.013, 0.041))
    lugs = lugs.union(_box((0.008, 0.010, 0.024), (-0.033, -0.013, 0.041)))
    lugs = lugs.union(_box((0.010, 0.008, 0.010), (-0.033, 0.013, 0.053)))
    lugs = lugs.union(_box((0.010, 0.008, 0.010), (-0.033, -0.013, 0.053)))
    return lugs


def build_body_door_lugs_shape() -> cq.Workplane:
    lugs = _box((0.004, 0.006, 0.006), (0.012, 0.010, 0.0065))
    lugs = lugs.union(_box((0.004, 0.006, 0.006), (0.012, -0.010, 0.0065)))
    lugs = lugs.union(_box((0.006, 0.005, 0.007), (0.012, 0.010, 0.0065)))
    lugs = lugs.union(_box((0.006, 0.005, 0.007), (0.012, -0.010, 0.0065)))
    return lugs


def build_handle_shape() -> cq.Workplane:
    handle = _box((0.074, 0.030, 0.009), (0.045, 0.0, 0.0120))
    handle = handle.union(_box((0.010, 0.030, 0.009), (0.082, 0.0, 0.0120)))
    handle = handle.union(_box((0.012, 0.016, 0.010), (0.006, 0.0, 0.0075)))
    handle = handle.union(_box((0.010, 0.018, 0.004), (0.060, 0.0, 0.0085)))
    return handle


def build_handle_barrel_shape() -> cq.Workplane:
    return _box((0.010, 0.016, 0.009), (0.0, 0.0, 0.0))


def build_door_shape() -> cq.Workplane:
    door = _box((0.015, 0.024, 0.0024), (0.0115, 0.0, -0.0012))
    door = door.union(_box((0.005, 0.014, 0.003), (0.0025, 0.0, -0.0015)))
    door = door.union(_box((0.003, 0.018, 0.004), (0.0195, 0.0, -0.0032)))
    return door


def build_door_barrel_shape() -> cq.Workplane:
    return _box((0.004, 0.014, 0.006), (0.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_hole_punch")

    body_finish = model.material("body_finish", rgba=(0.23, 0.24, 0.27, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_housing_shape(), "body_housing"),
        material=body_finish,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(build_body_handle_lugs_shape(), "body_handle_lugs"),
        material=body_finish,
        name="handle_hinge_lugs",
    )
    body.visual(
        mesh_from_cadquery(build_body_door_lugs_shape(), "body_door_lugs"),
        material=body_finish,
        name="door_hinge_lugs",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(build_handle_shape(), "handle_grip"),
        material=handle_finish,
        name="grip",
    )
    handle.visual(
        mesh_from_cadquery(build_handle_barrel_shape(), "handle_barrel"),
        material=handle_finish,
        name="hinge_barrel",
    )

    waste_door = model.part("waste_door")
    waste_door.visual(
        mesh_from_cadquery(build_door_shape(), "waste_door"),
        material=body_finish,
        name="panel",
    )
    waste_door.visual(
        mesh_from_cadquery(build_door_barrel_shape(), "waste_door_barrel"),
        material=body_finish,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.033, 0.0, 0.053)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=4.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "body_to_waste_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=waste_door,
        origin=Origin(xyz=(0.012, 0.0, 0.0065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    waste_door = object_model.get_part("waste_door")
    handle_hinge = object_model.get_articulation("body_to_handle")
    door_hinge = object_model.get_articulation("body_to_waste_door")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="handle_hinge_lugs",
        elem_b="hinge_barrel",
        reason="The rear lever pivot is represented with simplified interleaved hinge knuckles centered on the same hinge axis.",
    )
    ctx.allow_overlap(
        body,
        waste_door,
        elem_a="door_hinge_lugs",
        elem_b="hinge_barrel",
        reason="The waste-bin flap uses simplified hinge knuckles that occupy the same pivot volume at the lower housing edge.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_size = tuple(hi - lo for lo, hi in zip(body_aabb[0], body_aabb[1]))
        ctx.check(
            "compact desktop scale",
            0.10 <= body_size[0] <= 0.13 and 0.035 <= body_size[1] <= 0.05 and 0.03 <= body_size[2] <= 0.06,
            details=f"body_size={body_size}",
        )
    else:
        ctx.fail("compact desktop scale", "Body AABB was not available.")

    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.020,
        name="handle stays over the punch body",
    )
    ctx.expect_overlap(
        waste_door,
        body,
        axes="xy",
        min_overlap=0.012,
        name="waste door sits under the housing",
    )

    rest_handle = ctx.part_element_world_aabb(handle, elem="grip")
    rest_door = ctx.part_element_world_aabb(waste_door, elem="panel")
    ctx.check(
        "waste door closes above the desk plane",
        rest_door is not None and rest_door[0][2] > 0.001,
        details=f"door_aabb={rest_door}",
    )

    with ctx.pose({handle_hinge: 0.90}):
        open_handle = ctx.part_element_world_aabb(handle, elem="grip")
    ctx.check(
        "handle opens upward from the rear hinge",
        rest_handle is not None
        and open_handle is not None
        and open_handle[1][2] > rest_handle[1][2] + 0.020,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    with ctx.pose({door_hinge: 0.95}):
        open_door = ctx.part_element_world_aabb(waste_door, elem="panel")
    ctx.check(
        "waste door swings downward",
        rest_door is not None
        and open_door is not None
        and open_door[0][2] < rest_door[0][2] - 0.010,
        details=f"rest_door={rest_door}, open_door={open_door}",
    )

    return ctx.report()


object_model = build_object_model()
