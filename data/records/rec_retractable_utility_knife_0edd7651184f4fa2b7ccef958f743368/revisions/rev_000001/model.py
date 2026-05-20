from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _xz_prism(points: list[tuple[float, float]], thickness: float) -> MeshGeometry:
    """Build a thin prism from an X/Z side profile, extruded along local Y."""
    half = thickness / 2.0
    geom = MeshGeometry()
    front = [geom.add_vertex(x, half, z) for x, z in points]
    back = [geom.add_vertex(x, -half, z) for x, z in points]
    n = len(points)

    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

    yellow = model.material("safety_yellow", rgba=(1.0, 0.70, 0.06, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.74, 1.0))
    shadow = model.material("etched_shadow", rgba=(0.08, 0.08, 0.075, 1.0))

    handle = model.part("handle")
    handle.visual(
        Box((0.160, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark,
        name="channel_floor",
    )
    handle.visual(
        Box((0.152, 0.006, 0.018)),
        origin=Origin(xyz=(0.004, -0.012, 0.009)),
        material=yellow,
        name="side_rail_0",
    )
    handle.visual(
        Box((0.152, 0.006, 0.018)),
        origin=Origin(xyz=(0.004, 0.012, 0.009)),
        material=yellow,
        name="side_rail_1",
    )
    handle.visual(
        Box((0.146, 0.011, 0.003)),
        origin=Origin(xyz=(0.003, -0.0095, 0.018)),
        material=yellow,
        name="top_lip_0",
    )
    handle.visual(
        Box((0.146, 0.011, 0.003)),
        origin=Origin(xyz=(0.003, 0.0095, 0.018)),
        material=yellow,
        name="top_lip_1",
    )
    handle.visual(
        Box((0.030, 0.002, 0.014)),
        origin=Origin(xyz=(0.000, -0.0158, 0.010)),
        material=rubber,
        name="grip_strip_0",
    )
    handle.visual(
        Box((0.030, 0.002, 0.014)),
        origin=Origin(xyz=(0.000, 0.0158, 0.010)),
        material=rubber,
        name="grip_strip_1",
    )
    handle.visual(
        Box((0.014, 0.007, 0.007)),
        origin=Origin(xyz=(-0.077, -0.016, 0.0165)),
        material=yellow,
        name="hinge_lug_0",
    )
    handle.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(xyz=(-0.080, -0.016, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel_0",
    )
    handle.visual(
        Box((0.014, 0.007, 0.007)),
        origin=Origin(xyz=(-0.077, 0.016, 0.0165)),
        material=yellow,
        name="hinge_lug_1",
    )
    handle.visual(
        Cylinder(radius=0.0025, length=0.006),
        origin=Origin(xyz=(-0.080, 0.016, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel_1",
    )

    carrier = model.part("carrier")
    carrier.visual(
        Box((0.112, 0.010, 0.006)),
        origin=Origin(xyz=(-0.004, 0.0, 0.007)),
        material=dark,
        name="carrier_rail",
    )
    blade_profile = [
        (0.020, 0.006),
        (0.116, 0.006),
        (0.129, 0.010),
        (0.116, 0.014),
        (0.020, 0.014),
    ]
    carrier.visual(
        mesh_from_geometry(_xz_prism(blade_profile, 0.004), "segmented_blade"),
        material=steel,
        name="segmented_blade",
    )
    for i, x in enumerate((0.041, 0.055, 0.069, 0.083, 0.097, 0.111)):
        carrier.visual(
            Box((0.014, 0.0007, 0.00055)),
            origin=Origin(
                xyz=(x, -0.00235, 0.010),
                rpy=(0.0, 0.72, 0.0),
            ),
            material=shadow,
            name=f"score_line_{i}",
        )

    top_slider = model.part("top_slider")
    top_slider.visual(
        Box((0.030, 0.006, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, 0.015)),
        material=dark,
        name="slider_stem",
    )
    top_slider.visual(
        Box((0.036, 0.020, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, 0.023)),
        material=rubber,
        name="thumb_pad",
    )
    for i, x in enumerate((-0.024, -0.018, -0.012, -0.006, 0.000)):
        top_slider.visual(
            Box((0.002, 0.018, 0.0012)),
            origin=Origin(xyz=(x, 0.0, 0.0266)),
            material=shadow,
            name=f"thumb_rib_{i}",
        )

    rear_cap = model.part("rear_cap")
    rear_cap.visual(
        Cylinder(radius=0.00235, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="cap_barrel",
    )
    rear_cap.visual(
        Box((0.010, 0.022, 0.016)),
        origin=Origin(xyz=(-0.005, 0.0, -0.008)),
        material=yellow,
        name="breaker_plate",
    )
    rear_cap.visual(
        Box((0.001, 0.015, 0.004)),
        origin=Origin(xyz=(-0.01045, 0.0, -0.008)),
        material=dark,
        name="blade_break_slot",
    )
    rear_cap.visual(
        Box((0.006, 0.010, 0.003)),
        origin=Origin(xyz=(-0.004, 0.0, -0.0015)),
        material=yellow,
        name="hinge_leaf",
    )

    model.articulation(
        "carrier_slide",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.035),
    )
    model.articulation(
        "slider_mount",
        ArticulationType.FIXED,
        parent=carrier,
        child=top_slider,
        origin=Origin(),
    )
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=rear_cap,
        origin=Origin(xyz=(-0.080, 0.0, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("carrier")
    top_slider = object_model.get_part("top_slider")
    rear_cap = object_model.get_part("rear_cap")
    slide = object_model.get_articulation("carrier_slide")
    hinge = object_model.get_articulation("cap_hinge")

    ctx.expect_gap(
        carrier,
        handle,
        axis="z",
        positive_elem="carrier_rail",
        negative_elem="channel_floor",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carrier rail rides on the guide floor",
    )
    ctx.expect_within(
        carrier,
        handle,
        axes="y",
        inner_elem="carrier_rail",
        margin=0.0005,
        name="carrier stays between the side rails",
    )
    ctx.expect_contact(
        rear_cap,
        handle,
        elem_a="cap_barrel",
        elem_b="hinge_barrel_1",
        contact_tol=0.0008,
        name="rear cap barrel is clipped to hinge knuckle",
    )

    rest_carrier = ctx.part_world_position(carrier)
    rest_slider = ctx.part_world_position(top_slider)
    with ctx.pose({slide: 0.035}):
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="channel_floor",
            min_overlap=0.080,
            name="extended carrier remains retained in the handle channel",
        )
        extended_carrier = ctx.part_world_position(carrier)
        extended_slider = ctx.part_world_position(top_slider)

    ctx.check(
        "carrier slide extends along the handle axis",
        rest_carrier is not None
        and extended_carrier is not None
        and extended_carrier[0] > rest_carrier[0] + 0.030,
        details=f"rest={rest_carrier}, extended={extended_carrier}",
    )
    ctx.check(
        "top slider translates with the carrier",
        rest_slider is not None
        and extended_slider is not None
        and rest_carrier is not None
        and extended_carrier is not None
        and abs((extended_slider[0] - rest_slider[0]) - (extended_carrier[0] - rest_carrier[0])) < 0.001,
        details=f"slider rest={rest_slider}, slider extended={extended_slider}",
    )

    closed_cap = ctx.part_element_world_aabb(rear_cap, elem="breaker_plate")
    with ctx.pose({hinge: 1.20}):
        open_cap = ctx.part_element_world_aabb(rear_cap, elem="breaker_plate")
        ctx.expect_contact(
            rear_cap,
            handle,
            elem_a="cap_barrel",
            elem_b="hinge_barrel_1",
            contact_tol=0.0008,
            name="rear cap remains clipped while opened",
        )

    ctx.check(
        "rear breaker cap swings upward on the back hinge",
        closed_cap is not None
        and open_cap is not None
        and open_cap[1][2] > closed_cap[1][2] + 0.005,
        details=f"closed={closed_cap}, open={open_cap}",
    )

    return ctx.report()


object_model = build_object_model()
