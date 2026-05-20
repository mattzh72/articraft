from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _add_side_channel(part, *, side_sign: float, height: float, z0: float, material) -> None:
    web_x = side_sign * 0.595
    flange_x = side_sign * 0.540
    center_z = z0 + height * 0.5
    side_name = "0" if side_sign < 0.0 else "1"

    part.visual(
        Box((0.03, 0.22, height)),
        origin=Origin(xyz=(web_x, 0.0, center_z)),
        material=material,
        name=f"channel_web_{side_name}",
    )
    for flange_name, y_pos in (("front", 0.085), ("rear", -0.085)):
        part.visual(
            Box((0.08, 0.03, height)),
            origin=Origin(xyz=(flange_x, y_pos, center_z)),
            material=material,
            name=f"channel_{flange_name}_{side_name}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    painted_steel = model.material("painted_steel", rgba=(0.28, 0.34, 0.39, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.24, 0.29, 0.33, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    zinc = model.material("zinc", rgba=(0.69, 0.72, 0.75, 1.0))
    gasket = model.material("gasket", rgba=(0.08, 0.09, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.32, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=painted_steel,
        name="sill",
    )
    _add_side_channel(frame, side_sign=-1.0, height=2.24, z0=-0.005, material=painted_steel)
    _add_side_channel(frame, side_sign=1.0, height=2.24, z0=-0.005, material=painted_steel)
    frame.visual(
        Box((1.32, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.95)),
        material=painted_steel,
        name="beam",
    )
    frame.visual(
        Box((0.18, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.08)),
        material=cast_iron,
        name="bearing_block",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(xyz=(0.0, 0.10, 2.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="axle_boss",
    )
    frame.visual(
        Box((0.28, 0.20, 0.18)),
        origin=Origin(xyz=(0.27, 0.0, 2.15)),
        material=cast_iron,
        name="gearbox",
    )
    frame.visual(
        Box((0.09, 0.16, 0.14)),
        origin=Origin(xyz=(-0.27, 0.0, 2.04)),
        material=cast_iron,
        name="counterweight_box",
    )
    for index, z_pos in enumerate((2.11, 2.19)):
        frame.visual(
            Cylinder(radius=0.006, length=0.035),
            origin=Origin(xyz=(0.405, 0.091, z_pos)),
            material=zinc,
            name=f"cover_hinge_knuckle_{index}",
        )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.96, 0.05, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=gate_steel,
        name="panel",
    )
    gate_leaf.visual(
        Box((0.98, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
        material=gate_steel,
        name="top_cap",
    )
    gate_leaf.visual(
        Box((0.98, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=gasket,
        name="bottom_seal",
    )
    for index, x_pos in enumerate((-0.28, 0.0, 0.28)):
        gate_leaf.visual(
            Box((0.08, 0.04, 0.92)),
            origin=Origin(xyz=(x_pos, -0.045, 0.56)),
            material=gate_steel,
            name=f"stiffener_{index}",
        )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.19, tube=0.013, radial_segments=20, tubular_segments=48).rotate_x(
                math.pi / 2.0
            ),
            "handwheel_rim",
        ),
        material=cast_iron,
        name="rim",
    )
    handwheel.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="hub",
    )
    handwheel.visual(
        Box((0.36, 0.018, 0.032)),
        material=cast_iron,
        name="spoke_cross",
    )
    handwheel.visual(
        Box((0.032, 0.018, 0.36)),
        material=cast_iron,
        name="spoke_upright",
    )
    handwheel.visual(
        Box((0.36, 0.018, 0.028)),
        origin=Origin(rpy=(0.0, math.pi / 4.0, 0.0)),
        material=cast_iron,
        name="spoke_diag_0",
    )
    handwheel.visual(
        Box((0.36, 0.018, 0.028)),
        origin=Origin(rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=cast_iron,
        name="spoke_diag_1",
    )

    gearbox_cover = model.part("gearbox_cover")
    gearbox_cover.visual(
        Box((0.14, 0.008, 0.16)),
        origin=Origin(xyz=(-0.07, 0.004, 0.0)),
        material=painted_steel,
        name="cover_panel",
    )
    gearbox_cover.visual(
        Box((0.025, 0.022, 0.045)),
        origin=Origin(xyz=(-0.11, 0.018, 0.0)),
        material=zinc,
        name="cover_handle",
    )

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.20, lower=0.0, upper=0.65),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.15, 2.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=6.0),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=gearbox_cover,
        origin=Origin(xyz=(0.405, 0.10, 2.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    handwheel = object_model.get_part("handwheel")
    gearbox_cover = object_model.get_part("gearbox_cover")

    leaf_slide = object_model.get_articulation("leaf_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.check(
        "gate leaf uses vertical prismatic slide",
        tuple(round(value, 3) for value in leaf_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={leaf_slide.axis}",
    )
    ctx.check(
        "handwheel spins on a horizontal axle",
        tuple(round(value, 3) for value in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )
    ctx.check(
        "gearbox cover uses a side hinge",
        tuple(round(value, 3) for value in cover_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={cover_hinge.axis}",
    )

    ctx.expect_contact(
        gate_leaf,
        frame,
        elem_a="bottom_seal",
        elem_b="sill",
        name="closed leaf bears on the sill seal",
    )
    ctx.expect_contact(
        handwheel,
        frame,
        elem_a="hub",
        elem_b="axle_boss",
        name="handwheel hub seats on the axle boss",
    )
    ctx.expect_gap(
        gearbox_cover,
        frame,
        axis="y",
        positive_elem="cover_panel",
        negative_elem="gearbox",
        min_gap=0.0,
        max_gap=0.012,
        name="closed gearbox cover sits flush on the housing front",
    )

    leaf_rest = ctx.part_world_position(gate_leaf)
    with ctx.pose({leaf_slide: 0.65}):
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="z",
            positive_elem="beam",
            negative_elem="panel",
            min_gap=0.03,
            name="raised leaf still clears the top beam",
        )
        leaf_open = ctx.part_world_position(gate_leaf)
    ctx.check(
        "gate leaf raises upward",
        leaf_rest is not None and leaf_open is not None and leaf_open[2] > leaf_rest[2] + 0.60,
        details=f"rest={leaf_rest}, open={leaf_open}",
    )

    cover_closed_aabb = ctx.part_world_aabb(gearbox_cover)
    with ctx.pose({cover_hinge: 1.10}):
        cover_open_aabb = ctx.part_world_aabb(gearbox_cover)
    ctx.check(
        "gearbox cover swings outward",
        cover_closed_aabb is not None
        and cover_open_aabb is not None
        and cover_open_aabb[1][1] > cover_closed_aabb[1][1] + 0.07,
        details=f"closed={cover_closed_aabb}, open={cover_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
