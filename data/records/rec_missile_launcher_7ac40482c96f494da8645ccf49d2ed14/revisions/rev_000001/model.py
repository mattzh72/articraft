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
    LatheGeometry,
    mesh_from_geometry,
)


def _hollow_launch_tube(length: float, outer_radius: float, inner_radius: float):
    rear = -0.30
    front = rear + length
    lip = 0.030
    profile = [
        (outer_radius * 1.08, rear),
        (outer_radius, rear + lip),
        (outer_radius, front - lip),
        (outer_radius * 1.10, front),
        (inner_radius, front),
        (inner_radius, rear),
    ]
    tube = LatheGeometry(profile, segments=72, closed=True)
    tube.rotate_y(math.pi / 2.0)
    return tube


def _wheel_rim_geometry():
    rim = TorusGeometry(0.066, 0.0065, radial_segments=16, tubular_segments=64)
    rim.rotate_x(math.pi / 2.0)
    return rim


def _muzzle_ring_geometry():
    ring = TorusGeometry(0.051, 0.006, radial_segments=14, tubular_segments=56)
    ring.rotate_y(math.pi / 2.0)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_twin_tube_launcher")

    olive = model.material("olive_drab", rgba=(0.28, 0.33, 0.18, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.16, 0.20, 0.12, 1.0))
    parkerized = model.material("parkerized_steel", rgba=(0.08, 0.085, 0.075, 1.0))
    worn_edge = model.material("worn_muzzle_metal", rgba=(0.38, 0.39, 0.34, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.56, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_olive,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.150, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=dark_olive,
        name="top_bearing_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=parkerized,
        name="bearing_race",
    )
    for ix, x in enumerate((-0.175, 0.175)):
        for iy, y in enumerate((-0.135, 0.135)):
            pedestal.visual(
                Box((0.040, 0.040, 0.320)),
                origin=Origin(xyz=(x, y, 0.195)),
                material=olive,
                name=f"upright_{ix}_{iy}",
            )
    for y in (-0.135, 0.135):
        pedestal.visual(
            Box((0.390, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.080)),
            material=olive,
            name=f"lower_rail_{'pos' if y > 0 else 'neg'}",
        )
        pedestal.visual(
            Box((0.390, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.335)),
            material=olive,
            name=f"upper_rail_{'pos' if y > 0 else 'neg'}",
        )
    for x in (-0.175, 0.175):
        pedestal.visual(
            Box((0.035, 0.310, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.335)),
            material=olive,
            name=f"side_rail_{'pos' if x > 0 else 'neg'}",
        )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.142, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=parkerized,
        name="turntable_disc",
    )
    yaw_head.visual(
        Cylinder(radius=0.112, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=olive,
        name="upper_disc",
    )
    yaw_head.visual(
        Box((0.250, 0.420, 0.070)),
        origin=Origin(xyz=(-0.035, 0.0, 0.100)),
        material=olive,
        name="yoke_base",
    )
    yaw_head.visual(
        Box((0.160, 0.062, 0.310)),
        origin=Origin(xyz=(-0.045, -0.176, 0.270)),
        material=olive,
        name="side_cheek_neg",
    )
    yaw_head.visual(
        Box((0.160, 0.062, 0.310)),
        origin=Origin(xyz=(-0.045, 0.176, 0.270)),
        material=olive,
        name="side_cheek_pos",
    )
    yaw_head.visual(
        Box((0.050, 0.420, 0.090)),
        origin=Origin(xyz=(-0.135, 0.0, 0.180)),
        material=olive,
        name="rear_bridge",
    )
    yaw_head.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(-0.125, -0.212, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="handwheel_boss",
    )

    tube_pair = model.part("tube_pair")
    for idx, y in enumerate((-0.075, 0.075)):
        tube_geometry = _hollow_launch_tube(0.900, 0.055, 0.042)
        tube_pair.visual(
            mesh_from_geometry(tube_geometry, f"launch_tube_{idx}"),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=olive,
            name=f"tube_{idx}",
        )
        tube_pair.visual(
            mesh_from_geometry(_muzzle_ring_geometry(), f"muzzle_ring_{idx}"),
            origin=Origin(xyz=(0.605, y, 0.0)),
            material=worn_edge,
            name=f"muzzle_lip_{idx}",
        )
    tube_pair.visual(
        Box((0.220, 0.155, 0.026)),
        origin=Origin(xyz=(-0.070, 0.0, 0.058)),
        material=dark_olive,
        name="upper_clamp",
    )
    tube_pair.visual(
        Box((0.220, 0.155, 0.026)),
        origin=Origin(xyz=(-0.070, 0.0, -0.058)),
        material=dark_olive,
        name="lower_clamp",
    )
    tube_pair.visual(
        Box((0.070, 0.230, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_olive,
        name="trunnion_block",
    )
    tube_pair.visual(
        Cylinder(radius=0.024, length=0.475),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="trunnion_pin",
    )
    tube_pair.visual(
        Box((0.150, 0.230, 0.018)),
        origin=Origin(xyz=(-0.245, 0.0, -0.062)),
        material=parkerized,
        name="rear_tie_bar",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(_wheel_rim_geometry(), "handwheel_rim"),
        origin=Origin(),
        material=parkerized,
        name="rim",
    )
    handwheel.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="hub",
    )
    handwheel.visual(
        Cylinder(radius=0.011, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="axle_stub",
    )
    handwheel.visual(
        Box((0.122, 0.008, 0.008)),
        origin=Origin(),
        material=parkerized,
        name="spoke_horizontal",
    )
    handwheel.visual(
        Box((0.008, 0.008, 0.122)),
        origin=Origin(),
        material=parkerized,
        name="spoke_vertical",
    )
    handwheel.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.047, -0.010, 0.047), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="finger_grip",
    )

    model.articulation(
        "pedestal_to_yaw_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yaw_head_to_tube_pair",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=tube_pair,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=0.9, lower=-0.15, upper=0.85),
    )
    model.articulation(
        "yaw_head_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=yaw_head,
        child=handwheel,
        origin=Origin(xyz=(-0.125, -0.266, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_head = object_model.get_part("yaw_head")
    tube_pair = object_model.get_part("tube_pair")
    handwheel = object_model.get_part("handwheel")
    yaw = object_model.get_articulation("pedestal_to_yaw_head")
    elevation = object_model.get_articulation("yaw_head_to_tube_pair")
    wheel_spin = object_model.get_articulation("yaw_head_to_handwheel")

    ctx.allow_overlap(
        yaw_head,
        tube_pair,
        elem_a="side_cheek_neg",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally clipped through the side cheek to keep the elevating tube pair visibly captured.",
    )
    ctx.allow_overlap(
        yaw_head,
        tube_pair,
        elem_a="side_cheek_pos",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally clipped through the side cheek to keep the elevating tube pair visibly captured.",
    )
    ctx.allow_overlap(
        yaw_head,
        handwheel,
        elem_a="handwheel_boss",
        elem_b="axle_stub",
        reason="The handwheel axle is intentionally seated into the boss on the yoke side plate.",
    )

    ctx.expect_contact(
        pedestal,
        yaw_head,
        elem_a="bearing_race",
        elem_b="turntable_disc",
        contact_tol=0.002,
        name="yaw bearing sits on pedestal",
    )
    ctx.expect_overlap(
        yaw_head,
        tube_pair,
        axes="yz",
        elem_a="side_cheek_neg",
        elem_b="trunnion_pin",
        min_overlap=0.020,
        name="negative cheek captures trunnion pin",
    )
    ctx.expect_overlap(
        yaw_head,
        tube_pair,
        axes="yz",
        elem_a="side_cheek_pos",
        elem_b="trunnion_pin",
        min_overlap=0.020,
        name="positive cheek captures trunnion pin",
    )
    ctx.expect_overlap(
        yaw_head,
        handwheel,
        axes="yz",
        elem_a="handwheel_boss",
        elem_b="axle_stub",
        min_overlap=0.015,
        name="handwheel axle remains in boss",
    )
    ctx.expect_gap(
        yaw_head,
        handwheel,
        axis="y",
        positive_elem="side_cheek_neg",
        negative_elem="rim",
        min_gap=0.010,
        name="handwheel rim clears yoke cheek",
    )

    rest_aabb = ctx.part_world_aabb(tube_pair)
    with ctx.pose({elevation: 0.65}):
        raised_aabb = ctx.part_world_aabb(tube_pair)
        ctx.expect_overlap(
            yaw_head,
            tube_pair,
            axes="yz",
            elem_a="side_cheek_neg",
            elem_b="trunnion_pin",
            min_overlap=0.012,
            name="raised tube pair remains clipped in trunnion",
        )
    ctx.check(
        "elevation raises launcher front",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.15,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    rest_yaw = ctx.part_world_position(yaw_head)
    with ctx.pose({yaw: 0.75, wheel_spin: 1.2}):
        turned_yaw = ctx.part_world_position(yaw_head)
        ctx.expect_overlap(
            yaw_head,
            handwheel,
            axes="yz",
            elem_a="handwheel_boss",
            elem_b="axle_stub",
            min_overlap=0.015,
            name="spinning handwheel remains mounted",
        )
    ctx.check(
        "yaw head stays on vertical bearing while rotating",
        rest_yaw is not None and turned_yaw is not None and abs(rest_yaw[2] - turned_yaw[2]) < 0.001,
        details=f"rest={rest_yaw}, turned={turned_yaw}",
    )

    return ctx.report()


object_model = build_object_model()
