from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _contoured_seat_mesh(
    *,
    radius_x: float = 0.215,
    radius_y: float = 0.190,
    radial_segments: int = 9,
    angular_segments: int = 80,
) -> MeshGeometry:
    """Watertight shallow-dished oval bar-stool seat in the local +Z frame."""

    geom = MeshGeometry()

    def top_z(r: float, y: float) -> float:
        # A low center, lifted side bolsters, and a rounded raised perimeter lip.
        center_dish = -0.020 * (1.0 - r) ** 1.7
        rim_lift = 0.022 * max(0.0, (r - 0.68) / 0.32) ** 2
        side_bolster = 0.010 * (abs(y) / radius_y) ** 2 * (0.35 + 0.65 * r)
        return 0.058 + center_dish + rim_lift + side_bolster

    def bottom_z(r: float) -> float:
        # Slightly convex underside so the shell reads molded, not a flat puck.
        return 0.010 + 0.008 * r**1.5

    top_center = geom.add_vertex(0.0, 0.0, top_z(0.0, 0.0))
    bottom_center = geom.add_vertex(0.0, 0.0, bottom_z(0.0))
    top_rings: list[list[int]] = []
    bottom_rings: list[list[int]] = []

    for i in range(1, radial_segments + 1):
        r = i / radial_segments
        top_ring: list[int] = []
        bottom_ring: list[int] = []
        for j in range(angular_segments):
            theta = 2.0 * math.pi * j / angular_segments
            x = radius_x * r * math.cos(theta)
            y = radius_y * r * math.sin(theta)
            top_ring.append(geom.add_vertex(x, y, top_z(r, y)))
            bottom_ring.append(geom.add_vertex(x, y, bottom_z(r)))
        top_rings.append(top_ring)
        bottom_rings.append(bottom_ring)

    def nxt(j: int) -> int:
        return (j + 1) % angular_segments

    first_top = top_rings[0]
    first_bottom = bottom_rings[0]
    for j in range(angular_segments):
        geom.add_face(top_center, first_top[j], first_top[nxt(j)])
        geom.add_face(bottom_center, first_bottom[nxt(j)], first_bottom[j])

    for i in range(radial_segments - 1):
        inner_top = top_rings[i]
        outer_top = top_rings[i + 1]
        inner_bottom = bottom_rings[i]
        outer_bottom = bottom_rings[i + 1]
        for j in range(angular_segments):
            k = nxt(j)
            geom.add_face(inner_top[j], outer_top[j], outer_top[k])
            geom.add_face(inner_top[j], outer_top[k], inner_top[k])
            geom.add_face(inner_bottom[j], outer_bottom[k], outer_bottom[j])
            geom.add_face(inner_bottom[j], inner_bottom[k], outer_bottom[k])

    top_outer = top_rings[-1]
    bottom_outer = bottom_rings[-1]
    for j in range(angular_segments):
        k = nxt(j)
        geom.add_face(bottom_outer[j], top_outer[j], top_outer[k])
        geom.add_face(bottom_outer[j], top_outer[k], bottom_outer[k])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.18, 0.19, 0.20, 1.0))
    walnut = model.material("molded_walnut", rgba=(0.46, 0.24, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=chrome,
        name="base_disc",
    )
    pedestal.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_chrome,
        name="base_shadow",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=chrome,
        name="post",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=chrome,
        name="base_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.058, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=chrome,
        name="upper_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.6575)),
        material=dark_chrome,
        name="top_bearing",
    )
    pedestal.visual(
        Box((0.026, 0.130, 0.035)),
        origin=Origin(xyz=(0.043, 0.0, 0.287)),
        material=chrome,
        name="hinge_bridge",
    )
    pedestal.visual(
        Box((0.032, 0.010, 0.052)),
        origin=Origin(xyz=(0.064, -0.064, 0.287)),
        material=chrome,
        name="hinge_cheek_0",
    )
    pedestal.visual(
        Box((0.032, 0.010, 0.052)),
        origin=Origin(xyz=(0.064, 0.064, 0.287)),
        material=chrome,
        name="hinge_cheek_1",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_geometry(_contoured_seat_mesh(), "contoured_seat_shell"),
        material=walnut,
        name="seat_shell",
    )
    seat.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_chrome,
        name="swivel_plate",
    )
    seat.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=chrome,
        name="seat_hub",
    )

    step = model.part("step")
    step.visual(
        Box((0.026, 0.110, 0.250)),
        origin=Origin(xyz=(0.023, 0.0, 0.130)),
        material=dark_chrome,
        name="tread_body",
    )
    step.visual(
        Cylinder(radius=0.012, length=0.118),
        origin=Origin(xyz=(0.023, 0.0, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    for i, z in enumerate((0.065, 0.105, 0.145, 0.185, 0.225)):
        step.visual(
            Box((0.006, 0.094, 0.008)),
            origin=Origin(xyz=(0.009, 0.0, z)),
            material=black_rubber,
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "post_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "post_to_step",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=step,
        origin=Origin(xyz=(0.052, 0.0, 0.280)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    step = object_model.get_part("step")
    seat_joint = object_model.get_articulation("post_to_seat")
    step_joint = object_model.get_articulation("post_to_step")

    ctx.check(
        "seat has continuous swivel",
        seat_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={seat_joint.articulation_type}",
    )
    ctx.check(
        "step has right angle fold-out travel",
        step_joint.motion_limits is not None
        and abs(step_joint.motion_limits.lower - 0.0) < 1e-6
        and abs(step_joint.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={step_joint.motion_limits}",
    )

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_plate",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel plate sits on the pedestal bearing",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_plate",
        elem_b="top_bearing",
        min_overlap=0.090,
        name="seat bearing is centered over the post",
    )
    ctx.expect_gap(
        step,
        pedestal,
        axis="x",
        positive_elem="tread_body",
        negative_elem="post",
        min_gap=0.015,
        name="folded step clears the post side",
    )
    ctx.expect_contact(
        step,
        pedestal,
        elem_a="hinge_barrel",
        elem_b="hinge_cheek_0",
        contact_tol=0.0002,
        name="step hinge barrel is captured by the post bracket",
    )

    stowed_aabb = ctx.part_element_world_aabb(step, elem="tread_body")
    with ctx.pose({step_joint: math.pi / 2.0}):
        deployed_aabb = ctx.part_element_world_aabb(step, elem="tread_body")
        ctx.expect_gap(
            step,
            pedestal,
            axis="x",
            positive_elem="tread_body",
            negative_elem="post",
            min_gap=0.015,
            name="deployed step clears the post side",
        )
        ctx.expect_gap(
            step,
            pedestal,
            axis="z",
            positive_elem="tread_body",
            negative_elem="base_disc",
            min_gap=0.180,
            name="deployed step is raised above the floor base",
        )

    if stowed_aabb is not None and deployed_aabb is not None:
        stowed_x = stowed_aabb[1][0] - stowed_aabb[0][0]
        deployed_x = deployed_aabb[1][0] - deployed_aabb[0][0]
        ctx.check(
            "step folds outward from vertical storage",
            deployed_x > stowed_x + 0.170 and deployed_aabb[1][0] > stowed_aabb[1][0] + 0.160,
            details=f"stowed_aabb={stowed_aabb}, deployed_aabb={deployed_aabb}",
        )
    else:
        ctx.fail("step folds outward from vertical storage", "missing tread AABB")

    return ctx.report()


object_model = build_object_model()
