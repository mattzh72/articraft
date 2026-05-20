from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cq_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Hollow cylinder centered on Z, used where the rotating steerer passes."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )


def _rounded_box(size: tuple[float, float, float], fillet: float) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(fillet)


def _cylinder_x(radius: float, length: float, x: float, y: float, z: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)
    )


def _cylinder_y(radius: float, length: float, x: float, y: float, z: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_jump_fork_steering")

    chrome = model.material("polished_chrome_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.60, 0.62, 0.64, 1.0))
    black = model.material("satin_black", rgba=(0.02, 0.02, 0.018, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    frame_blue = model.material("dark_blue_frame_paint", rgba=(0.02, 0.07, 0.16, 1.0))

    head_tube = model.part("head_tube")
    head_tube.visual(
        mesh_from_cadquery(_cq_tube(0.050, 0.029, 0.230), "head_tube_shell"),
        material=frame_blue,
        name="head_tube_shell",
    )
    head_tube.visual(
        mesh_from_cadquery(_cq_tube(0.058, 0.031, 0.024), "upper_bearing_cup"),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=brushed,
        name="upper_bearing_cup",
    )
    head_tube.visual(
        mesh_from_cadquery(_cq_tube(0.058, 0.031, 0.024), "lower_bearing_cup"),
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
        material=brushed,
        name="lower_bearing_cup",
    )

    # Short frame stubs make the root read as a fixed bicycle head tube without
    # adding unrelated bicycle mechanisms.
    down_geo, down_origin = _cylinder_y(0.022, 0.24, 0.0, -0.180, -0.070)
    head_tube.visual(
        down_geo,
        origin=down_origin,
        material=frame_blue,
        name="down_tube_stub",
    )
    head_tube.visual(
        Box((0.070, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, -0.056, -0.070)),
        material=frame_blue,
        name="down_tube_lug",
    )
    top_geo, top_origin = _cylinder_y(0.018, 0.20, 0.0, -0.150, 0.070)
    head_tube.visual(
        top_geo,
        origin=top_origin,
        material=frame_blue,
        name="top_tube_stub",
    )
    head_tube.visual(
        Box((0.060, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, -0.055, 0.070)),
        material=frame_blue,
        name="top_tube_lug",
    )

    steering = model.part("steering")

    # Continuous threaded steerer: it runs through the hollow head tube and ties
    # the crown and quill stem together on the real steering axis.
    steering.visual(
        Cylinder(radius=0.020, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed,
        name="steerer",
    )
    for idx, z in enumerate([0.155, 0.168, 0.181, 0.194, 0.207, 0.220, 0.233, 0.246]):
        steering.visual(
            mesh_from_geometry(TorusGeometry(radius=0.0208, tube=0.0014, tubular_segments=24), f"thread_ring_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"thread_ring_{idx}",
        )
    steering.visual(
        mesh_from_cadquery(_cq_tube(0.045, 0.019, 0.020), "upper_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=chrome,
        name="upper_bearing_race",
    )
    steering.visual(
        mesh_from_cadquery(_cq_tube(0.043, 0.019, 0.020), "lower_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, -0.139)),
        material=chrome,
        name="lower_bearing_race",
    )

    # Wide two-plate triple-clamp crown with three clamp stations: both fork legs
    # and the steerer. The plates are chromed rounded steel, while black grooves
    # and bolts mark the pinch-split details.
    for label, z in (("upper", -0.168), ("lower", -0.253)):
        steering.visual(
            mesh_from_cadquery(_rounded_box((0.350, 0.105, 0.044), 0.020), f"{label}_crown_plate"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"{label}_crown_plate",
        )
        for clamp_x in (-0.112, 0.0, 0.112):
            steering.visual(
                Cylinder(radius=0.037, length=0.056),
                origin=Origin(xyz=(clamp_x, 0.0, z)),
                material=chrome,
                name=f"{label}_clamp_{clamp_x:+.3f}",
            )
            steering.visual(
                Box((0.006, 0.006, 0.052)),
                origin=Origin(xyz=(clamp_x, -0.055, z)),
                material=black,
                name=f"{label}_split_{clamp_x:+.3f}",
            )
            bolt_geo, bolt_origin = _cylinder_y(0.0045, 0.052, clamp_x + 0.024, -0.060, z + 0.010)
            steering.visual(
                bolt_geo,
                origin=bolt_origin,
                material=black,
                name=f"{label}_pinch_bolt_a_{clamp_x:+.3f}",
            )
            bolt_geo, bolt_origin = _cylinder_y(0.0045, 0.052, clamp_x - 0.024, -0.060, z - 0.010)
            steering.visual(
                bolt_geo,
                origin=bolt_origin,
                material=black,
                name=f"{label}_pinch_bolt_b_{clamp_x:+.3f}",
            )

    # Two parallel round fork legs with forged-looking dropouts.
    for idx, x in enumerate((-0.112, 0.112)):
        steering.visual(
            Cylinder(radius=0.019, length=0.585),
            origin=Origin(xyz=(x, 0.0, -0.465)),
            material=chrome,
            name=f"fork_leg_{idx}",
        )
        steering.visual(
            mesh_from_cadquery(_rounded_box((0.050, 0.030, 0.090), 0.010), f"dropout_{idx}"),
            origin=Origin(xyz=(x, 0.0, -0.785)),
            material=chrome,
            name=f"dropout_{idx}",
        )
        axle_geo, axle_origin = _cylinder_x(0.010, 0.006, x, -0.016, -0.796)
        steering.visual(
            axle_geo,
            origin=axle_origin,
            material=black,
            name=f"axle_slot_{idx}",
        )
    axle_geo, axle_origin = _cylinder_x(0.0075, 0.250, 0.0, -0.016, -0.796)
    steering.visual(
        axle_geo,
        origin=axle_origin,
        material=brushed,
        name="axle_rod",
    )

    # Quill stem and handlebar clamp collar. The swept tube links the vertical
    # steerer clamp to the transverse handlebar clamp so the whole child part is
    # a single supported steering assembly.
    steering.visual(
        Cylinder(radius=0.030, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=chrome,
        name="quill_collar",
    )
    steering.visual(
        Box((0.010, 0.008, 0.135)),
        origin=Origin(xyz=(0.0, -0.031, 0.245)),
        material=black,
        name="quill_split",
    )
    for z in (0.215, 0.275):
        bolt_geo, bolt_origin = _cylinder_y(0.005, 0.052, 0.020, -0.034, z)
        steering.visual(
            bolt_geo,
            origin=bolt_origin,
            material=black,
            name=f"quill_pinch_bolt_{z:.3f}",
        )

    stem_tube = tube_from_spline_points(
        [
            (0.0, 0.000, 0.280),
            (0.0, 0.018, 0.315),
            (0.0, 0.035, 0.350),
            (0.0, 0.042, 0.382),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=24,
        cap_ends=True,
    )
    steering.visual(
        mesh_from_geometry(stem_tube, "stem_neck"),
        material=chrome,
        name="stem_neck",
    )
    clamp_geo, clamp_origin = _cylinder_x(0.034, 0.116, 0.0, 0.044, 0.382)
    steering.visual(
        clamp_geo,
        origin=clamp_origin,
        material=chrome,
        name="bar_clamp_collar",
    )
    steering.visual(
        Box((0.112, 0.020, 0.054)),
        origin=Origin(xyz=(0.0, 0.004, 0.382)),
        material=chrome,
        name="bar_clamp_face",
    )
    for x in (-0.040, 0.040):
        for z in (0.365, 0.399):
            bolt_geo, bolt_origin = _cylinder_y(0.0045, 0.060, x, -0.004, z)
            steering.visual(
                bolt_geo,
                origin=bolt_origin,
                material=black,
                name=f"bar_bolt_{x:+.2f}_{z:.3f}",
            )

    bar_tube = tube_from_spline_points(
        [
            (-0.430, 0.040, 0.505),
            (-0.335, 0.040, 0.505),
            (-0.250, 0.044, 0.488),
            (-0.155, 0.048, 0.430),
            (-0.060, 0.046, 0.388),
            (0.000, 0.044, 0.382),
            (0.060, 0.046, 0.388),
            (0.155, 0.048, 0.430),
            (0.250, 0.044, 0.488),
            (0.335, 0.040, 0.505),
            (0.430, 0.040, 0.505),
        ],
        radius=0.0125,
        samples_per_segment=16,
        radial_segments=24,
        cap_ends=True,
    )
    steering.visual(
        mesh_from_geometry(bar_tube, "riser_handlebar"),
        material=chrome,
        name="riser_handlebar",
    )
    for idx, x in enumerate((-0.390, 0.390)):
        grip_geo, grip_origin = _cylinder_x(0.018, 0.110, x, 0.040, 0.505)
        steering.visual(
            grip_geo,
            origin=grip_origin,
            material=rubber,
            name=f"grip_{idx}",
        )
        for offset in (-0.040, -0.020, 0.0, 0.020, 0.040):
            steering.visual(
                mesh_from_geometry(TorusGeometry(radius=0.0182, tube=0.0013, tubular_segments=18), f"grip_rib_{idx}_{offset:+.2f}"),
                origin=Origin(xyz=(x + offset, 0.040, 0.505), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"grip_rib_{idx}_{offset:+.2f}",
            )

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    steering = object_model.get_part("steering")
    head_tube = object_model.get_part("head_tube")
    joint = object_model.get_articulation("steering_axis")

    ctx.check(
        "single head-tube steering revolute",
        joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube_shell",
        margin=0.0,
        name="steerer remains centered in hollow head tube",
    )
    ctx.expect_overlap(
        steering,
        head_tube,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube_shell",
        min_overlap=0.20,
        name="steerer spans the head tube bearings",
    )

    def _center_from_aabb(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    leg_0 = _center_from_aabb(ctx.part_element_world_aabb(steering, elem="fork_leg_0"))
    leg_1 = _center_from_aabb(ctx.part_element_world_aabb(steering, elem="fork_leg_1"))
    ctx.check(
        "parallel fork legs are wide and level",
        abs(leg_0[1] - leg_1[1]) < 0.003
        and abs(leg_0[2] - leg_1[2]) < 0.003
        and 0.20 <= abs(leg_1[0] - leg_0[0]) <= 0.24,
        details=f"leg centers={leg_0}, {leg_1}",
    )

    grip_0 = _center_from_aabb(ctx.part_element_world_aabb(steering, elem="grip_0"))
    grip_1 = _center_from_aabb(ctx.part_element_world_aabb(steering, elem="grip_1"))
    ctx.check(
        "wide riser handlebar span",
        abs(grip_1[0] - grip_0[0]) > 0.70 and grip_0[2] > 0.48 and grip_1[2] > 0.48,
        details=f"grip centers={grip_0}, {grip_1}",
    )

    with ctx.pose({joint: 0.80}):
        turned_grip_0 = _center_from_aabb(ctx.part_element_world_aabb(steering, elem="grip_0"))
        ctx.expect_overlap(
            steering,
            head_tube,
            axes="z",
            elem_a="steerer",
            elem_b="head_tube_shell",
            min_overlap=0.20,
            name="turned steerer stays captured in head tube",
        )
    ctx.check(
        "handlebar turns about steering axis",
        abs(turned_grip_0[1] - grip_0[1]) > 0.20 and abs(turned_grip_0[2] - grip_0[2]) < 0.010,
        details=f"rest={grip_0}, turned={turned_grip_0}",
    )

    return ctx.report()


object_model = build_object_model()
