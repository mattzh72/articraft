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
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_box(size: tuple[float, float, float], radius: float, center: tuple[float, float, float]):
    body = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        body = body.edges().fillet(radius)
    return body.translate(center)


def _stand_base_body():
    shoe = _rounded_box((0.58, 0.36, 0.08), 0.025, (0.04, 0.0, 0.04))
    rear_pedestal = _rounded_box((0.17, 0.24, 0.35), 0.030, (-0.19, 0.0, 0.235))
    neck_cap = _rounded_box((0.22, 0.28, 0.055), 0.022, (-0.18, 0.0, 0.405))
    rail_boss = _rounded_box((0.24, 0.25, 0.030), 0.012, (0.09, 0.0, 0.095))
    body = shoe.union(rear_pedestal).union(neck_cap).union(rail_boss)
    return body


def _head_body():
    shell = _rounded_box((0.40, 0.24, 0.13), 0.045, (0.17, 0.0, 0.025))
    front_nose = _rounded_box((0.16, 0.20, 0.085), 0.035, (0.285, 0.0, -0.030))
    rear_bulge = _rounded_box((0.12, 0.22, 0.115), 0.035, (0.010, 0.0, 0.015))
    return shell.union(front_nose).union(rear_bulge)


def _bowl_shell_geometry():
    outer_profile = [
        (0.050, 0.075),
        (0.075, 0.095),
        (0.112, 0.165),
        (0.132, 0.230),
        (0.136, 0.245),
    ]
    inner_profile = [
        (0.034, 0.092),
        (0.061, 0.112),
        (0.096, 0.172),
        (0.117, 0.228),
        (0.122, 0.238),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )


def _whisk_wire_geometry():
    wires = MeshGeometry()
    for i in range(6):
        angle = i * math.tau / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        points = [
            (0.016 * c, 0.016 * s, -0.075),
            (0.050 * c, 0.050 * s, -0.110),
            (0.078 * c, 0.078 * s, -0.170),
            (0.046 * c, 0.046 * s, -0.205),
            (-0.046 * c, -0.046 * s, -0.205),
            (-0.078 * c, -0.078 * s, -0.170),
            (-0.050 * c, -0.050 * s, -0.110),
            (-0.016 * c, -0.016 * s, -0.075),
        ]
        wires.merge(
            tube_from_spline_points(
                points,
                radius=0.0023,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            )
        )
    return wires


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bakery_corner_stand_mixer")

    painted = model.material("warm_cream_paint", color=(0.86, 0.78, 0.63, 1.0))
    painted_shadow = model.material("painted_shadow", color=(0.62, 0.52, 0.42, 1.0))
    stainless = model.material("brushed_stainless", color=(0.78, 0.77, 0.72, 1.0))
    dark = model.material("dark_control", color=(0.06, 0.055, 0.050, 1.0))
    chrome = model.material("polished_chrome", color=(0.88, 0.88, 0.84, 1.0))
    red_mark = model.material("red_speed_mark", color=(0.70, 0.04, 0.02, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box((0.58, 0.36, 0.08), 0.025, (0.04, 0.0, 0.04)), "painted_pedestal"),
        material=painted,
        name="painted_pedestal",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.17, 0.24, 0.35), 0.030, (-0.19, 0.0, 0.235)), "rear_pedestal"),
        material=painted,
        name="rear_pedestal",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.22, 0.28, 0.055), 0.022, (-0.18, 0.0, 0.405)), "neck_cap"),
        material=painted,
        name="neck_cap",
    )
    base.visual(
        mesh_from_cadquery(_rounded_box((0.24, 0.25, 0.030), 0.012, (0.09, 0.0, 0.095)), "front_rail_boss"),
        material=painted,
        name="front_rail_boss",
    )
    for y, name in ((-0.075, "slide_rail_0"), (0.075, "slide_rail_1")):
        base.visual(
            Box((0.24, 0.024, 0.028)),
            origin=Origin(xyz=(0.09, y, 0.094)),
            material=chrome,
            name=name,
        )
    base.visual(
        Box((0.110, 0.006, 0.040)),
        origin=Origin(xyz=(-0.080, 0.118, 0.215)),
        material=dark,
        name="lock_slot",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(-0.080, -0.118, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_shadow,
        name="speed_dial_plate",
    )
    for y, name in ((-0.145, "hinge_cheek_0"), (0.145, "hinge_cheek_1")):
        base.visual(
            Box((0.070, 0.030, 0.120)),
            origin=Origin(xyz=(-0.190, y, 0.460)),
            material=painted,
            name=name,
        )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.165, 0.215, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=painted,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=painted,
        name="bowl_foot",
    )
    bowl_carriage.visual(
        mesh_from_geometry(_bowl_shell_geometry(), "stainless_bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        mesh_from_geometry(TorusGeometry(0.129, 0.006, radial_segments=24, tubular_segments=96), "bowl_rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=chrome,
        name="bowl_rim",
    )
    bowl_carriage.visual(
        Box((0.115, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.105, 0.048)),
        material=chrome,
        name="front_lip",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_body(), "substantial_head"),
        material=painted,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_shadow,
        name="rear_hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.032),
        origin=Origin(xyz=(0.280, 0.0, -0.085)),
        material=chrome,
        name="mixing_socket",
    )
    head.visual(
        Box((0.100, 0.010, 0.022)),
        origin=Origin(xyz=(0.285, -0.125, 0.020)),
        material=painted_shadow,
        name="side_trim",
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.0075, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=stainless,
        name="shaft",
    )
    whisk.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=chrome,
        name="wire_collar",
    )
    whisk.visual(
        mesh_from_geometry(_whisk_wire_geometry(), "wire_whisk_cage"),
        material=stainless,
        name="wire_cage",
    )
    whisk.visual(
        Box((0.012, 0.006, 0.006)),
        origin=Origin(xyz=(0.029, 0.0, -0.083)),
        material=red_mark,
        name="rotation_mark",
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.065,
                0.026,
                body_style="skirted",
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "fluted_speed_control",
        ),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="speed_knob",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.066, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=dark,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.018, 0.034, 0.034)),
        origin=Origin(xyz=(0.028, 0.018, 0.0)),
        material=chrome,
        name="finger_tab",
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.090, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.080),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.190, 0.0, 0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=0.0, upper=0.850),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.280, 0.0, -0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.080, -0.121, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.080, 0.121, 0.215)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.10, lower=0.0, upper=0.035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    head_lock = object_model.get_part("head_lock")
    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    whisk_spin = object_model.get_articulation("head_to_whisk")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.allow_overlap(
        head,
        whisk,
        elem_a="mixing_socket",
        elem_b="shaft",
        reason="The whisk drive shaft is intentionally seated inside the head's mixer socket.",
    )

    ctx.check(
        "all requested mechanisms are explicit",
        all(
            joint is not None
            for joint in (
                bowl_slide,
                head_hinge,
                whisk_spin,
                object_model.get_articulation("base_to_speed_control"),
                lock_slide,
            )
        ),
        details="Expected bowl slide, head hinge, whisk spin, speed control, and lock slider joints.",
    )

    ctx.expect_contact(
        bowl_carriage,
        base,
        elem_a="carriage_plate",
        elem_b="slide_rail_0",
        contact_tol=0.0015,
        name="carriage rides on base rails",
    )
    ctx.expect_overlap(
        bowl_carriage,
        base,
        axes="xy",
        elem_a="carriage_plate",
        elem_b="slide_rail_0",
        min_overlap=0.020,
        name="front carriage overlaps rail footprint",
    )
    ctx.expect_within(
        whisk,
        bowl_carriage,
        axes="xy",
        inner_elem="wire_cage",
        outer_elem="bowl_shell",
        margin=0.005,
        name="whisk sits inside bowl plan",
    )
    ctx.expect_gap(
        head,
        bowl_carriage,
        axis="z",
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        min_gap=0.040,
        name="head clears the bowl rim",
    )
    ctx.expect_within(
        whisk,
        head,
        axes="xy",
        inner_elem="shaft",
        outer_elem="mixing_socket",
        margin=0.002,
        name="whisk shaft is centered in mixer socket",
    )
    ctx.expect_overlap(
        whisk,
        head,
        axes="z",
        elem_a="shaft",
        elem_b="mixing_socket",
        min_overlap=0.010,
        name="whisk shaft stays inserted in socket",
    )

    rest_bowl = ctx.part_world_position(bowl_carriage)
    with ctx.pose({bowl_slide: 0.080}):
        extended_bowl = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage translates forward",
        rest_bowl is not None and extended_bowl is not None and extended_bowl[0] > rest_bowl[0] + 0.070,
        details=f"rest={rest_bowl}, extended={extended_bowl}",
    )

    socket_rest = ctx.part_element_world_aabb(head, elem="mixing_socket")
    with ctx.pose({head_hinge: 0.800}):
        socket_tilted = ctx.part_element_world_aabb(head, elem="mixing_socket")
    if socket_rest is not None and socket_tilted is not None:
        rest_z = (socket_rest[0][2] + socket_rest[1][2]) * 0.5
        tilted_z = (socket_tilted[0][2] + socket_tilted[1][2]) * 0.5
    else:
        rest_z = tilted_z = None
    ctx.check(
        "head hinge tilts socket upward",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.12,
        details=f"rest_z={rest_z}, tilted_z={tilted_z}",
    )

    mark_rest = ctx.part_element_world_aabb(whisk, elem="rotation_mark")
    with ctx.pose({whisk_spin: math.pi / 2.0}):
        mark_rotated = ctx.part_element_world_aabb(whisk, elem="rotation_mark")
    if mark_rest is not None and mark_rotated is not None:
        rest_center = (
            (mark_rest[0][0] + mark_rest[1][0]) * 0.5,
            (mark_rest[0][1] + mark_rest[1][1]) * 0.5,
        )
        rotated_center = (
            (mark_rotated[0][0] + mark_rotated[1][0]) * 0.5,
            (mark_rotated[0][1] + mark_rotated[1][1]) * 0.5,
        )
    else:
        rest_center = rotated_center = None
    ctx.check(
        "whisk continuous joint visibly rotates",
        rest_center is not None
        and rotated_center is not None
        and abs(rotated_center[1] - rest_center[1]) > 0.020,
        details=f"rest={rest_center}, rotated={rotated_center}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: 0.035}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider has short stroke",
        lock_rest is not None and lock_extended is not None and lock_extended[0] > lock_rest[0] + 0.030,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


object_model = build_object_model()
