from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_box_mesh(
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    fillet: float,
):
    shape = (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges()
        .fillet(fillet)
        .translate(center)
    )
    return mesh_from_cadquery(shape, name, tolerance=0.001, angular_tolerance=0.08)


def _whisk_wire(angle: float, name: str):
    c = cos(angle)
    s = sin(angle)

    def p(radial: float, z: float) -> tuple[float, float, float]:
        return (radial * c, radial * s, z)

    points = [
        p(0.014, -0.026),
        p(0.055, -0.060),
        p(0.061, -0.101),
        p(0.024, -0.148),
        p(-0.024, -0.148),
        p(-0.061, -0.101),
        p(-0.055, -0.060),
        p(-0.014, -0.026),
    ]
    geom = tube_from_spline_points(
        points,
        radius=0.0038,
        samples_per_segment=12,
        closed_spline=True,
        radial_segments=14,
    )
    return mesh_from_geometry(geom, name)


def _mixing_bowl_mesh():
    outer_profile = [
        (0.060, 0.000),
        (0.074, 0.016),
        (0.095, 0.055),
        (0.117, 0.113),
        (0.126, 0.148),
    ]
    inner_profile = [
        (0.045, 0.013),
        (0.066, 0.027),
        (0.085, 0.063),
        (0.108, 0.120),
        (0.116, 0.140),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    return mesh_from_geometry(geom, "bowl_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bright_kitchen_stand_mixer")

    shell_yellow = model.material("glossy_sunflower_shell", rgba=(1.0, 0.72, 0.05, 1.0))
    chrome = model.material("polished_stainless_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    dark = model.material("dark_rubber_feet", rgba=(0.035, 0.035, 0.04, 1.0))
    black = model.material("black_control_plastic", rgba=(0.02, 0.02, 0.025, 1.0))
    shadow = model.material("shadow_gap_black", rgba=(0.08, 0.075, 0.065, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_box_mesh("base_rounded_shell", (0.52, 0.34, 0.070), (0.060, 0.0, 0.035), fillet=0.020),
        material=shell_yellow,
        name="base_shell",
    )
    base.visual(
        _rounded_box_mesh("rear_pedestal_shell", (0.155, 0.235, 0.275), (-0.135, 0.0, 0.182), fillet=0.026),
        material=shell_yellow,
        name="rear_pedestal",
    )
    base.visual(Box((0.050, 0.038, 0.075)), origin=Origin(xyz=(-0.105, 0.134, 0.333)), material=shell_yellow, name="hinge_cheek_0")
    base.visual(Box((0.050, 0.038, 0.075)), origin=Origin(xyz=(-0.105, -0.134, 0.333)), material=shell_yellow, name="hinge_cheek_1")
    base.visual(
        Cylinder(radius=0.030, length=0.044),
        origin=Origin(xyz=(-0.105, 0.153, 0.380), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_bushing_0",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.044),
        origin=Origin(xyz=(-0.105, -0.153, 0.380), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_bushing_1",
    )
    base.visual(Box((0.29, 0.018, 0.012)), origin=Origin(xyz=(0.105, 0.092, 0.075)), material=chrome, name="slide_rail_0")
    base.visual(Box((0.29, 0.018, 0.012)), origin=Origin(xyz=(0.105, -0.092, 0.075)), material=chrome, name="slide_rail_1")
    base.visual(Box((0.37, 0.035, 0.010)), origin=Origin(xyz=(0.105, 0.0, 0.078)), material=shadow, name="slide_slot")
    for index, x in enumerate((-0.135, 0.250)):
        for y in (-0.125, 0.125):
            base.visual(
                Cylinder(radius=0.022, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material=dark,
                name=f"foot_{index}_{0 if y < 0 else 1}",
            )

    head = model.part("head")
    head.visual(
        _rounded_box_mesh("rounded_motor_head", (0.430, 0.225, 0.138), (0.200, 0.0, 0.020), fillet=0.038),
        material=shell_yellow,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.180),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.245, 0.0, -0.071)),
        material=chrome,
        name="beater_socket",
    )
    head.visual(
        Box((0.120, 0.010, 0.026)),
        origin=Origin(xyz=(0.235, -0.113, -0.010)),
        material=chrome,
        name="trim_band",
    )

    carriage = model.part("bowl_carriage")
    carriage.visual(
        _rounded_box_mesh("slide_carriage_plate", (0.278, 0.220, 0.026), (0.120, 0.0, 0.023), fillet=0.010),
        material=shell_yellow,
        name="slide_plate",
    )
    carriage.visual(Box((0.240, 0.016, 0.014)), origin=Origin(xyz=(0.110, 0.080, 0.006)), material=chrome, name="runner_0")
    carriage.visual(Box((0.240, 0.016, 0.014)), origin=Origin(xyz=(0.110, -0.080, 0.006)), material=chrome, name="runner_1")
    carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.060, tube=0.0065, radial_segments=64, tubular_segments=12), "bowl_seat_ring"),
        origin=Origin(xyz=(0.120, 0.0, 0.038)),
        material=chrome,
        name="bowl_seat",
    )
    carriage.visual(_mixing_bowl_mesh(), origin=Origin(xyz=(0.120, 0.0, 0.042)), material=chrome, name="bowl_shell")
    carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.126, tube=0.0045, radial_segments=72, tubular_segments=12), "bowl_rolled_rim"),
        origin=Origin(xyz=(0.120, 0.0, 0.192)),
        material=chrome,
        name="rolled_rim",
    )

    whisk = model.part("whisk")
    whisk.visual(Cylinder(radius=0.012, length=0.030), origin=Origin(xyz=(0.0, 0.0, -0.015)), material=chrome, name="drive_collar")
    whisk.visual(Cylinder(radius=0.0065, length=0.056), origin=Origin(xyz=(0.0, 0.0, -0.053)), material=chrome, name="center_stem")
    whisk.visual(Sphere(radius=0.016), origin=Origin(xyz=(0.0, 0.0, -0.151)), material=chrome, name="bottom_hub")
    for index, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0, pi)):
        whisk.visual(_whisk_wire(angle, f"whisk_wire_{index}"), material=chrome, name=f"wire_{index}")

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_disk",
    )
    speed_lever.visual(
        Box((0.018, 0.010, 0.087)),
        origin=Origin(xyz=(0.030, -0.013, 0.040), rpy=(0.0, 0.18, 0.0)),
        material=black,
        name="lever_paddle",
    )
    speed_lever.visual(Sphere(radius=0.015), origin=Origin(xyz=(0.040, -0.016, 0.086)), material=black, name="lever_tip")

    lock_button = model.part("lock_button")
    lock_button.visual(
        _rounded_box_mesh("lock_button_cap", (0.044, 0.018, 0.026), (0.0, 0.010, 0.0), fillet=0.004),
        material=black,
        name="button_cap",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.380)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.95),
    )
    model.articulation(
        "bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.020, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.075),
    )
    model.articulation(
        "whisk_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.245, 0.0, -0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    model.articulation(
        "speed_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.065, -0.119, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "lock_button_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.060, 0.120, 0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=-0.010, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    lock_button = object_model.get_part("lock_button")
    bowl_slide = object_model.get_articulation("bowl_slide")
    head_tilt = object_model.get_articulation("head_tilt")
    whisk_spin = object_model.get_articulation("whisk_spin")
    speed_lever_pivot = object_model.get_articulation("speed_lever_pivot")
    lock_button_slide = object_model.get_articulation("lock_button_slide")

    ctx.allow_overlap(
        base,
        lock_button,
        elem_a="rear_pedestal",
        elem_b="button_cap",
        reason="The lock button is intentionally allowed to enter its molded side recess when pressed.",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        max_gap=0.030,
        max_penetration=0.0,
        positive_elem="slide_plate",
        negative_elem="slide_rail_0",
        name="slide carriage rides just above the base rails",
    )
    ctx.expect_contact(
        head,
        whisk,
        elem_a="beater_socket",
        elem_b="drive_collar",
        contact_tol=0.002,
        name="whisk drive collar seats under the head socket",
    )
    ctx.expect_within(
        whisk,
        carriage,
        axes="xy",
        outer_elem="bowl_shell",
        margin=0.006,
        name="whisk sits inside the bowl footprint",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({bowl_slide: 0.075}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            max_gap=0.030,
            max_penetration=0.0,
            positive_elem="slide_plate",
            negative_elem="slide_rail_0",
            name="extended carriage remains supported above the rail",
        )
    ctx.check(
        "bowl carriage slides forward",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.060,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({head_tilt: 0.90}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head tilt raises the front of the motor head",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.070,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    rest_button_pos = ctx.part_world_position(lock_button)
    with ctx.pose({lock_button_slide: -0.010}):
        pressed_button_pos = ctx.part_world_position(lock_button)
        ctx.expect_gap(
            lock_button,
            base,
            axis="y",
            max_penetration=0.012,
            positive_elem="button_cap",
            negative_elem="rear_pedestal",
            name="pressed lock button enters only a shallow recess",
        )
    ctx.check(
        "lock button travels inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.008,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    ctx.check(
        "whisk uses a continuous spin joint",
        whisk_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={whisk_spin.articulation_type}",
    )
    ctx.check(
        "speed lever has a small revolute throw",
        speed_lever_pivot.motion_limits is not None
        and speed_lever_pivot.motion_limits.lower <= -0.50
        and speed_lever_pivot.motion_limits.upper >= 0.50,
        details=f"limits={speed_lever_pivot.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
