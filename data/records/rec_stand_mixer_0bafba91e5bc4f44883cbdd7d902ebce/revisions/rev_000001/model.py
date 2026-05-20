from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luxury_metallic_stand_mixer")

    champagne = model.material("champagne_metal", rgba=(0.83, 0.70, 0.50, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.92, 0.92, 0.88, 1.0))
    shadow = model.material("shadow_black", rgba=(0.025, 0.025, 0.03, 1.0))
    bowl_steel = model.material("brushed_steel", rgba=(0.82, 0.84, 0.82, 1.0))

    def section(cx: float, zc: float, width_x: float, width_y: float, radius: float) -> list[tuple[float, float, float]]:
        return [(cx + x, y, zc) for x, y in rounded_rect_profile(width_x, width_y, radius, corner_segments=10)]

    def yz_section(x: float, zc: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
        return [(x, y, zc + z) for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=12)]

    def shifted(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.50, 0.30, 0.070, corner_segments=14),
            [],
            0.055,
            center=True,
        ),
        "base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.060, 0.0, 0.0275)),
        material=champagne,
        name="base_plate",
    )
    base.visual(
        Box((0.40, 0.008, 0.014)),
        origin=Origin(xyz=(0.075, -0.151, 0.045)),
        material=chrome,
        name="front_chrome_strip",
    )
    base.visual(
        Box((0.40, 0.008, 0.014)),
        origin=Origin(xyz=(0.075, 0.151, 0.045)),
        material=chrome,
        name="rear_chrome_strip",
    )
    base.visual(
        Box((0.250, 0.024, 0.020)),
        origin=Origin(xyz=(0.110, -0.074, 0.065)),
        material=chrome,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.250, 0.024, 0.020)),
        origin=Origin(xyz=(0.110, 0.074, 0.065)),
        material=chrome,
        name="slide_rail_1",
    )
    base.visual(
        Box((0.230, 0.006, 0.012)),
        origin=Origin(xyz=(0.110, 0.0, 0.055)),
        material=shadow,
        name="slide_slot",
    )

    pedestal = section_loft(
        [
            section(-0.125, 0.055, 0.155, 0.210, 0.045),
            section(-0.132, 0.150, 0.135, 0.180, 0.042),
            section(-0.128, 0.265, 0.105, 0.150, 0.035),
            section(-0.128, 0.350, 0.112, 0.158, 0.036),
        ]
    )
    base.visual(mesh_from_geometry(pedestal, "pedestal"), material=champagne, name="pedestal")
    base.visual(
        Box((0.088, 0.176, 0.010)),
        origin=Origin(xyz=(-0.128, 0.0, 0.306)),
        material=chrome,
        name="pedestal_chrome_band",
    )
    base.visual(
        Box((0.036, 0.032, 0.086)),
        origin=Origin(xyz=(-0.130, -0.097, 0.389)),
        material=champagne,
        name="hinge_post_0",
    )
    base.visual(
        Box((0.036, 0.032, 0.086)),
        origin=Origin(xyz=(-0.130, 0.097, 0.389)),
        material=champagne,
        name="hinge_post_1",
    )
    base.visual(
        Box((0.052, 0.220, 0.026)),
        origin=Origin(xyz=(-0.130, 0.0, 0.356)),
        material=champagne,
        name="hinge_bridge",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.046),
        origin=Origin(xyz=(-0.130, -0.108, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_yoke_0",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.046),
        origin=Origin(xyz=(-0.130, 0.108, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_yoke_1",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.070, -0.151, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_mount",
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.188, 0.166, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=chrome,
        name="bowl_lock_plate",
    )
    outer_bowl = [
        (0.036, 0.000),
        (0.060, 0.018),
        (0.104, 0.075),
        (0.132, 0.165),
        (0.145, 0.225),
        (0.151, 0.238),
    ]
    inner_bowl = [
        (0.012, 0.010),
        (0.046, 0.025),
        (0.096, 0.082),
        (0.121, 0.166),
        (0.137, 0.230),
    ]
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_bowl,
        inner_bowl,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl_carriage.visual(
        mesh_from_geometry(bowl_shell, "deep_mixing_bowl"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=bowl_steel,
        name="bowl_shell",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.0, 0.132, 0.165),
            (0.0, 0.188, 0.153),
            (0.0, 0.205, 0.095),
            (0.0, 0.170, 0.060),
            (0.0, 0.098, 0.064),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
    )
    bowl_carriage.visual(
        mesh_from_geometry(handle_geom, "bowl_handle"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=chrome,
        name="bowl_handle",
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            yz_section(0.000, 0.000, 0.132, 0.108, 0.034),
            yz_section(0.110, 0.006, 0.190, 0.160, 0.052),
            yz_section(0.285, -0.004, 0.188, 0.150, 0.050),
            yz_section(0.430, -0.014, 0.140, 0.118, 0.040),
        ]
    )
    head.visual(mesh_from_geometry(head_shell, "sculpted_head_shell"), material=champagne, name="head_shell")
    head.visual(
        Cylinder(radius=0.017, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.066, length=0.015),
        origin=Origin(xyz=(0.426, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="nose_cap",
    )
    head.visual(
        Box((0.260, 0.018, 0.010)),
        origin=Origin(xyz=(0.215, 0.0, 0.082)),
        material=chrome,
        name="top_chrome_crest",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.031, tube=0.004, radial_segments=18, tubular_segments=48), "drive_trim_ring"),
        origin=Origin(xyz=(0.285, 0.0, -0.080)),
        material=chrome,
        name="drive_trim_ring",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.285, 0.0, -0.080)),
        material=shadow,
        name="drive_socket",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=chrome,
        name="drive_shaft",
    )
    paddle.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=chrome,
        name="paddle_collar",
    )
    outer_paddle = [
        (-0.014, -0.034),
        (0.014, -0.034),
        (0.034, -0.060),
        (0.050, -0.112),
        (0.038, -0.160),
        (0.000, -0.184),
        (-0.038, -0.160),
        (-0.050, -0.112),
        (-0.034, -0.060),
    ]
    hole = superellipse_profile(0.021, 0.070, exponent=2.4, segments=32)
    paddle_plate = ExtrudeWithHolesGeometry(
        outer_paddle,
        [
            shifted(hole, -0.022, -0.111),
            shifted(hole, 0.022, -0.111),
        ],
        0.010,
        center=True,
    )
    paddle.visual(
        mesh_from_geometry(paddle_plate, "flat_paddle_blade"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bowl_steel,
        name="paddle_blade",
    )

    speed_dial = model.part("speed_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.026,
            body_style="skirted",
            top_diameter=0.047,
            skirt=KnobSkirt(0.066, 0.006, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=22, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "speed_dial_knob",
    )
    speed_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_knob",
    )
    speed_dial.visual(
        Cylinder(radius=0.008, length=0.007),
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="dial_pointer",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="plunger_stem",
    )
    head_lock.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="plunger_button",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.130, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=-0.020, upper=0.040),
    )
    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.130, 0.0, 0.435)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.9, lower=0.0, upper=math.radians(58.0)),
    )
    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.285, 0.0, -0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7.0, velocity=25.0),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.070, -0.157, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-math.radians(45.0), upper=math.radians(135.0)),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.136, 0.085, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl_carriage")
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    paddle = object_model.get_part("paddle")
    dial = object_model.get_part("speed_dial")
    lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    hinge = object_model.get_articulation("rear_hinge")
    drive = object_model.get_articulation("head_to_paddle")
    speed = object_model.get_articulation("base_to_speed_dial")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "required mechanisms use requested joint types",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and drive.articulation_type == ArticulationType.CONTINUOUS
        and speed.articulation_type == ArticulationType.REVOLUTE
        and lock_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"bowl={bowl_slide.articulation_type}, hinge={hinge.articulation_type}, "
            f"drive={drive.articulation_type}, speed={speed.articulation_type}, lock={lock_slide.articulation_type}"
        ),
    )
    ctx.check(
        "controls remain mounted on base",
        speed.parent == "base" and speed.child == "speed_dial" and lock_slide.parent == "base" and lock_slide.child == "head_lock",
        details=f"speed=({speed.parent}->{speed.child}), lock=({lock_slide.parent}->{lock_slide.child})",
    )
    ctx.check(
        "paddle drive is vertical",
        tuple(round(v, 6) for v in drive.axis) == (0.0, 0.0, 1.0),
        details=f"axis={drive.axis}",
    )
    ctx.allow_overlap(
        head,
        paddle,
        elem_a="drive_socket",
        elem_b="drive_shaft",
        reason="The rotating shaft is intentionally captured inside the head drive socket.",
    )
    ctx.expect_within(
        paddle,
        head,
        axes="xy",
        inner_elem="drive_shaft",
        outer_elem="drive_socket",
        margin=0.0,
        name="paddle shaft centered in drive socket",
    )
    ctx.expect_overlap(
        paddle,
        head,
        axes="z",
        elem_a="drive_shaft",
        elem_b="drive_socket",
        min_overlap=0.008,
        name="paddle shaft remains inserted in socket",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="slide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage rests on slide rail",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="x",
        elem_a="carriage_plate",
        elem_b="slide_rail_0",
        min_overlap=0.120,
        name="bowl carriage retained on base slide",
    )

    rest_bowl = ctx.part_world_position(bowl)
    rest_lock = ctx.part_world_position(lock)
    rest_nose = ctx.part_element_world_aabb(head, elem="nose_cap")
    with ctx.pose({bowl_slide: 0.040, hinge: math.radians(58.0), lock_slide: 0.018}):
        extended_bowl = ctx.part_world_position(bowl)
        extended_lock = ctx.part_world_position(lock)
        tilted_nose = ctx.part_element_world_aabb(head, elem="nose_cap")
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="slide_rail_0",
            min_overlap=0.080,
            name="extended bowl slide remains engaged",
        )

    ctx.check(
        "bowl slide moves forward a short travel",
        rest_bowl is not None and extended_bowl is not None and 0.035 <= extended_bowl[0] - rest_bowl[0] <= 0.045,
        details=f"rest={rest_bowl}, extended={extended_bowl}",
    )
    ctx.check(
        "head lock plunger moves outward",
        rest_lock is not None and extended_lock is not None and extended_lock[1] - rest_lock[1] > 0.014,
        details=f"rest={rest_lock}, extended={extended_lock}",
    )
    ctx.check(
        "rear hinge tilts head upward",
        rest_nose is not None and tilted_nose is not None and tilted_nose[1][2] > rest_nose[1][2] + 0.050,
        details=f"rest_nose={rest_nose}, tilted_nose={tilted_nose}",
    )
    ctx.expect_within(
        paddle,
        bowl,
        axes="xy",
        inner_elem="paddle_blade",
        outer_elem="bowl_shell",
        margin=0.006,
        name="paddle sits inside deep bowl footprint",
    )

    return ctx.report()


object_model = build_object_model()
