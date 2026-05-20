from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _superellipse_section(
    x: float,
    *,
    width_y: float,
    height_z: float,
    z_offset: float = 0.0,
    exponent: float = 3.4,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    """Closed rounded-rectangle-like YZ section for lofted mixer shells."""

    points: list[tuple[float, float, float]] = []
    for index in range(segments):
        theta = (2.0 * math.pi * index) / segments
        c = math.cos(theta)
        s = math.sin(theta)
        y = math.copysign(abs(c) ** (2.0 / exponent), c) * width_y * 0.5
        z = z_offset + math.copysign(abs(s) ** (2.0 / exponent), s) * height_z * 0.5
        points.append((x, y, z))
    return points


def _head_shell_geometry() -> MeshGeometry:
    return section_loft(
        [
            _superellipse_section(0.040, width_y=0.110, height_z=0.096, z_offset=-0.002),
            _superellipse_section(0.108, width_y=0.150, height_z=0.128, z_offset=0.003),
            _superellipse_section(0.225, width_y=0.158, height_z=0.118, z_offset=-0.006),
            _superellipse_section(0.335, width_y=0.112, height_z=0.086, z_offset=-0.016),
        ]
    )


def _neck_geometry() -> MeshGeometry:
    return section_loft(
        [
            [(x, y, 0.000) for x, y in rounded_rect_profile(0.112, 0.142, 0.030)],
            [(x, y, 0.080) for x, y in rounded_rect_profile(0.102, 0.132, 0.030)],
            [(x, y, 0.145) for x, y in rounded_rect_profile(0.090, 0.116, 0.026)],
            [(x, y, 0.190) for x, y in rounded_rect_profile(0.078, 0.102, 0.023)],
        ]
    )


def _beater_geometry() -> MeshGeometry:
    beater = CylinderGeometry(radius=0.0052, height=0.052, radial_segments=24).translate(
        0.0,
        0.0,
        -0.032,
    )
    beater.merge(
        CylinderGeometry(radius=0.010, height=0.018, radial_segments=28).translate(
            0.0,
            0.0,
            -0.064,
        )
    )

    for angle in (0.0, math.pi / 2.0):
        c = math.cos(angle)
        s = math.sin(angle)
        loop = tube_from_spline_points(
            [
                (0.000, 0.000, -0.060),
                (0.020 * c, 0.020 * s, -0.074),
                (0.036 * c, 0.036 * s, -0.110),
                (0.000, 0.000, -0.148),
                (-0.036 * c, -0.036 * s, -0.110),
                (-0.020 * c, -0.020 * s, -0.074),
                (0.000, 0.000, -0.060),
            ],
            radius=0.0017,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
        beater.merge(loop)

    return beater


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_stand_mixer")

    enamel = model.material("warm_cream_enamel", rgba=(0.86, 0.78, 0.63, 1.0))
    enamel_shadow = model.material("cream_shadow", rgba=(0.68, 0.59, 0.45, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.77, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.075, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.06, 0.065, 0.07, 1.0))
    trim_chrome = model.material("trim_chrome", rgba=(0.92, 0.88, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh(
            "rounded_base_plate",
            ExtrudeGeometry(rounded_rect_profile(0.380, 0.230, 0.050, corner_segments=10), 0.052),
        ),
        origin=Origin(xyz=(0.055, 0.0, 0.026)),
        material=enamel,
        name="rounded_base_plate",
    )
    base.visual(
        _save_mesh("rear_pedestal", _neck_geometry()),
        origin=Origin(xyz=(-0.095, 0.0, 0.050)),
        material=enamel,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.235, 0.018, 0.022)),
        origin=Origin(xyz=(0.070, 0.101, 0.063)),
        material=enamel_shadow,
        name="carriage_rail_0",
    )
    base.visual(
        Box((0.235, 0.018, 0.022)),
        origin=Origin(xyz=(0.070, -0.101, 0.063)),
        material=enamel_shadow,
        name="carriage_rail_1",
    )
    base.visual(
        Box((0.102, 0.125, 0.014)),
        origin=Origin(xyz=(-0.105, 0.0, 0.246)),
        material=enamel,
        name="hinge_saddle",
    )
    base.visual(
        Box((0.102, 0.026, 0.035)),
        origin=Origin(xyz=(-0.105, 0.075, 0.2695)),
        material=enamel,
        name="hinge_boss_0",
    )
    base.visual(
        Box((0.102, 0.026, 0.035)),
        origin=Origin(xyz=(-0.105, -0.075, 0.2695)),
        material=enamel,
        name="hinge_boss_1",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(-0.105, 0.062, 0.307), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_chrome,
        name="hinge_barrel_0",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(-0.105, -0.062, 0.307), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_chrome,
        name="hinge_barrel_1",
    )
    for index, x in enumerate((-0.085, 0.200)):
        for side, y in enumerate((-0.082, 0.082)):
            base.visual(
                Sphere(radius=0.015),
                origin=Origin(xyz=(x, y, 0.010)),
                material=dark_rubber,
                name=f"rubber_foot_{index}_{side}",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.42, 0.26, 0.36)),
        mass=5.2,
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.205, 0.170, 0.026)),
        origin=Origin(xyz=(0.060, 0.0, 0.013)),
        material=enamel,
        name="front_carriage",
    )
    bowl.visual(
        Box((0.138, 0.045, 0.018)),
        origin=Origin(xyz=(0.034, 0.0, 0.006)),
        material=enamel_shadow,
        name="slide_tongue",
    )
    bowl.visual(
        _save_mesh(
            "small_mixing_bowl_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.028, 0.000),
                    (0.054, 0.010),
                    (0.077, 0.052),
                    (0.090, 0.110),
                    (0.096, 0.138),
                ],
                [
                    (0.018, 0.007),
                    (0.045, 0.018),
                    (0.069, 0.055),
                    (0.083, 0.111),
                    (0.089, 0.130),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.080, 0.0, 0.026)),
        material=brushed_steel,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.046, 0.017, 0.018)),
        origin=Origin(xyz=(0.080, 0.094, 0.122)),
        material=brushed_steel,
        name="bowl_handle_0",
    )
    bowl.visual(
        Box((0.046, 0.017, 0.018)),
        origin=Origin(xyz=(0.080, -0.094, 0.122)),
        material=brushed_steel,
        name="bowl_handle_1",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.096, length=0.164),
        mass=0.75,
        origin=Origin(xyz=(0.080, 0.0, 0.095)),
    )

    head = model.part("head")
    head.visual(
        _save_mesh("rounded_mixer_head", _head_shell_geometry()),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.050, 0.044, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=enamel,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.087),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_chrome,
        name="hinge_knuckle",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.232, 0.0, -0.060)),
        material=black_plastic,
        name="drive_hub",
    )
    head.visual(
        Box((0.115, 0.010, 0.020)),
        origin=Origin(xyz=(0.210, 0.081, -0.005)),
        material=trim_chrome,
        name="side_trim",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.335, 0.0, -0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_chrome,
        name="nose_cap",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.17, 0.14)),
        mass=2.7,
        origin=Origin(xyz=(0.16, 0.0, -0.004)),
    )

    beater = model.part("beater")
    beater.visual(
        _save_mesh("compact_flat_beater", _beater_geometry()),
        material=brushed_steel,
        name="beater_wire",
    )
    beater.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.160),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="lever_pivot",
    )
    speed_lever.visual(
        Box((0.012, 0.012, 0.052)),
        origin=Origin(xyz=(0.010, 0.010, 0.020), rpy=(0.0, -0.35, 0.0)),
        material=black_plastic,
        name="lever_stem",
    )
    speed_lever.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.019, 0.013, 0.041)),
        material=black_plastic,
        name="lever_tip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.050, 0.035, 0.075)),
        mass=0.04,
        origin=Origin(xyz=(0.014, 0.012, 0.026)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.034, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material=black_plastic,
        name="lock_button",
    )
    head_lock.visual(
        Box((0.044, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=trim_chrome,
        name="lock_bezel",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.046, 0.018, 0.030)),
        mass=0.035,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.035, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=0.0, upper=0.045),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.307)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.232, 0.0, -0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(0.030, 0.1145, 0.066)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.088, -0.0582, 0.236)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.06, lower=0.0, upper=0.012),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_lever = object_model.get_part("speed_lever")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_beater")
    speed_joint = object_model.get_articulation("base_to_speed_lever")
    lock_push = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "requested_kinematic_structure",
        len(object_model.parts) == 6 and len(object_model.articulations) == 5,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.expect_overlap(
        beater,
        bowl,
        axes="xy",
        min_overlap=0.040,
        elem_a="beater_wire",
        elem_b="bowl_shell",
        name="beater centered over bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.002,
        max_gap=0.035,
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        name="rounded head clears bowl rim",
    )
    ctx.expect_contact(
        speed_lever,
        base,
        elem_a="lever_pivot",
        elem_b="rounded_base_plate",
        contact_tol=0.004,
        name="speed lever mounted on base side",
    )
    ctx.expect_contact(
        head_lock,
        base,
        elem_a="lock_bezel",
        elem_b="rear_pedestal",
        contact_tol=0.004,
        name="head lock push control mounted on pedestal side",
    )

    rest_bowl = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.045}):
        extended_bowl = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            min_overlap=0.085,
            elem_a="slide_tongue",
            elem_b="rounded_base_plate",
            name="bowl slide retains carriage insertion",
        )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({head_tilt: 0.80}):
        tilted_head_aabb = ctx.part_world_aabb(head)

    rest_lock = ctx.part_world_position(head_lock)
    with ctx.pose({lock_push: 0.012}):
        pushed_lock = ctx.part_world_position(head_lock)

    ctx.check(
        "bowl carriage slides forward",
        rest_bowl is not None
        and extended_bowl is not None
        and extended_bowl[0] > rest_bowl[0] + 0.035,
        details=f"rest={rest_bowl}, extended={extended_bowl}",
    )
    ctx.check(
        "tilt head lifts upward",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.080,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )
    ctx.check(
        "head lock pushes inward",
        rest_lock is not None
        and pushed_lock is not None
        and pushed_lock[1] > rest_lock[1] + 0.010,
        details=f"rest={rest_lock}, pushed={pushed_lock}",
    )
    ctx.check(
        "control_joint_types",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and beater_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_push.articulation_type == ArticulationType.PRISMATIC,
        details="Expected slide, tilt, spin, speed lever, and push-lock joint types.",
    )

    return ctx.report()


object_model = build_object_model()
