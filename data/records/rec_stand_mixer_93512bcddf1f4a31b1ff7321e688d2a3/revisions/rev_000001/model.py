from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boutique_stand_mixer")

    glossy_paint = model.material("glossy_paint", rgba=(0.93, 0.55, 0.50, 1.0))
    cream_trim = model.material("cream_trim", rgba=(0.96, 0.93, 0.88, 1.0))
    bright_bowl = model.material("bright_bowl", rgba=(0.30, 0.83, 0.88, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.87, 0.89, 0.92, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.20, 0.20, 0.22, 1.0))

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]

    model.meta["style"] = "rounded boutique tilt-head stand mixer"

    base = model.part("base")

    base.visual(
        Box((0.36, 0.23, 0.026)),
        origin=Origin(xyz=(0.01, 0.0, 0.013)),
        material=cream_trim,
        name="foot",
    )
    base.visual(
        Box((0.20, 0.14, 0.012)),
        origin=Origin(xyz=(0.155, 0.0, 0.028)),
        material=glossy_paint,
        name="carriage_deck",
    )
    pedestal_mesh = mesh_from_geometry(
        repair_loft(
            [
                yz_section(0.13, 0.23, 0.032, -0.085, 0.140),
                yz_section(0.12, 0.29, 0.040, -0.048, 0.175),
                yz_section(0.10, 0.25, 0.034, -0.012, 0.160),
                yz_section(0.07, 0.15, 0.022, 0.018, 0.112),
            ],
            repair="mesh",
        ),
        "base_pedestal",
    )
    base.visual(
        pedestal_mesh,
        material=glossy_paint,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.108),
        origin=Origin(xyz=(-0.018, 0.0, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="hinge_barrel",
    )
    base.visual(
        Box((0.040, 0.006, 0.018)),
        origin=Origin(xyz=(-0.040, -0.063, 0.250)),
        material=satin_dark,
        name="lock_slot_bezel",
    )
    base.visual(
        Box((0.054, 0.102, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, 0.239)),
        material=cream_trim,
        name="head_rest_pad",
    )
    base.visual(
        Box((0.054, 0.012, 0.046)),
        origin=Origin(xyz=(-0.024, 0.065, 0.156)),
        material=cream_trim,
        name="speed_mount_boss",
    )
    base.visual(
        Box((0.092, 0.014, 0.010)),
        origin=Origin(xyz=(0.060, 0.052, 0.025)),
        material=glossy_paint,
        name="left_carriage_rib",
    )
    base.visual(
        Box((0.092, 0.014, 0.010)),
        origin=Origin(xyz=(0.060, -0.052, 0.025)),
        material=glossy_paint,
        name="right_carriage_rib",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.23, 0.33)),
        mass=7.2,
        origin=Origin(xyz=(0.00, 0.0, 0.145)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.18, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cream_trim,
        name="carriage_plate",
    )
    bowl.visual(
        Box((0.06, 0.06, 0.010)),
        origin=Origin(xyz=(-0.110, 0.0, 0.005)),
        material=cream_trim,
        name="carriage_tongue",
    )
    bowl.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cream_trim,
        name="bowl_pedestal",
    )
    bowl_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.018, 0.000),
                (0.040, 0.006),
                (0.078, 0.040),
                (0.094, 0.100),
                (0.100, 0.132),
                (0.106, 0.140),
            ],
            [
                (0.000, 0.004),
                (0.036, 0.012),
                (0.071, 0.040),
                (0.088, 0.100),
                (0.095, 0.136),
            ],
            segments=56,
            end_cap="round",
            lip_samples=8,
        ),
        "mixing_bowl",
    )
    bowl.visual(
        bowl_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bright_bowl,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.160, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=0.0,
            upper=0.045,
        ),
    )

    head = model.part("head")
    head_shell = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.09, 0.12, 0.028, 0.055, 0.040),
                yz_section(0.15, 0.16, 0.050, 0.140, 0.032),
                yz_section(0.12, 0.14, 0.040, 0.245, 0.015),
            ]
        ),
        "head_shell",
    )
    head.visual(
        head_shell,
        material=glossy_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.036, 0.080, 0.014)),
        origin=Origin(xyz=(0.058, 0.0, -0.049)),
        material=cream_trim,
        name="head_rest_shoe",
    )
    head.visual(
        Box((0.030, 0.058, 0.034)),
        origin=Origin(xyz=(0.053, 0.0, -0.025)),
        material=glossy_paint,
        name="head_rest_web",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.072),
        origin=Origin(xyz=(0.180, 0.0, -0.036)),
        material=polished_steel,
        name="attachment_nozzle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.28, 0.16, 0.18)),
        mass=3.6,
        origin=Origin(xyz=(0.150, 0.0, 0.020)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.018, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.005, height=0.044).translate(0.0, 0.0, -0.022)
    whisk_geom.merge(CylinderGeometry(radius=0.009, height=0.026).translate(0.0, 0.0, -0.057))
    whisk_geom.merge(CylinderGeometry(radius=0.0125, height=0.014).translate(0.0, 0.0, -0.077))
    whisk_geom.merge(CylinderGeometry(radius=0.004, height=0.010).translate(0.0, 0.0, -0.151))
    for index in range(8):
        angle = index * math.pi / 8.0
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.0045 * cos_a, 0.0045 * sin_a, -0.045),
                    (0.015 * cos_a, 0.015 * sin_a, -0.066),
                    (0.028 * cos_a, 0.028 * sin_a, -0.101),
                    (0.038 * cos_a, 0.038 * sin_a, -0.132),
                    (0.0, 0.0, -0.151),
                    (-0.038 * cos_a, -0.038 * sin_a, -0.132),
                    (-0.028 * cos_a, -0.028 * sin_a, -0.101),
                    (-0.015 * cos_a, -0.015 * sin_a, -0.066),
                    (-0.0045 * cos_a, -0.0045 * sin_a, -0.045),
                ],
                radius=0.0014,
                samples_per_segment=18,
                radial_segments=14,
            )
        )
    whisk.visual(
        mesh_from_geometry(whisk_geom, "balloon_whisk"),
        material=polished_steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.16),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.180, 0.0, -0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=polished_steel,
        name="dial_body",
    )
    speed_control.visual(
        Box((0.018, 0.006, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.015)),
        material=satin_dark,
        name="dial_handle",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.036, 0.020, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.018, 0.071, 0.155), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.2,
            lower=-0.90,
            upper=0.90,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=polished_steel,
        name="lock_thumb",
    )
    head_lock.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=satin_dark,
        name="lock_stem",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.030, 0.018, 0.012)),
        mass=0.04,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.058, -0.075, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    articulation_contract = {
        "base_to_bowl": ArticulationType.PRISMATIC,
        "base_to_head": ArticulationType.REVOLUTE,
        "head_to_whisk": ArticulationType.CONTINUOUS,
        "base_to_speed_control": ArticulationType.REVOLUTE,
        "base_to_head_lock": ArticulationType.PRISMATIC,
    }
    for joint_name, expected_type in articulation_contract.items():
        articulation = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} uses the requested articulation type",
            articulation.articulation_type == expected_type,
            details=f"expected={expected_type}, got={articulation.articulation_type}",
        )

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_hinge = object_model.get_articulation("base_to_head")
    head_lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="carriage_deck",
        max_gap=0.003,
        max_penetration=0.0,
        name="bowl carriage rides just above the base deck",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_shell",
        outer_elem="bowl_shell",
        margin=0.020,
        name="whisk footprint stays inside the bowl rim",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.030,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        whisk_raised = ctx.part_world_position(whisk)
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.030,
            name="raised head clears the bowl vertically",
        )
    ctx.check(
        "head hinge lifts the whisk upward",
        whisk_rest is not None
        and whisk_raised is not None
        and whisk_raised[2] > whisk_rest[2] + 0.080,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({head_lock_slide: head_lock_slide.motion_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider moves forward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.008,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    ctx.check(
        "controls remain base-mounted",
        speed_control in [object_model.get_part("speed_control")]
        and head_lock in [object_model.get_part("head_lock")]
        and object_model.get_articulation("base_to_speed_control").parent == base.name
        and object_model.get_articulation("base_to_head_lock").parent == base.name,
        details="Both control articulations should parent from the base part.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
