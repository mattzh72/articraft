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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bullet_personal_blender")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.81, 0.86, 0.48))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.63, 0.65, 0.68, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def blade_wing_mesh(name: str, span: float, chord: float, thickness: float, tip_lift: float):
        def blade_section(x: float, chord_scale: float, thickness_scale: float, z_bias: float):
            c = chord * chord_scale
            t = thickness * thickness_scale
            return [
                (x, -0.26 * c, -0.45 * t + z_bias),
                (x, 0.20 * c, -0.12 * t + z_bias),
                (x, 0.08 * c, 0.80 * t + z_bias),
                (x, -0.10 * c, 0.28 * t + z_bias),
            ]

        blade_geom = section_loft(
            [
                blade_section(-0.5 * span, 1.0, 1.0, 0.0),
                blade_section(0.0, 0.92, 0.95, 0.0012),
                blade_section(0.5 * span, 0.46, 0.65, tip_lift),
            ]
        )
        return save_mesh(name, blade_geom)

    base = model.part("base")
    base_body = LatheGeometry(
        [
            (0.0, 0.000),
            (0.060, 0.000),
            (0.074, 0.006),
            (0.084, 0.022),
            (0.086, 0.050),
            (0.082, 0.078),
            (0.072, 0.092),
            (0.060, 0.098),
            (0.0, 0.098),
        ],
        segments=72,
    )
    base.visual(
        save_mesh("blender_base_body", base_body),
        material=matte_black,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.083, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_graphite,
        name="rubber_foot_ring",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=brushed_aluminum,
        name="cup_seat",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=dark_graphite,
        name="drive_nozzle",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.118),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    cup = model.part("cup")
    cup_shell = LatheGeometry.from_shell_profiles(
        [
            (0.058, 0.000),
            (0.058, 0.018),
            (0.056, 0.090),
            (0.053, 0.168),
            (0.051, 0.196),
            (0.046, 0.212),
            (0.0, 0.214),
        ],
        [
            (0.051, 0.004),
            (0.051, 0.020),
            (0.049, 0.089),
            (0.046, 0.166),
            (0.043, 0.190),
            (0.030, 0.203),
            (0.0, 0.206),
        ],
        segments=72,
    )
    cup.visual(
        save_mesh("blender_cup_shell", cup_shell),
        material=clear_smoke,
        name="cup_shell",
    )
    cup.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.209)),
        material=clear_smoke,
        name="top_puck",
    )
    cup.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.056, 0.0, 0.016)),
        material=clear_smoke,
        name="twist_lug_pos",
    )
    cup.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(-0.056, 0.0, 0.016)),
        material=clear_smoke,
        name="twist_lug_neg",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.214),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_graphite,
        name="blade_shaft",
    )
    blade_assembly.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed_aluminum,
        name="blade_hub",
    )
    blade_assembly.visual(
        blade_wing_mesh("blade_wing_a", 0.064, 0.016, 0.0032, 0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.039), rpy=(0.0, 0.40, math.radians(18.0))),
        material=steel,
        name="blade_span_a",
    )
    blade_assembly.visual(
        blade_wing_mesh("blade_wing_b", 0.060, 0.015, 0.0030, 0.0028),
        origin=Origin(xyz=(0.0, 0.0, 0.039), rpy=(0.0, -0.38, math.radians(108.0))),
        material=steel,
        name="blade_span_b",
    )
    blade_assembly.visual(
        Cylinder(radius=0.008, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_graphite,
        name="blade_retainer",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.050),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "base_to_cup",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=30.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cup = object_model.get_part("cup")
    blade_assembly = object_model.get_part("blade_assembly")
    cup_joint = object_model.get_articulation("base_to_cup")
    blade_joint = object_model.get_articulation("base_to_blade")

    ctx.check(
        "cup uses revolute screw joint",
        cup_joint.articulation_type == ArticulationType.REVOLUTE and cup_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={cup_joint.articulation_type}, axis={cup_joint.axis}",
    )
    ctx.check(
        "blade assembly spins continuously about vertical axis",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS and blade_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={blade_joint.articulation_type}, axis={blade_joint.axis}",
    )

    with ctx.pose({cup_joint: 0.0, blade_joint: 0.0}):
        ctx.expect_contact(
            cup,
            base,
            elem_a="cup_shell",
            elem_b="cup_seat",
            contact_tol=0.0025,
            name="cup rim seats on the base collar",
        )
        ctx.expect_overlap(
            cup,
            base,
            axes="xy",
            elem_a="cup_shell",
            elem_b="cup_seat",
            min_overlap=0.100,
            name="cup remains centered over the compact base",
        )
        ctx.expect_within(
            blade_assembly,
            cup,
            axes="xy",
            outer_elem="cup_shell",
            margin=0.024,
            name="blade span stays within cup diameter",
        )
        ctx.expect_gap(
            cup,
            base,
            axis="z",
            positive_elem="cup_shell",
            negative_elem="cup_seat",
            max_gap=0.003,
            max_penetration=0.001,
            name="cup shell sits just above the base seat",
        )

    blade_rest = ctx.part_world_position(blade_assembly)
    with ctx.pose({blade_joint: math.tau / 4.0}):
        blade_spun = ctx.part_world_position(blade_assembly)
        ctx.expect_within(
            blade_assembly,
            cup,
            axes="xy",
            outer_elem="cup_shell",
            margin=0.024,
            name="blade remains centered while spinning",
        )
    ctx.check(
        "blade spins in place",
        blade_rest is not None
        and blade_spun is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(blade_rest, blade_spun)),
        details=f"rest={blade_rest}, spun={blade_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
