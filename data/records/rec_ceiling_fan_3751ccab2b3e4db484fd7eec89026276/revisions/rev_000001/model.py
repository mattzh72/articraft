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
    mesh_from_cadquery,
)


HOUSING_CENTER_Z = -0.32
HOUSING_RADIUS = 0.155
HINGE_RADIUS = 0.225
HINGE_Z = -0.035
BLADE_OPEN_ANGLE = math.pi / 2.0


def _motor_housing_mesh():
    """Rounded drum and shallow caps for the rotating motor housing."""
    main_drum = cq.Workplane("XY").cylinder(0.135, HOUSING_RADIUS)
    top_cap = cq.Workplane("XY").cylinder(0.042, 0.112).translate((0.0, 0.0, 0.062))
    bottom_cap = cq.Workplane("XY").cylinder(0.035, 0.128).translate((0.0, 0.0, -0.072))
    return main_drum.union(top_cap).union(bottom_cap).edges().fillet(0.006)


def _folding_blade_mesh():
    """Thin swept aluminium blade with a round vertical root hinge barrel."""
    blade_thickness = 0.006
    root_barrel_height = 0.035
    outline = [
        (-0.038, -0.020),
        (-0.050, 0.055),
        (-0.040, 0.230),
        (-0.010, 0.500),
        (0.018, 0.635),
        (0.080, 0.660),
        (0.118, 0.500),
        (0.105, 0.185),
        (0.066, -0.030),
    ]
    panel = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(blade_thickness)
        .translate((0.0, 0.0, -blade_thickness / 2.0))
    )
    root_barrel = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(root_barrel_height)
        .translate((0.0, 0.0, -root_barrel_height / 2.0))
    )
    raised_spine = cq.Workplane("XY").box(0.012, 0.500, 0.004).translate((0.030, 0.300, 0.004))
    return panel.union(root_barrel).union(raised_spine)


def _bearing_ring_mesh():
    """Annular top bearing ring with a real clearance hole around the downrod."""
    return cq.Workplane("XY").circle(0.036).circle(0.024).extrude(0.012).translate((0.0, 0.0, -0.006))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_blade_ceiling_fan")

    brushed_aluminium = Material("brushed_aluminium", rgba=(0.74, 0.76, 0.76, 1.0))
    satin_white = Material("satin_white", rgba=(0.86, 0.86, 0.82, 1.0))
    dark_gasket = Material("dark_gasket", rgba=(0.04, 0.045, 0.05, 1.0))
    shadow_grey = Material("shadow_grey", rgba=(0.16, 0.17, 0.18, 1.0))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.105, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_white,
        name="ceiling_canopy",
    )
    mount.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=satin_white,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=dark_gasket,
        name="axle_pin",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_motor_housing_mesh(), "motor_shell"),
        material=satin_white,
        name="motor_shell",
    )
    housing.visual(
        mesh_from_cadquery(_bearing_ring_mesh(), "top_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=shadow_grey,
        name="top_bearing_ring",
    )
    housing.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
        material=shadow_grey,
        name="lower_spindle_cap",
    )

    for i in range(4):
        angle = i * math.tau / 4.0
        radial_center = 0.192
        housing.visual(
            Box((0.075, 0.050, 0.020)),
            origin=Origin(
                xyz=(
                    radial_center * math.cos(angle),
                    radial_center * math.sin(angle),
                    HINGE_Z - 0.0275,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=shadow_grey,
            name=f"hinge_pad_{i}",
        )

    model.articulation(
        "mount_to_housing",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    blade_mesh = _folding_blade_mesh()
    for i in range(4):
        angle = i * math.tau / 4.0
        blade = model.part(f"blade_{i}")
        blade.visual(
            mesh_from_cadquery(blade_mesh, f"aluminium_blade_{i}"),
            material=brushed_aluminium,
            name="blade_shell",
        )
        model.articulation(
            f"housing_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=blade,
            origin=Origin(
                xyz=(
                    HINGE_RADIUS * math.cos(angle),
                    HINGE_RADIUS * math.sin(angle),
                    HINGE_Z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=BLADE_OPEN_ANGLE),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount = object_model.get_part("mount")
    housing = object_model.get_part("housing")
    spin_joint = object_model.get_articulation("mount_to_housing")

    ctx.allow_overlap(
        mount,
        housing,
        elem_a="axle_pin",
        elem_b="motor_shell",
        reason="The stationary central axle is intentionally captured inside the simplified solid motor-housing proxy.",
    )
    ctx.expect_within(
        mount,
        housing,
        axes="xy",
        inner_elem="axle_pin",
        outer_elem="motor_shell",
        margin=0.001,
        name="central axle stays inside housing footprint",
    )
    ctx.expect_overlap(
        mount,
        housing,
        axes="z",
        elem_a="axle_pin",
        elem_b="motor_shell",
        min_overlap=0.10,
        name="central axle passes through motor housing",
    )

    ctx.check(
        "housing has central spin axle",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(spin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin_joint.articulation_type}, axis={spin_joint.axis}",
    )

    for i in range(4):
        blade = object_model.get_part(f"blade_{i}")
        joint = object_model.get_articulation(f"housing_to_blade_{i}")
        ctx.check(
            f"blade {i} folds on root hinge",
            joint.motion_limits is not None
            and abs((joint.motion_limits.lower or 0.0) - 0.0) < 1e-9
            and abs((joint.motion_limits.upper or 0.0) - BLADE_OPEN_ANGLE) < 1e-9
            and tuple(joint.axis) == (0.0, 0.0, -1.0),
            details=f"limits={joint.motion_limits}, axis={joint.axis}",
        )
        ctx.expect_contact(
            blade,
            housing,
            elem_a="blade_shell",
            elem_b=f"hinge_pad_{i}",
            contact_tol=0.003,
            name=f"blade {i} root barrel sits on hinge pad",
        )

    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    blade_0_joint = object_model.get_articulation("housing_to_blade_0")
    blade_1_joint = object_model.get_articulation("housing_to_blade_1")
    folded_0 = ctx.part_element_world_aabb(blade_0, elem="blade_shell")
    folded_1 = ctx.part_element_world_aabb(blade_1, elem="blade_shell")
    with ctx.pose({blade_0_joint: BLADE_OPEN_ANGLE, blade_1_joint: BLADE_OPEN_ANGLE}):
        extended_0 = ctx.part_element_world_aabb(blade_0, elem="blade_shell")
        extended_1 = ctx.part_element_world_aabb(blade_1, elem="blade_shell")

    ctx.check(
        "blade 0 extends outward from stored tangent",
        folded_0 is not None and extended_0 is not None and extended_0[1][0] > folded_0[1][0] + 0.40,
        details=f"folded={folded_0}, extended={extended_0}",
    )
    ctx.check(
        "blade 1 extends outward from stored tangent",
        folded_1 is not None and extended_1 is not None and extended_1[1][1] > folded_1[1][1] + 0.40,
        details=f"folded={folded_1}, extended={extended_1}",
    )

    return ctx.report()


object_model = build_object_model()
