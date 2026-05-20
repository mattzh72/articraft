from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
    segments: int = 64,
) -> MeshGeometry:
    """Create a vertical annular band with a real open bore."""
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.006,
        radial_segments=segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _merged_mesh(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    frame_blue = model.material("frame_blue", rgba=(0.12, 0.25, 0.46, 1.0))
    matte_black = model.material("matte_black", rgba=(0.04, 0.04, 0.045, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.73, 0.16, 1.0))
    concrete = model.material("concrete", rgba=(0.55, 0.55, 0.53, 1.0))

    fixed_frame = model.part("fixed_frame")

    # Ground sill and slim rectangular cage keep the fixed support visually light
    # compared with the sweeping rotor.
    fixed_frame.visual(
        Box((2.10, 1.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="floor_plate",
    )
    fixed_frame.visual(
        Cylinder(radius=0.13, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=dark_steel,
        name="base_pedestal",
    )
    fixed_frame.visual(
        Cylinder(radius=0.075, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=frame_blue,
        name="lower_column",
    )
    fixed_frame.visual(
        Cylinder(radius=0.035, length=1.54),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=brushed_steel,
        name="center_shaft",
    )
    fixed_frame.visual(
        Cylinder(radius=0.082, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.8575)),
        material=dark_steel,
        name="lower_bearing",
    )
    fixed_frame.visual(
        Cylinder(radius=0.082, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 1.2425)),
        material=dark_steel,
        name="upper_bearing",
    )

    post_x = 0.98
    post_y = 0.86
    for ix, x in enumerate((-post_x, post_x)):
        for iy, y in enumerate((-post_y, post_y)):
            fixed_frame.visual(
                Cylinder(radius=0.035, length=1.70),
                origin=Origin(xyz=(x, y, 0.89)),
                material=frame_blue,
                name=f"corner_post_{ix}_{iy}",
            )

    for z, rail_name in ((0.68, "lower_guard"), (1.42, "upper_guard"), (1.72, "top_rail")):
        rail_thickness = 0.055 if z < 1.7 else 0.065
        fixed_frame.visual(
            Box((2.00, rail_thickness, rail_thickness)),
            origin=Origin(xyz=(0.0, post_y, z)),
            material=frame_blue,
            name=f"{rail_name}_front",
        )
        fixed_frame.visual(
            Box((2.00, rail_thickness, rail_thickness)),
            origin=Origin(xyz=(0.0, -post_y, z)),
            material=frame_blue,
            name=f"{rail_name}_rear",
        )
        fixed_frame.visual(
            Box((rail_thickness, 1.78, rail_thickness)),
            origin=Origin(xyz=(post_x, 0.0, z)),
            material=frame_blue,
            name=f"{rail_name}_side_0",
        )
        fixed_frame.visual(
            Box((rail_thickness, 1.78, rail_thickness)),
            origin=Origin(xyz=(-post_x, 0.0, z)),
            material=frame_blue,
            name=f"{rail_name}_side_1",
        )

    # A high bridge ties the bearing shaft to the surrounding frame without
    # interfering with the rotor sweep.
    fixed_frame.visual(
        Box((0.10, 1.78, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        material=frame_blue,
        name="center_bridge",
    )
    fixed_frame.visual(
        Cylinder(radius=0.095, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.67)),
        material=dark_steel,
        name="top_boss",
    )

    rotor = model.part("rotor")
    hub_mesh = mesh_from_geometry(
        _merged_mesh(
            [
                _ring_band(outer_radius=0.125, inner_radius=0.055, height=0.27),
                _ring_band(outer_radius=0.152, inner_radius=0.055, height=0.045, z_center=-0.135),
                _ring_band(outer_radius=0.152, inner_radius=0.055, height=0.045, z_center=0.135),
            ]
        ),
        "turnstile_rotor_hub",
    )
    rotor.visual(hub_mesh, material=dark_steel, name="hub")

    arm_radius = 0.034
    inner_radius = 0.105
    outer_radius = 0.82
    arm_length = outer_radius - inner_radius
    arm_center_radius = inner_radius + arm_length * 0.5
    for index in range(3):
        angle = index * math.tau / 3.0
        x = arm_center_radius * math.cos(angle)
        y = arm_center_radius * math.sin(angle)
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, math.pi / 2.0, angle)),
            material=brushed_steel,
            name=f"arm_{index}",
        )
        end_x = outer_radius * math.cos(angle)
        end_y = outer_radius * math.sin(angle)
        rotor.visual(
            Sphere(radius=0.050),
            origin=Origin(xyz=(end_x, end_y, 0.0)),
            material=safety_yellow,
            name=f"arm_tip_{index}",
        )
        grip_x = (outer_radius - 0.12) * math.cos(angle)
        grip_y = (outer_radius - 0.12) * math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.045, length=0.16),
            origin=Origin(xyz=(grip_x, grip_y, 0.0), rpy=(0.0, math.pi / 2.0, angle)),
            material=matte_black,
            name=f"grip_{index}",
        )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=fixed_frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    rotor = object_model.get_part("rotor")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "hub spins continuously about vertical axis",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(hub_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
    )
    ctx.expect_within(
        rotor,
        fixed_frame,
        axes="xy",
        margin=0.0,
        name="rotor sweep remains inside fixed frame footprint",
    )
    ctx.expect_overlap(
        rotor,
        fixed_frame,
        axes="z",
        elem_a="hub",
        elem_b="center_shaft",
        min_overlap=0.20,
        name="hub surrounds the central support shaft height",
    )
    ctx.expect_contact(
        rotor,
        fixed_frame,
        elem_a="hub",
        elem_b="lower_bearing",
        contact_tol=0.001,
        name="lower bearing supports rotating hub",
    )
    ctx.expect_contact(
        rotor,
        fixed_frame,
        elem_a="hub",
        elem_b="upper_bearing",
        contact_tol=0.001,
        name="upper bearing retains rotating hub",
    )

    return ctx.report()


object_model = build_object_model()
