from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_ceiling_light")

    white_metal = Material("satin_white_metal", rgba=(0.86, 0.85, 0.80, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    frosted_glass = Material("warm_frosted_glass", rgba=(0.92, 0.88, 0.72, 0.42))
    warm_bulb = Material("warm_bulb_glow", rgba=(1.0, 0.86, 0.43, 0.72))
    shadow = Material("shadowed_slots", rgba=(0.05, 0.05, 0.045, 1.0))

    fixture = model.part("fixture")

    # Flat circular plate that sits flush to the ceiling plane at z=0.
    fixture.visual(
        Cylinder(radius=0.255, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=white_metal,
        name="base_plate",
    )
    fixture.visual(
        Cylinder(radius=0.168, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=white_metal,
        name="mounting_collar",
    )

    # Screw heads and slots are slightly embedded into the lower face of the base
    # plate so they read as mounted hardware rather than floating decoration.
    for i, x in enumerate((-0.115, 0.115)):
        fixture.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(x, 0.0, -0.026)),
            material=brushed_steel,
            name=f"screw_head_{i}",
        )
        fixture.visual(
            Box((0.026, 0.004, 0.002)),
            origin=Origin(xyz=(x, 0.0, -0.0285), rpy=(0.0, 0.0, 0.55)),
            material=shadow,
            name=f"screw_slot_{i}",
        )

    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.190, -0.034),
            (0.224, -0.075),
            (0.240, -0.127),
            (0.225, -0.158),
        ],
        [
            (0.168, -0.043),
            (0.199, -0.083),
            (0.213, -0.126),
            (0.202, -0.151),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    fixture.visual(
        mesh_from_geometry(shade_shell, "frosted_shade_shell"),
        material=frosted_glass,
        name="shade_shell",
    )

    bottom_frame = ExtrudeWithHolesGeometry(
        _circle_profile(0.205, 96),
        [rounded_rect_profile(0.235, 0.110, 0.020, corner_segments=8)],
        0.006,
        cap=True,
        center=True,
    )
    fixture.visual(
        mesh_from_geometry(bottom_frame, "bottom_diffuser_frame"),
        origin=Origin(xyz=(0.0, 0.0, -0.163)),
        material=frosted_glass,
        name="bottom_frame",
    )

    # A warm bulb and socket are visible through the frosted shade/access opening.
    fixture.visual(
        Cylinder(radius=0.038, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=brushed_steel,
        name="socket",
    )
    fixture.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=warm_bulb,
        name="bulb_globe",
    )

    hinge_x = -0.1325
    hinge_z = -0.178
    hinge_roll = -math.pi / 2.0

    # Stationary hinge knuckles and leaves fixed to the shade bottom frame.
    for i, y in enumerate((-0.104, 0.104)):
        fixture.visual(
            Box((0.020, 0.048, 0.018)),
            origin=Origin(xyz=(hinge_x - 0.014, y, -0.169)),
            material=white_metal,
            name=f"hinge_leaf_{i}",
        )
        fixture.visual(
            Cylinder(radius=0.007, length=0.045),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(hinge_roll, 0.0, 0.0)),
            material=brushed_steel,
            name=f"fixed_knuckle_{i}",
        )

    fixture.visual(
        Box((0.024, 0.034, 0.010)),
        origin=Origin(xyz=(0.153, 0.0, -0.168)),
        material=brushed_steel,
        name="latch_catch",
    )

    door = model.part("access_door")
    door_panel = ExtrudeGeometry(
        rounded_rect_profile(0.265, 0.150, 0.020, corner_segments=8),
        0.006,
        cap=True,
        center=True,
    )
    door.visual(
        mesh_from_geometry(door_panel, "access_door_panel"),
        origin=Origin(xyz=(0.1325, 0.0, 0.009)),
        material=frosted_glass,
        name="door_panel",
    )
    door.visual(
        Box((0.026, 0.120, 0.004)),
        origin=Origin(xyz=(0.014, 0.0, 0.005)),
        material=white_metal,
        name="moving_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(hinge_roll, 0.0, 0.0)),
        material=brushed_steel,
        name="moving_knuckle",
    )
    door.visual(
        Box((0.032, 0.045, 0.010)),
        origin=Origin(xyz=(0.245, 0.0, 0.003)),
        material=brushed_steel,
        name="pull_tab",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture = object_model.get_part("fixture")
    door = object_model.get_part("access_door")
    hinge = object_model.get_articulation("door_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            fixture,
            door,
            axis="z",
            positive_elem="bottom_frame",
            negative_elem="door_panel",
            max_gap=0.001,
            max_penetration=0.00001,
            name="closed access door sits flush under shade bottom",
        )
        ctx.expect_within(
            door,
            fixture,
            axes="xy",
            inner_elem="door_panel",
            outer_elem="bottom_frame",
            margin=0.0,
            name="closed door is contained within round bottom frame",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "hinged access door swings downward for bulb replacement",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
