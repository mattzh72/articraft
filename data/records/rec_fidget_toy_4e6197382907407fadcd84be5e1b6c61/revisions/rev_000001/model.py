from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (cos(2.0 * pi * i / segments) * radius, sin(2.0 * pi * i / segments) * radius)
        for i in range(segments)
    ]


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, height: float, *, segments: int = 96):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            height=height,
            center=True,
        ),
        name,
    )


def _gear_mesh(name: str, *, outer_radius: float, root_radius: float, bore_radius: float, height: float):
    teeth = 18
    outer_samples: list[tuple[float, float, float]] = []
    # Four samples per tooth make a clear squared-off fidget-toy gear silhouette
    # while leaving a little backlash-like valley between neighboring teeth.
    for tooth in range(teeth):
        base = 2.0 * pi * tooth / teeth
        for frac, radius in ((0.00, root_radius), (0.18, outer_radius), (0.50, outer_radius), (0.68, root_radius)):
            angle = base + frac * 2.0 * pi / teeth
            outer_samples.append((angle, cos(angle) * radius, sin(angle) * radius))

    geom = MeshGeometry()
    half_h = height * 0.5
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for angle, x, y in outer_samples:
        outer_bottom.append(geom.add_vertex(x, y, -half_h))
        outer_top.append(geom.add_vertex(x, y, half_h))
        inner_bottom.append(geom.add_vertex(cos(angle) * bore_radius, sin(angle) * bore_radius, -half_h))
        inner_top.append(geom.add_vertex(cos(angle) * bore_radius, sin(angle) * bore_radius, half_h))

    count = len(outer_samples)
    for i in range(count):
        j = (i + 1) % count
        # Outer tooth wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner bore wall, wound inward so the center remains open.
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        # Top and bottom annular faces.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_ring_fidget_toy")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    anodized_blue = model.material("anodized_blue", rgba=(0.08, 0.32, 0.88, 1.0))
    gear_orange = model.material("gear_orange", rgba=(0.95, 0.34, 0.06, 1.0))
    gear_green = model.material("gear_green", rgba=(0.12, 0.74, 0.36, 1.0))
    gear_purple = model.material("gear_purple", rgba=(0.50, 0.20, 0.86, 1.0))

    frame = model.part("frame")

    carrier_thickness = 0.006
    planet_radius = 0.038
    planet_outer = 0.0165
    gear_height = 0.012
    gear_center_z = 0.012
    axle_bore_radius = 0.0058

    frame.visual(
        _annulus_mesh("outer_guard_ring", 0.064, 0.058, carrier_thickness),
        material=matte_black,
        name="outer_guard",
    )
    frame.visual(
        _annulus_mesh("central_axle_ring", 0.018, 0.0115, 0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=anodized_blue,
        name="central_axle_ring",
    )
    frame.visual(
        _annulus_mesh("central_bearing_lip", 0.013, 0.010, 0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=brushed_steel,
        name="central_bearing_lip",
    )

    arm_length = 0.055
    arm_center = 0.028
    planet_positions: list[tuple[float, float]] = []
    for index in range(3):
        angle = 2.0 * pi * index / 3.0
        x = cos(angle) * planet_radius
        y = sin(angle) * planet_radius
        planet_positions.append((x, y))

        frame.visual(
            Box((arm_length, 0.0075, carrier_thickness)),
            origin=Origin(
                xyz=(cos(angle) * arm_center, sin(angle) * arm_center, 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"carrier_arm_{index}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=carrier_thickness),
            origin=Origin(xyz=(x, y, 0.0)),
            material=graphite,
            name=f"axle_boss_{index}",
        )
        frame.visual(
            Cylinder(radius=axle_bore_radius + 0.00002, length=0.023),
            origin=Origin(xyz=(x, y, 0.0095)),
            material=brushed_steel,
            name=f"axle_post_{index}",
        )
        frame.visual(
            Cylinder(radius=0.0074, length=0.003),
            origin=Origin(xyz=(x, y, 0.0225)),
            material=brushed_steel,
            name=f"retainer_cap_{index}",
        )

    gear_meshes = [
        _gear_mesh(
            "planet_gear_0",
            outer_radius=planet_outer,
            root_radius=0.0140,
            bore_radius=axle_bore_radius,
            height=gear_height,
        ),
        _gear_mesh(
            "planet_gear_1",
            outer_radius=planet_outer,
            root_radius=0.0140,
            bore_radius=axle_bore_radius,
            height=gear_height,
        ),
        _gear_mesh(
            "planet_gear_2",
            outer_radius=planet_outer,
            root_radius=0.0140,
            bore_radius=axle_bore_radius,
            height=gear_height,
        ),
    ]
    gear_materials = [gear_orange, gear_green, gear_purple]

    for index, ((x, y), mesh, material) in enumerate(zip(planet_positions, gear_meshes, gear_materials)):
        gear = model.part(f"planet_{index}")
        gear.visual(mesh, material=material, name="gear_body")
        for dot_index in range(3):
            dot_angle = 2.0 * pi * dot_index / 3.0
            dot_xy = (cos(dot_angle) * 0.0091, sin(dot_angle) * 0.0091)
            gear.visual(
                Cylinder(radius=0.00125, length=0.0012),
                origin=Origin(xyz=(dot_xy[0], dot_xy[1], gear_height * 0.5 + 0.00055)),
                material=brushed_steel,
                name=f"bearing_dot_{dot_index}",
            )
        model.articulation(
            f"planet_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=gear,
            origin=Origin(xyz=(x, y, gear_center_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.45, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    planet_parts = [object_model.get_part(f"planet_{index}") for index in range(3)]
    spin_joints = [object_model.get_articulation(f"planet_{index}_spin") for index in range(3)]

    ctx.check("three articulated planetary gears", len(planet_parts) == 3 and len(spin_joints) == 3)
    for index, (planet, joint) in enumerate(zip(planet_parts, spin_joints)):
        ctx.allow_overlap(
            frame,
            planet,
            elem_a=f"axle_post_{index}",
            elem_b="gear_body",
            reason="The fixed metal axle is intentionally captured inside the planet gear's central bore; the gear mesh collision proxy does not subtract that bore.",
        )
        ctx.check(
            f"planet {index} spins about its axle",
            tuple(joint.axis) == (0.0, 0.0, 1.0)
            and str(joint.articulation_type).endswith("CONTINUOUS"),
        )
        ctx.expect_gap(
            planet,
            frame,
            axis="z",
            min_gap=0.002,
            max_gap=0.0045,
            positive_elem="gear_body",
            negative_elem=f"carrier_arm_{index}",
            name=f"planet {index} floats above carrier plate on axle",
        )
        ctx.expect_contact(
            planet,
            frame,
            contact_tol=0.00025,
            elem_a="gear_body",
            elem_b=f"axle_post_{index}",
            name=f"planet {index} bore is captured on axle post",
        )

        rest_pos = ctx.part_world_position(planet)
        with ctx.pose({joint: 1.35}):
            spun_pos = ctx.part_world_position(planet)
        ctx.check(
            f"planet {index} rotates in place",
            rest_pos is not None and spun_pos is not None and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


object_model = build_object_model()
