from __future__ import annotations

from math import cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


FOCUS_ROTATION = 1.5 * pi
FRONT_TRAVEL = 0.024


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_screw_mount_prime")

    model.material("satin_black", rgba=(0.015, 0.016, 0.015, 1.0))
    model.material("rubber_black", rgba=(0.025, 0.025, 0.023, 1.0))
    model.material("aged_aluminum", rgba=(0.72, 0.70, 0.64, 1.0))
    model.material("brass", rgba=(0.76, 0.55, 0.23, 1.0))
    model.material("white_paint", rgba=(0.92, 0.90, 0.82, 1.0))
    model.material("glass_green", rgba=(0.42, 0.72, 0.70, 0.42))
    model.material("dark_glass", rgba=(0.03, 0.06, 0.06, 0.65))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell(
                [(0.0210, -0.019), (0.0210, -0.003)],
                inner_radius=0.0160,
                bevel=0.00035,
            ),
            "rear_screw_mount",
        ),
        material="aged_aluminum",
        name="rear_screw_mount",
    )
    for idx, z in enumerate((-0.0170, -0.0142, -0.0114, -0.0086, -0.0058)):
        barrel.visual(
            mesh_from_geometry(TorusGeometry(0.02095, 0.00045, radial_segments=10, tubular_segments=72), f"screw_thread_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="aged_aluminum",
            name=f"screw_thread_{idx}",
        )
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell(
                [
                    (0.0260, -0.004),
                    (0.0285, 0.000),
                    (0.0285, 0.007),
                    (0.0245, 0.010),
                ],
                inner_radius=0.0165,
                bevel=0.00045,
            ),
            "mount_flange",
        ),
        material="aged_aluminum",
        name="mount_flange",
    )
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell(
                [
                    (0.0202, 0.004),
                    (0.0209, 0.010),
                    (0.0209, 0.066),
                    (0.0197, 0.068),
                ],
                inner_radius=0.0145,
                bevel=0.00035,
            ),
            "fixed_inner_sleeve",
        ),
        material="satin_black",
        name="fixed_inner_sleeve",
    )
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell([(0.0260, 0.0095), (0.0260, 0.0105)], inner_radius=0.02070, bevel=0.0),
            "aperture_bearing",
        ),
        material="satin_black",
        name="aperture_bearing",
    )
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell([(0.0280, 0.0278), (0.0280, 0.0290)], inner_radius=0.02070, bevel=0.0),
            "focus_bearing_root",
        ),
        material="satin_black",
        name="focus_bearing_root",
    )
    barrel.visual(
        Cylinder(radius=0.0140, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, -0.0157)),
        material="dark_glass",
        name="rear_glass",
    )
    barrel.visual(
        mesh_from_geometry(
            _stepped_shell([(0.0164, -0.0172), (0.0164, -0.0142)], inner_radius=0.0137, bevel=0.00025),
            "rear_retaining_ring",
        ),
        material="satin_black",
        name="rear_retaining_ring",
    )
    _add_scale_marks(barrel, z0=0.028, count=7, radius=0.02105, prefix="depth_mark")

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_geometry(
            _stepped_shell(
                [
                    (0.0260, -0.008),
                    (0.0274, -0.006),
                    (0.0274, 0.006),
                    (0.0262, 0.008),
                ],
                inner_radius=0.0222,
                bevel=0.00035,
            ),
            "aperture_shell",
        ),
        material="satin_black",
        name="aperture_shell",
    )
    _add_radial_ribs(aperture_ring, count=28, radius=0.0277, radial_depth=0.0019, width=0.0014, length=0.0125, material="satin_black", prefix="aperture_rib")
    _add_tangent_mark(aperture_ring, x=0.000, z=-0.0046, radius=0.0279, width=0.010, height=0.0009, name="aperture_index")
    for i, x in enumerate((-0.006, -0.003, 0.003, 0.006)):
        _add_tangent_mark(aperture_ring, x=x, z=0.0048, radius=0.0279, width=0.0012, height=0.0017, name=f"aperture_tick_{i}")

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _stepped_shell(
                [
                    (0.0280, -0.019),
                    (0.0304, -0.016),
                    (0.0304, 0.016),
                    (0.0280, 0.019),
                ],
                inner_radius=0.0233,
                bevel=0.00055,
            ),
            "focus_shell",
        ),
        material="rubber_black",
        name="focus_shell",
    )
    _add_radial_ribs(focus_ring, count=44, radius=0.0310, radial_depth=0.0028, width=0.0016, length=0.0325, material="rubber_black", prefix="focus_grip")
    for i, (x, z, width) in enumerate(((-0.010, 0.013, 0.0010), (-0.005, 0.013, 0.0010), (0.000, 0.013, 0.0026), (0.005, 0.013, 0.0010), (0.010, 0.013, 0.0010))):
        _add_tangent_mark(focus_ring, x=x, z=z, radius=0.0318, width=width, height=0.0030, name=f"focus_distance_mark_{i}")

    front_barrel = model.part("front_barrel")
    front_barrel.visual(
        mesh_from_geometry(
            _stepped_shell([(0.0219, -0.032), (0.0219, 0.006)], inner_radius=0.0209, bevel=0.00025),
            "helicoid_sleeve",
        ),
        material="aged_aluminum",
        name="helicoid_sleeve",
    )
    for i, z in enumerate((-0.026, -0.017, -0.008)):
        front_barrel.visual(
            mesh_from_geometry(TorusGeometry(0.0220, 0.00035, radial_segments=8, tubular_segments=72), f"helicoid_thread_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="brass",
            name=f"helicoid_thread_{i}",
        )
    front_barrel.visual(
        Box((0.0020, 0.0008, 0.031)),
        origin=Origin(xyz=(0.0060, -0.0224, -0.011), rpy=(0.0, 0.36, 0.0)),
        material="brass",
        name="helicoid_lead",
    )
    front_barrel.visual(
        mesh_from_geometry(
            _stepped_shell(
                [
                    (0.0238, 0.004),
                    (0.0268, 0.011),
                    (0.0268, 0.036),
                    (0.0242, 0.041),
                    (0.0242, 0.048),
                ],
                inner_radius=0.0182,
                bevel=0.00050,
            ),
            "front_shell",
        ),
        material="satin_black",
        name="front_shell",
    )
    front_barrel.visual(
        mesh_from_geometry(
            _stepped_shell([(0.0204, 0.0405), (0.0204, 0.0475)], inner_radius=0.0168, bevel=0.00025),
            "filter_retainer",
        ),
        material="aged_aluminum",
        name="filter_retainer",
    )
    front_barrel.visual(
        Cylinder(radius=0.0171, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0450)),
        material="glass_green",
        name="front_glass",
    )
    _add_tangent_mark(front_barrel, x=0.000, z=0.034, radius=0.0272, width=0.011, height=0.0009, name="filter_index")

    model.articulation(
        "barrel_to_aperture_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=0.4, velocity=1.0),
    )
    focus_joint = model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=FOCUS_ROTATION, effort=0.8, velocity=1.0),
    )
    model.articulation(
        "barrel_to_front_barrel",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=front_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=FRONT_TRAVEL, effort=15.0, velocity=0.04),
        meta={"helicoid_driver": focus_joint.name, "travel_per_radian": FRONT_TRAVEL / FOCUS_ROTATION},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    front_barrel = object_model.get_part("front_barrel")
    focus_joint = object_model.get_articulation("barrel_to_focus_ring")
    front_slide = object_model.get_articulation("barrel_to_front_barrel")

    ctx.check(
        "focus ring is revolute about the optical axis",
        focus_joint.articulation_type == ArticulationType.REVOLUTE and tuple(focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )
    ctx.check(
        "front barrel slide is indexed to focus throw",
        front_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(front_slide.axis) == (0.0, 0.0, 1.0)
        and front_slide.meta.get("helicoid_driver") == focus_joint.name
        and abs(front_slide.meta.get("travel_per_radian", 0.0) - FRONT_TRAVEL / FOCUS_ROTATION) < 1e-9,
        details=f"type={front_slide.articulation_type}, axis={front_slide.axis}, meta={front_slide.meta}",
    )

    ctx.expect_overlap(
        front_barrel,
        focus_ring,
        axes="z",
        elem_a="helicoid_sleeve",
        elem_b="focus_shell",
        min_overlap=0.025,
        name="collapsed helicoid remains engaged under the focus ring",
    )
    ctx.expect_within(
        front_barrel,
        focus_ring,
        axes="xy",
        inner_elem="helicoid_sleeve",
        outer_elem="focus_shell",
        margin=0.001,
        name="helicoid sleeve stays coaxial inside focus ring envelope",
    )
    ctx.expect_overlap(
        barrel,
        focus_ring,
        axes="z",
        elem_a="fixed_inner_sleeve",
        elem_b="focus_shell",
        min_overlap=0.030,
        name="focus ring is carried on the fixed inner sleeve",
    )

    rest_position = ctx.part_world_position(front_barrel)
    with ctx.pose({focus_joint: FOCUS_ROTATION, front_slide: FRONT_TRAVEL}):
        extended_position = ctx.part_world_position(front_barrel)
        ctx.expect_overlap(
            front_barrel,
            focus_ring,
            axes="z",
            elem_a="helicoid_sleeve",
            elem_b="focus_shell",
            min_overlap=0.006,
            name="extended helicoid still has retained insertion",
        )
    ctx.check(
        "turning the focus ring extends the front barrel",
        rest_position is not None and extended_position is not None and extended_position[2] > rest_position[2] + FRONT_TRAVEL * 0.9,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


def _stepped_shell(
    outer_profile: list[tuple[float, float]],
    *,
    inner_radius: float,
    bevel: float = 0.0004,
    segments: int = 96,
) -> MeshGeometry:
    """Revolved thin-walled vintage lens ring with small softened shoulders."""
    z_min = min(z for _, z in outer_profile)
    z_max = max(z for _, z in outer_profile)
    outer: list[tuple[float, float]] = []
    for radius, z in outer_profile:
        outer.append((max(radius - bevel * 0.15, inner_radius + 0.0004), z))
    inner = [
        (inner_radius, z_min),
        (inner_radius, z_max),
    ]
    return LatheGeometry.from_shell_profiles(outer, inner, segments=segments, start_cap="flat", end_cap="flat", lip_samples=5)


def _add_radial_ribs(
    part,
    *,
    count: int,
    radius: float,
    radial_depth: float,
    width: float,
    length: float,
    material: str,
    prefix: str,
) -> None:
    for i in range(count):
        angle = 2.0 * pi * i / count
        part.visual(
            Box((radial_depth, width, length)),
            origin=Origin(xyz=(radius * cos(angle), radius * sin(angle), 0.0), rpy=(0.0, 0.0, angle)),
            material=material,
            name=f"{prefix}_{i}",
        )


def _add_tangent_mark(part, *, x: float, z: float, radius: float, width: float, height: float, name: str) -> None:
    y = -sqrt(max(radius * radius - x * x, 0.0))
    part.visual(
        Box((width, 0.00135, height)),
        origin=Origin(xyz=(x, y, z)),
        material="white_paint",
        name=name,
    )


def _add_scale_marks(part, *, z0: float, count: int, radius: float, prefix: str) -> None:
    spacing = 0.0042
    start = -0.5 * spacing * (count - 1)
    for i in range(count):
        x = start + spacing * i
        height = 0.0032 if i in (0, count - 1, count // 2) else 0.0018
        _add_tangent_mark(part, x=x, z=z0, radius=radius, width=0.0009, height=height, name=f"{prefix}_{i}")


object_model = build_object_model()
