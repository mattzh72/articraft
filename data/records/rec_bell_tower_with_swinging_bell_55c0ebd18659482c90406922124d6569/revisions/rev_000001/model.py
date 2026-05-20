from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _bell_shell_geometry(mouth_diameter: float, height: float, wall: float) -> LatheGeometry:
    """Thin-walled cast bell profile, with a heavy rounded mouth lip."""
    r = mouth_diameter * 0.5
    top_z = -0.24
    mouth_z = top_z - height
    outer = [
        (0.12 * mouth_diameter, top_z),
        (0.18 * mouth_diameter, top_z - 0.04 * height),
        (0.24 * mouth_diameter, top_z - 0.20 * height),
        (0.30 * mouth_diameter, top_z - 0.46 * height),
        (0.39 * mouth_diameter, top_z - 0.74 * height),
        (r * 0.96, top_z - 0.93 * height),
        (r, mouth_z),
    ]
    inner = [
        (max(0.03, 0.12 * mouth_diameter - wall), top_z - 0.025),
        (max(0.04, 0.18 * mouth_diameter - wall), top_z - 0.08 * height),
        (max(0.05, 0.24 * mouth_diameter - wall), top_z - 0.26 * height),
        (max(0.06, 0.30 * mouth_diameter - wall), top_z - 0.52 * height),
        (max(0.07, 0.39 * mouth_diameter - wall * 1.25), top_z - 0.78 * height),
        (max(0.08, r * 0.82), top_z - 0.94 * height),
        (max(0.08, r - wall * 2.4), mouth_z + 0.012),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _add_course_band(
    tower,
    z: float,
    *,
    material: Material,
    name_prefix: str,
    width: float = 2.08,
    depth: float = 2.08,
) -> None:
    """Four shallow stone-course strips wrapped around the square shaft."""
    tower.visual(
        Box((width + 0.04, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.008, z)),
        material=material,
        name=f"{name_prefix}_front",
    )
    tower.visual(
        Box((width + 0.04, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.008, z)),
        material=material,
        name=f"{name_prefix}_rear",
    )
    tower.visual(
        Box((0.035, depth, 0.045)),
        origin=Origin(xyz=(width * 0.5 + 0.008, 0.0, z)),
        material=material,
        name=f"{name_prefix}_side_0",
    )
    tower.visual(
        Box((0.035, depth, 0.045)),
        origin=Origin(xyz=(-width * 0.5 - 0.008, 0.0, z)),
        material=material,
        name=f"{name_prefix}_side_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carillon_bell_tower")

    stone = model.material("limestone", rgba=(0.60, 0.57, 0.50, 1.0))
    darker_stone = model.material("shadowed_stone", rgba=(0.42, 0.40, 0.36, 1.0))
    slate = model.material("blue_gray_slate", rgba=(0.19, 0.22, 0.26, 1.0))
    oak = model.material("aged_oak", rgba=(0.36, 0.20, 0.10, 1.0))
    iron = model.material("blackened_iron", rgba=(0.04, 0.04, 0.04, 1.0))
    bronze = model.material("aged_bell_bronze", rgba=(0.70, 0.48, 0.22, 1.0))

    tower = model.part("tower")

    # Square stone shaft and base, deliberately sized at small tower scale.
    tower.visual(
        Box((2.55, 2.55, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=darker_stone,
        name="base_plinth",
    )
    tower.visual(
        Box((2.08, 2.08, 4.55)),
        origin=Origin(xyz=(0.0, 0.0, 2.625)),
        material=stone,
        name="square_shaft",
    )
    tower.visual(
        Box((2.35, 2.35, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 4.98)),
        material=darker_stone,
        name="belfry_floor_slab",
    )
    for index, z in enumerate((0.74, 1.42, 2.10, 2.78, 3.46, 4.14)):
        _add_course_band(tower, z, material=darker_stone, name_prefix=f"course_{index}")

    # Open belfry stage: four corner piers, lintels, and beams leave all sides open.
    pier_xy = 0.88
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.32, 0.32, 1.74)),
                origin=Origin(xyz=(sx * pier_xy, sy * pier_xy, 5.91)),
                material=stone,
                name=f"belfry_pier_{sx:+.0f}_{sy:+.0f}".replace("+", "p").replace("-", "m"),
            )
    tower.visual(
        Box((2.25, 0.34, 0.30)),
        origin=Origin(xyz=(0.0, 1.00, 6.86)),
        material=darker_stone,
        name="front_lintel",
    )
    tower.visual(
        Box((2.25, 0.34, 0.30)),
        origin=Origin(xyz=(0.0, -1.00, 6.86)),
        material=darker_stone,
        name="rear_lintel",
    )
    tower.visual(
        Box((0.34, 2.25, 0.30)),
        origin=Origin(xyz=(1.00, 0.0, 6.86)),
        material=darker_stone,
        name="side_lintel_0",
    )
    tower.visual(
        Box((0.34, 2.25, 0.30)),
        origin=Origin(xyz=(-1.00, 0.0, 6.86)),
        material=darker_stone,
        name="side_lintel_1",
    )
    tower.visual(
        Box((2.42, 2.42, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 7.07)),
        material=darker_stone,
        name="top_cornice",
    )
    tower.visual(
        mesh_from_geometry(ConeGeometry(radius=1.46, height=0.92, radial_segments=4), "slate_pyramidal_roof"),
        origin=Origin(xyz=(0.0, 0.0, 7.64), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=slate,
        name="slate_roof",
    )

    # Paired oak belfry beams carry the individual bell bearings.
    for y, name in ((0.66, "front_bell_beam"), (-0.66, "rear_bell_beam")):
        tower.visual(
            Box((2.00, 0.18, 0.20)),
            origin=Origin(xyz=(0.0, y, 6.04)),
            material=oak,
            name=name,
        )
    tower.visual(
        Box((0.18, 1.50, 0.16)),
        origin=Origin(xyz=(-1.06, 0.0, 6.26)),
        material=oak,
        name="cross_tie_0",
    )
    tower.visual(
        Box((0.18, 1.50, 0.16)),
        origin=Origin(xyz=(1.06, 0.0, 6.26)),
        material=oak,
        name="cross_tie_1",
    )

    bell_specs = [
        # (x position, mouth diameter, height)
        (-0.82, 0.44, 0.56),
        (-0.36, 0.36, 0.47),
        (0.02, 0.30, 0.40),
        (0.34, 0.25, 0.34),
        (0.62, 0.20, 0.28),
    ]
    hinge_z = 6.02
    for i, (x, mouth_diameter, height) in enumerate(bell_specs):
        bearing_x = max(0.15, mouth_diameter * 0.54)
        for y, label in ((0.51, "front"), (-0.51, "rear")):
            tower.visual(
                Box((bearing_x, 0.12, 0.16)),
                origin=Origin(xyz=(x, y, hinge_z)),
                material=iron,
                name=f"bearing_{i}_{label}",
            )

        bell = model.part(f"bell_{i}")
        headstock_w = max(0.22, mouth_diameter * 0.78)
        bell.visual(
            Box((headstock_w, 0.68, 0.18)),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=oak,
            name="headstock",
        )
        bell.visual(
            Cylinder(radius=0.035, length=0.98),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name="axle",
        )
        strap_x = max(0.06, mouth_diameter * 0.16)
        for side, sx in enumerate((-strap_x, strap_x)):
            bell.visual(
                Box((0.038, 0.08, 0.22)),
                origin=Origin(xyz=(sx, 0.0, -0.205)),
                material=iron,
                name=f"crown_strap_{side}",
            )
        bell.visual(
            Cylinder(radius=max(0.035, mouth_diameter * 0.105), length=0.13),
            origin=Origin(xyz=(0.0, 0.0, -0.245)),
            material=bronze,
            name="crown_cap",
        )
        bell.visual(
            mesh_from_geometry(
                _bell_shell_geometry(mouth_diameter, height, wall=max(0.018, mouth_diameter * 0.055)),
                f"bell_{i}_hollow_shell",
            ),
            material=bronze,
            name="bell_shell",
        )
        rod_top = -0.275
        ball_z = -0.24 - height + max(0.095, mouth_diameter * 0.36)
        rod_len = max(0.06, abs(ball_z - rod_top))
        bell.visual(
            Cylinder(radius=max(0.006, mouth_diameter * 0.018), length=rod_len),
            origin=Origin(xyz=(0.0, 0.0, (rod_top + ball_z) * 0.5)),
            material=iron,
            name="clapper_stem",
        )
        bell.visual(
            Sphere(radius=max(0.030, mouth_diameter * 0.095)),
            origin=Origin(xyz=(0.0, 0.0, ball_z)),
            material=iron,
            name="clapper_ball",
        )

        model.articulation(
            f"bell_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=bell,
            origin=Origin(xyz=(x, 0.0, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=75.0,
                velocity=1.8,
                lower=-math.radians(28.0),
                upper=math.radians(28.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    joints = [object_model.get_articulation(f"bell_{i}_hinge") for i in range(5)]
    bells = [object_model.get_part(f"bell_{i}") for i in range(5)]

    ctx.check("five independently hinged bells", len(joints) == 5 and len(bells) == 5)
    for i, (joint, bell) in enumerate(zip(joints, bells)):
        ctx.check(
            f"bell_{i} has a horizontal swing axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"bell_{i} has symmetric swing limits",
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper,
        )

        for label in ("front", "rear"):
            ctx.allow_overlap(
                tower,
                bell,
                elem_a=f"bearing_{i}_{label}",
                elem_b="axle",
                reason="The bell axle is intentionally captured inside its iron belfry bearing.",
            )
            ctx.expect_overlap(
                tower,
                bell,
                axes="xyz",
                elem_a=f"bearing_{i}_{label}",
                elem_b="axle",
                min_overlap=0.012,
                name=f"bell_{i} axle passes through {label} bearing",
            )
            ctx.expect_within(
                bell,
                tower,
                axes="xz",
                inner_elem="axle",
                outer_elem=f"bearing_{i}_{label}",
                margin=0.035,
                name=f"bell_{i} axle is centered in {label} bearing",
            )

        rest_aabb = ctx.part_world_aabb(bell)
        with ctx.pose({joint: math.radians(20.0)}):
            swung_aabb = ctx.part_world_aabb(bell)
        if rest_aabb is None or swung_aabb is None:
            ctx.fail(f"bell_{i} swing is measurable", "missing world AABB")
        else:
            rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
            swung_center_x = (swung_aabb[0][0] + swung_aabb[1][0]) * 0.5
            ctx.check(
                f"bell_{i} swings about its own hinge",
                abs(swung_center_x - rest_center_x) > 0.035,
                details=f"rest_x={rest_center_x:.3f}, swung_x={swung_center_x:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
