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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arc_points(
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (end_angle - start_angle) * step / segments),
            radius * math.sin(start_angle + (end_angle - start_angle) * step / segments),
        )
        for step in range(segments + 1)
    ]


def _annular_sector_profile(
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    end_angle: float,
    arc_segments: int = 24,
) -> list[tuple[float, float]]:
    outer_arc = _arc_points(outer_radius, start_angle, end_angle, segments=arc_segments)
    inner_arc = _arc_points(inner_radius, end_angle, start_angle, segments=arc_segments)
    return outer_arc + inner_arc


def _sector_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    end_angle: float,
    height: float,
    arc_segments: int = 28,
):
    profile = _annular_sector_profile(
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        start_angle=start_angle,
        end_angle=end_angle,
        arc_segments=arc_segments,
    )
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(profile, height, cap=True, closed=True),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_mantrap_revolving_door")

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.82, 0.90, 0.34))
    floor_stone = model.material("floor_stone", rgba=(0.22, 0.23, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    outer_radius = 1.12
    inner_radius = 1.06
    floor_thickness = 0.035
    canopy_bottom = 2.27
    canopy_thickness = 0.15
    drum_height = canopy_bottom + canopy_thickness
    wall_height = 2.29
    portal_half_angle = math.radians(30.0)
    wall_arc_half_span = math.radians(60.0)

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=outer_radius, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=floor_stone,
        name="floor_disk",
    )
    drum.visual(
        Cylinder(radius=outer_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + canopy_thickness * 0.5)),
        material=brushed_steel,
        name="canopy_disk",
    )
    drum.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=dark_steel,
        name="ceiling_bearing_housing",
    )

    wall_sector_pos = _sector_mesh(
        "drum_side_wall_pos_y",
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        start_angle=(math.pi * 0.5) - wall_arc_half_span,
        end_angle=(math.pi * 0.5) + wall_arc_half_span,
        height=wall_height,
    )
    wall_sector_neg = _sector_mesh(
        "drum_side_wall_neg_y",
        inner_radius=inner_radius,
        outer_radius=outer_radius,
        start_angle=-(math.pi * 0.5) - wall_arc_half_span,
        end_angle=-(math.pi * 0.5) + wall_arc_half_span,
        height=wall_height,
    )
    drum.visual(wall_sector_pos, material=glass, name="side_wall_pos_y")
    drum.visual(wall_sector_neg, material=glass, name="side_wall_neg_y")

    jamb_radius = 0.5 * (inner_radius + outer_radius)
    jamb_angles = (
        portal_half_angle,
        -portal_half_angle,
        math.pi - portal_half_angle,
        -math.pi + portal_half_angle,
    )
    for index, angle in enumerate(jamb_angles):
        drum.visual(
            Box((outer_radius - inner_radius + 0.03, 0.06, wall_height + 0.03)),
            origin=Origin(
                xyz=(
                    jamb_radius * math.cos(angle),
                    jamb_radius * math.sin(angle),
                    0.5 * (wall_height + 0.03),
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"portal_jamb_{index}",
        )

    drum.inertial = Inertial.from_geometry(
        Box((outer_radius * 2.0, outer_radius * 2.0, drum_height)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, drum_height * 0.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="lower_bearing",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=2.18),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=brushed_steel,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 2.155)),
        material=dark_steel,
        name="upper_hub",
    )
    rotor.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.07)),
        material=brushed_steel,
        name="upper_spindle_collar",
    )

    wing_panel_length = 0.95
    wing_glass_thickness = 0.012
    wing_frame_thickness = 0.03
    wing_height = 2.02
    wing_bottom = 0.09
    wing_center_z = wing_bottom + wing_height * 0.5
    rail_height = 0.05
    glass_center_x = 0.555
    inner_stile_center_x = 0.07
    outer_stile_center_x = 1.03
    rail_center_x = 0.56

    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0
        glass_xy = _polar_xy(glass_center_x, angle)
        inner_xy = _polar_xy(inner_stile_center_x, angle)
        outer_xy = _polar_xy(outer_stile_center_x, angle)
        rail_xy = _polar_xy(rail_center_x, angle)
        top_sweep_xy = _polar_xy(0.09, angle)
        rotor.visual(
            Box((wing_panel_length, wing_glass_thickness, wing_height)),
            origin=Origin(
                xyz=(glass_xy[0], glass_xy[1], wing_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=glass,
            name=f"wing_{index}_glass",
        )
        rotor.visual(
            Box((0.04, wing_frame_thickness, wing_height + 0.06)),
            origin=Origin(
                xyz=(inner_xy[0], inner_xy[1], wing_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"wing_{index}_inner_stile",
        )
        rotor.visual(
            Box((0.03, wing_frame_thickness, wing_height + 0.06)),
            origin=Origin(
                xyz=(outer_xy[0], outer_xy[1], wing_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"wing_{index}_outer_stile",
        )
        rotor.visual(
            Box((wing_panel_length, wing_frame_thickness, rail_height)),
            origin=Origin(
                xyz=(rail_xy[0], rail_xy[1], wing_bottom + rail_height * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"wing_{index}_bottom_rail",
        )
        rotor.visual(
            Box((wing_panel_length, wing_frame_thickness, rail_height)),
            origin=Origin(
                xyz=(rail_xy[0], rail_xy[1], wing_bottom + wing_height - rail_height * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"wing_{index}_top_rail",
        )
        rotor.visual(
            Box((0.10, wing_frame_thickness, 0.08)),
            origin=Origin(
                xyz=(top_sweep_xy[0], top_sweep_xy[1], 2.03),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"wing_{index}_top_sweep",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.05, length=2.21),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
    )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("drum_to_rotor")

    ctx.expect_gap(
        rotor,
        drum,
        axis="z",
        positive_elem="lower_bearing",
        negative_elem="floor_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower bearing sits on the floor plane",
    )
    ctx.expect_gap(
        drum,
        rotor,
        axis="z",
        positive_elem="ceiling_bearing_housing",
        negative_elem="upper_hub",
        min_gap=0.008,
        max_gap=0.040,
        name="upper hub clears the ceiling bearing housing",
    )

    with ctx.pose({spin: 0.0}):
        wing_centers = []
        for index in range(3):
            wing_centers.append(
                _aabb_center(ctx.part_element_world_aabb(rotor, elem=f"wing_{index}_outer_stile"))
            )

        tips_present = all(center is not None for center in wing_centers)
        ctx.check(
            "all three wing tips are present",
            tips_present,
            details=f"wing_centers={wing_centers}",
        )

        if tips_present:
            radii = [math.hypot(center[0], center[1]) for center in wing_centers]
            angles = sorted(math.atan2(center[1], center[0]) % (2.0 * math.pi) for center in wing_centers)
            angle_steps = [
                (angles[(index + 1) % 3] - angles[index]) % (2.0 * math.pi)
                for index in range(3)
            ]
            ctx.check(
                "wing tips run close to the drum wall",
                all(0.99 <= radius <= 1.05 for radius in radii),
                details=f"radii={radii}",
            )
            ctx.check(
                "three wings are spaced at equal 120 degree intervals",
                all(abs(step - (2.0 * math.pi / 3.0)) <= 0.08 for step in angle_steps),
                details=f"angles={angles}, angle_steps={angle_steps}",
            )

        rest_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile"))

    with ctx.pose({spin: math.pi / 3.0}):
        turned_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile"))

    rotates_cleanly = (
        rest_tip is not None
        and turned_tip is not None
        and turned_tip[0] < rest_tip[0] - 0.35
        and turned_tip[1] > rest_tip[1] + 0.75
        and abs(math.hypot(turned_tip[0], turned_tip[1]) - math.hypot(rest_tip[0], rest_tip[1])) <= 0.03
    )
    ctx.check(
        "continuous rotation moves the wing assembly around the central post",
        rotates_cleanly,
        details=f"rest_tip={rest_tip}, turned_tip={turned_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
