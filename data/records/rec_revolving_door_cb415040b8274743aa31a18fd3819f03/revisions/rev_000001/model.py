from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * cos(angle), radius * sin(angle), z)


def _sector_loop(
    inner_radius: float,
    outer_radius: float,
    z: float,
    start_angle: float,
    end_angle: float,
    *,
    arc_samples: int = 20,
) -> list[tuple[float, float, float]]:
    outer = [
        _polar(
            outer_radius,
            start_angle + (end_angle - start_angle) * i / arc_samples,
            z,
        )
        for i in range(arc_samples + 1)
    ]
    inner = [
        _polar(
            inner_radius,
            end_angle - (end_angle - start_angle) * i / arc_samples,
            z,
        )
        for i in range(arc_samples + 1)
    ]
    return outer + inner


def _curved_wall_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    bottom_z: float,
    top_z: float,
    start_angle: float,
    end_angle: float,
    arc_samples: int = 20,
):
    return section_loft(
        [
            _sector_loop(
                inner_radius,
                outer_radius,
                bottom_z,
                start_angle,
                end_angle,
                arc_samples=arc_samples,
            ),
            _sector_loop(
                inner_radius,
                outer_radius,
                top_z,
                start_angle,
                end_angle,
                arc_samples=arc_samples,
            ),
        ]
    )


def _radial_box_origin(radius: float, angle: float, z: float) -> Origin:
    return Origin(
        xyz=(radius * cos(angle), radius * sin(angle), z),
        rpy=(0.0, 0.0, angle + pi / 2.0),
    )


def _add_rotor_wing(
    rotor,
    *,
    angle: float,
    inner_stile_center: float,
    outer_stile_center: float,
    inner_stile_width: float,
    outer_stile_width: float,
    panel_thickness: float,
    wing_bottom: float,
    wing_height: float,
    top_rail_height: float,
    bottom_rail_height: float,
    glass_material,
    frame_material,
    kick_material,
    index: int,
) -> None:
    wing_mid_z = wing_bottom + wing_height * 0.5
    inner_outer_face = inner_stile_center + inner_stile_width * 0.5
    outer_inner_face = outer_stile_center - outer_stile_width * 0.5
    rail_span = outer_inner_face - inner_outer_face
    rail_center_x = inner_outer_face + rail_span * 0.5

    glass_margin = 0.02
    glass_span = rail_span - glass_margin * 2.0
    glass_center_x = rail_center_x
    glass_bottom = wing_bottom + bottom_rail_height
    glass_top = wing_bottom + wing_height - top_rail_height
    glass_height = glass_top - glass_bottom
    glass_mid_z = glass_bottom + glass_height * 0.5

    origin_xy = lambda radial_x, z_val: Origin(
        xyz=(radial_x * cos(angle), radial_x * sin(angle), z_val),
        rpy=(0.0, 0.0, angle),
    )

    rotor.visual(
        Box((inner_stile_width, panel_thickness, wing_height)),
        origin=origin_xy(inner_stile_center, wing_mid_z),
        material=frame_material,
        name=f"wing_{index}_inner_stile",
    )
    rotor.visual(
        Box((outer_stile_width, panel_thickness, wing_height)),
        origin=origin_xy(outer_stile_center, wing_mid_z),
        material=frame_material,
        name=f"wing_{index}_outer_stile",
    )
    rotor.visual(
        Box((rail_span, panel_thickness, top_rail_height)),
        origin=origin_xy(
            rail_center_x,
            wing_bottom + wing_height - top_rail_height * 0.5,
        ),
        material=frame_material,
        name=f"wing_{index}_top_rail",
    )
    rotor.visual(
        Box((rail_span, panel_thickness, bottom_rail_height)),
        origin=origin_xy(
            rail_center_x,
            wing_bottom + bottom_rail_height * 0.5,
        ),
        material=kick_material,
        name=f"wing_{index}_bottom_rail",
    )
    rotor.visual(
        Box((glass_span, 0.012, glass_height)),
        origin=origin_xy(glass_center_x, glass_mid_z),
        material=glass_material,
        name=f"wing_{index}_glass",
    )
    rotor.visual(
        Box((0.05, 0.03, wing_height)),
        origin=origin_xy(inner_stile_center - inner_stile_width * 0.5 + 0.025, wing_mid_z),
        material=frame_material,
        name=f"wing_{index}_post_bracket",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_three_wing_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.32, 0.35, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.82, 0.90, 0.28))
    smoked_glass = model.material("smoked_glass", rgba=(0.60, 0.72, 0.82, 0.38))
    floor_finish = model.material("floor_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    kick_plate = model.material("kick_plate", rgba=(0.52, 0.54, 0.57, 1.0))

    drum_radius = 1.58
    wall_inner_radius = 1.53
    wall_outer_radius = 1.55
    floor_thickness = 0.05
    canopy_bottom = 2.32
    canopy_thickness = 0.18
    door_opening_half_angle = 0.58
    side_sector_half_angle = pi / 2.0 - door_opening_half_angle

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Box((3.40, 3.40, 2.70)),
        mass=720.0,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
    )

    drum.visual(
        Cylinder(radius=drum_radius + 0.02, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=floor_finish,
        name="threshold_disc",
    )
    drum.visual(
        Cylinder(radius=drum_radius + 0.04, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + canopy_thickness * 0.5)),
        material=aluminum,
        name="canopy",
    )
    drum.visual(
        Cylinder(radius=0.19, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + canopy_thickness + 0.05)),
        material=dark_metal,
        name="brake_center_hub",
    )
    drum.visual(
        Box((0.68, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.28, canopy_bottom + canopy_thickness + 0.03)),
        material=dark_metal,
        name="brake_link_cover",
    )
    drum.visual(
        Box((0.44, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, -0.56, canopy_bottom + canopy_thickness + 0.06)),
        material=dark_metal,
        name="brake_housing",
    )

    right_wall = _curved_wall_sector(
        inner_radius=wall_inner_radius,
        outer_radius=wall_outer_radius,
        bottom_z=floor_thickness,
        top_z=canopy_bottom,
        start_angle=-side_sector_half_angle,
        end_angle=side_sector_half_angle,
        arc_samples=28,
    )
    left_wall = _curved_wall_sector(
        inner_radius=wall_inner_radius,
        outer_radius=wall_outer_radius,
        bottom_z=floor_thickness,
        top_z=canopy_bottom,
        start_angle=pi - side_sector_half_angle,
        end_angle=pi + side_sector_half_angle,
        arc_samples=28,
    )
    drum.visual(_save_mesh("right_drum_wall", right_wall), material=smoked_glass, name="right_wall")
    drum.visual(_save_mesh("left_drum_wall", left_wall), material=smoked_glass, name="left_wall")

    post_height = canopy_bottom - floor_thickness
    post_mid_z = floor_thickness + post_height * 0.5
    for index, angle in enumerate(
        (
            side_sector_half_angle,
            -side_sector_half_angle,
            pi - side_sector_half_angle,
            pi + side_sector_half_angle,
        )
    ):
        drum.visual(
            Box((0.08, 0.10, post_height)),
            origin=_radial_box_origin((wall_inner_radius + wall_outer_radius) * 0.5, angle, post_mid_z),
            material=aluminum,
            name=f"entry_post_{index}",
        )

    drum.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, canopy_bottom + 0.09)),
        material=dark_metal,
        name="bearing_sleeve",
    )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Box((2.95, 2.95, 2.35)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
    )
    rotor.visual(
        Cylinder(radius=0.09, length=2.20),
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        material=dark_metal,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=aluminum,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.15)),
        material=aluminum,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.21, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.24)),
        material=dark_metal,
        name="brake_drum",
    )

    wing_angles = (pi / 2.0, pi / 2.0 + 2.0 * pi / 3.0, pi / 2.0 + 4.0 * pi / 3.0)
    for index, angle in enumerate(wing_angles):
        _add_rotor_wing(
            rotor,
            angle=angle,
            inner_stile_center=0.13,
            outer_stile_center=1.445,
            inner_stile_width=0.08,
            outer_stile_width=0.07,
            panel_thickness=0.045,
            wing_bottom=0.12,
            wing_height=2.02,
            top_rail_height=0.07,
            bottom_rail_height=0.14,
            glass_material=glass,
            frame_material=aluminum,
            kick_material=kick_plate,
            index=index,
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("rotor_spin")

    ctx.check("drum part exists", drum is not None)
    ctx.check("rotor part exists", rotor is not None)
    ctx.check(
        "continuous vertical rotor joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=f"type={spin.articulation_type}, axis={spin.axis}, limits={spin.motion_limits}",
    )
    ctx.check(
        "gentle speed brake intent",
        spin.motion_limits is not None and spin.motion_limits.velocity <= 0.75,
        details=f"velocity={None if spin.motion_limits is None else spin.motion_limits.velocity}",
    )
    ctx.check(
        "brake housing visual present",
        drum.get_visual("brake_housing") is not None and rotor.get_visual("brake_drum") is not None,
    )

    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        margin=0.18,
        name="rotor footprint stays inside drum at rest",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")
    rest_center = None
    if rest_aabb is not None:
        rest_center = (
            (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5,
            (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5,
        )

    with ctx.pose({spin: pi / 3.0}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.18,
            name="rotor footprint stays inside drum while turning",
        )
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")
        turned_center = None
        if turned_aabb is not None:
            turned_center = (
                (turned_aabb[0][0] + turned_aabb[1][0]) * 0.5,
                (turned_aabb[0][1] + turned_aabb[1][1]) * 0.5,
            )
        moved_sideways = (
            rest_center is not None
            and turned_center is not None
            and abs(turned_center[0] - rest_center[0]) > 0.45
            and abs(turned_center[1] - rest_center[1]) > 0.45
        )
        ctx.check(
            "front wing rotates around center post",
            moved_sideways,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
