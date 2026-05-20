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
)


def _ring_mesh(name: str, *, inner_radius: float, outer_radius: float, height: float):
    half_h = height * 0.5
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half_h), (outer_radius, half_h)],
            [(inner_radius, -half_h), (inner_radius, half_h)],
            segments=80,
        ),
        name,
    )


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theater_revolving_door")

    bronze = model.material("bronze", rgba=(0.23, 0.20, 0.17, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.16, 0.14, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.82, 0.90, 0.28))
    floor_stone = model.material("floor_stone", rgba=(0.56, 0.56, 0.54, 1.0))
    floor_mat = model.material("floor_mat", rgba=(0.14, 0.15, 0.16, 1.0))
    canopy_finish = model.material("canopy_finish", rgba=(0.28, 0.27, 0.25, 1.0))

    drum_inner_radius = 1.74
    drum_outer_radius = 1.92
    canopy_radius = 1.98
    clear_height = 2.24
    top_ring_height = 0.12
    floor_thickness = 0.06

    sill_ring_mesh = _ring_mesh(
        "drum_sill_ring",
        inner_radius=drum_inner_radius,
        outer_radius=drum_outer_radius,
        height=0.05,
    )
    transom_ring_mesh = _ring_mesh(
        "drum_transom_ring",
        inner_radius=drum_inner_radius,
        outer_radius=drum_outer_radius,
        height=top_ring_height,
    )
    canopy_fascia_mesh = _ring_mesh(
        "canopy_fascia_ring",
        inner_radius=1.58,
        outer_radius=canopy_radius,
        height=0.16,
    )
    entry_mat_mesh = _ring_mesh(
        "entry_mat_ring",
        inner_radius=0.22,
        outer_radius=1.68,
        height=0.010,
    )
    rotor_sleeve_mesh = _ring_mesh(
        "rotor_center_sleeve",
        inner_radius=0.098,
        outer_radius=0.118,
        height=2.04,
    )
    rotor_collar_mesh = _ring_mesh(
        "rotor_collar",
        inner_radius=0.098,
        outer_radius=0.150,
        height=0.06,
    )

    floor_base = model.part("floor_base")
    floor_base.visual(
        Cylinder(radius=2.08, length=floor_thickness),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=floor_stone,
        name="floor_slab",
    )
    floor_base.visual(
        entry_mat_mesh,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.005)),
        material=floor_mat,
        name="entry_mat",
    )
    floor_base.inertial = Inertial.from_geometry(
        Cylinder(radius=2.08, length=floor_thickness),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
    )

    outer_enclosure = model.part("outer_enclosure")
    outer_enclosure.visual(
        sill_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_bronze,
        name="drum_sill",
    )
    outer_enclosure.visual(
        transom_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, clear_height + top_ring_height * 0.5)),
        material=dark_bronze,
        name="drum_transom",
    )

    jamb_angles_deg = (50.0, 130.0, -130.0, -50.0)
    mullion_angles_deg = (70.0, 90.0, 110.0, -110.0, -90.0, -70.0)
    glass_angles_deg = (60.0, 80.0, 100.0, 120.0, -120.0, -100.0, -80.0, -60.0)

    for index, angle_deg in enumerate(jamb_angles_deg):
        angle = math.radians(angle_deg)
        x, y = _polar_xy(1.83, angle)
        outer_enclosure.visual(
            Box((0.12, 0.14, clear_height)),
            origin=Origin(xyz=(x, y, clear_height * 0.5), rpy=(0.0, 0.0, angle + math.pi * 0.5)),
            material=bronze,
            name=f"opening_jamb_{index}",
        )

    for index, angle_deg in enumerate(mullion_angles_deg):
        angle = math.radians(angle_deg)
        x, y = _polar_xy(1.83, angle)
        outer_enclosure.visual(
            Box((0.06, 0.08, clear_height)),
            origin=Origin(xyz=(x, y, clear_height * 0.5), rpy=(0.0, 0.0, angle + math.pi * 0.5)),
            material=bronze,
            name=f"side_mullion_{index}",
        )

    for index, angle_deg in enumerate(glass_angles_deg):
        angle = math.radians(angle_deg)
        x, y = _polar_xy(1.82, angle)
        outer_enclosure.visual(
            Box((0.56, 0.018, 2.08)),
            origin=Origin(xyz=(x, y, 1.09), rpy=(0.0, 0.0, angle + math.pi * 0.5)),
            material=glass,
            name=f"curved_glass_panel_{index}",
        )

    outer_enclosure.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_outer_radius, length=2.41),
        mass=720.0,
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
    )

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=canopy_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=canopy_finish,
        name="ceiling_disk",
    )
    canopy.visual(
        canopy_fascia_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_bronze,
        name="ceiling_fascia",
    )
    canopy.visual(
        Cylinder(radius=1.52, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=canopy_finish,
        name="roof_crown",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=canopy_radius, length=0.18),
        mass=460.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    center_post = model.part("center_post")
    center_post.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_bronze,
        name="bottom_bearing_drum",
    )
    center_post.visual(
        Cylinder(radius=0.09, length=2.30),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=brushed_steel,
        name="central_post_shaft",
    )
    center_post.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.33)),
        material=dark_bronze,
        name="top_bearing_cap",
    )
    center_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=2.36),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        rotor_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=bronze,
        name="rotor_sleeve",
    )
    rotor.visual(
        rotor_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_bronze,
        name="lower_rotor_collar",
    )
    rotor.visual(
        rotor_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.195)),
        material=dark_bronze,
        name="upper_rotor_collar",
    )

    for wing_index in range(4):
        angle = wing_index * (math.pi * 0.5)
        spider_x, spider_y = _polar_xy(0.185, angle)
        inner_x, inner_y = _polar_xy(0.24, angle)
        outer_x, outer_y = _polar_xy(1.56, angle)
        panel_x, panel_y = _polar_xy(0.90, angle)
        rotor.visual(
            Box((0.14, 0.050, 0.040)),
            origin=Origin(xyz=(spider_x, spider_y, 0.17), rpy=(0.0, 0.0, angle)),
            material=dark_bronze,
            name=f"wing_{wing_index}_lower_spider_arm",
        )
        rotor.visual(
            Box((0.14, 0.050, 0.040)),
            origin=Origin(xyz=(spider_x, spider_y, 2.15), rpy=(0.0, 0.0, angle)),
            material=dark_bronze,
            name=f"wing_{wing_index}_upper_spider_arm",
        )
        rotor.visual(
            Box((0.060, 0.080, 2.10)),
            origin=Origin(xyz=(inner_x, inner_y, 1.11), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"wing_{wing_index}_inner_stile",
        )
        rotor.visual(
            Box((0.060, 0.080, 2.10)),
            origin=Origin(xyz=(outer_x, outer_y, 1.11), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"wing_{wing_index}_outer_stile",
        )
        rotor.visual(
            Box((1.36, 0.080, 0.050)),
            origin=Origin(xyz=(panel_x, panel_y, 0.085), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"wing_{wing_index}_bottom_rail",
        )
        rotor.visual(
            Box((1.36, 0.080, 0.050)),
            origin=Origin(xyz=(panel_x, panel_y, 2.135), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"wing_{wing_index}_top_rail",
        )
        rotor.visual(
            Box((1.18, 0.040, 0.045)),
            origin=Origin(xyz=(panel_x, panel_y, 1.07), rpy=(0.0, 0.0, angle)),
            material=dark_bronze,
            name=f"wing_{wing_index}_push_rail",
        )
        rotor.visual(
            Box((1.36, 0.018, 2.03)),
            origin=Origin(xyz=(panel_x, panel_y, 1.085), rpy=(0.0, 0.0, angle)),
            material=glass,
            name=f"wing_{wing_index}_glass",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.60, length=2.26),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
    )

    model.articulation(
        "floor_to_outer_enclosure",
        ArticulationType.FIXED,
        parent=floor_base,
        child=outer_enclosure,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness)),
    )
    model.articulation(
        "outer_enclosure_to_canopy",
        ArticulationType.FIXED,
        parent=outer_enclosure,
        child=canopy,
        origin=Origin(xyz=(0.0, 0.0, clear_height + top_ring_height)),
    )
    model.articulation(
        "floor_to_center_post",
        ArticulationType.FIXED,
        parent=floor_base,
        child=center_post,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness)),
    )
    model.articulation(
        "center_post_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=center_post,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    floor_base = object_model.get_part("floor_base")
    outer_enclosure = object_model.get_part("outer_enclosure")
    canopy = object_model.get_part("canopy")
    center_post = object_model.get_part("center_post")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("center_post_to_rotor")

    ctx.expect_contact(
        outer_enclosure,
        floor_base,
        elem_a="drum_sill",
        elem_b="floor_slab",
        name="drum sill seats on the theater floor",
    )
    ctx.expect_contact(
        canopy,
        outer_enclosure,
        elem_a="ceiling_disk",
        elem_b="drum_transom",
        name="canopy bears on the transom ring",
    )
    ctx.expect_contact(
        center_post,
        floor_base,
        elem_a="bottom_bearing_drum",
        elem_b="floor_slab",
        name="central post is mounted to the floor slab",
    )
    ctx.expect_origin_distance(
        center_post,
        rotor,
        axes="xy",
        max_dist=0.0001,
        name="rotor is centered on the post axis",
    )
    ctx.expect_gap(
        canopy,
        rotor,
        axis="z",
        positive_elem="ceiling_disk",
        negative_elem="upper_rotor_collar",
        min_gap=0.08,
        max_gap=0.14,
        name="rotor clears the canopy soffit",
    )
    ctx.expect_gap(
        rotor,
        floor_base,
        axis="z",
        positive_elem="wing_0_glass",
        negative_elem="floor_slab",
        min_gap=0.05,
        max_gap=0.08,
        name="door wings sweep just above the floor",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )

    rest_center = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="wing_0_glass"))
    with ctx.pose({rotor_spin: math.pi * 0.5}):
        quarter_turn_center = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="wing_0_glass"))

    rotation_ok = False
    if rest_center is not None and quarter_turn_center is not None:
        rest_radius = math.hypot(rest_center[0], rest_center[1])
        turn_radius = math.hypot(quarter_turn_center[0], quarter_turn_center[1])
        rotation_ok = (
            rest_center[0] > 0.70
            and abs(rest_center[1]) < 0.10
            and quarter_turn_center[1] > 0.70
            and abs(quarter_turn_center[0]) < 0.10
            and abs(rest_radius - turn_radius) < 0.02
        )
    ctx.check(
        "rotor quarter turn moves a wing around the central axis",
        rotation_ok,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
