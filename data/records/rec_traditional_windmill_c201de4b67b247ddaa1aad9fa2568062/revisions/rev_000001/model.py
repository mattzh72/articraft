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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.73, 0.70, 0.63, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.47, 0.34, 0.24, 1.0))
    dark_roof = model.material("dark_roof", rgba=(0.19, 0.17, 0.16, 1.0))
    aged_canvas = model.material("aged_canvas", rgba=(0.82, 0.78, 0.67, 1.0))
    iron = model.material("iron", rgba=(0.32, 0.31, 0.33, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.95, length=1.90),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=stone,
        name="tower_base_stage",
    )
    tower.visual(
        Cylinder(radius=1.78, length=2.20),
        origin=Origin(xyz=(0.0, 0.0, 2.95)),
        material=stone,
        name="tower_mid_stage_1",
    )
    tower.visual(
        Cylinder(radius=1.56, length=2.20),
        origin=Origin(xyz=(0.0, 0.0, 5.05)),
        material=stone,
        name="tower_mid_stage_2",
    )
    tower.visual(
        Cylinder(radius=1.34, length=2.30),
        origin=Origin(xyz=(0.0, 0.0, 7.05)),
        material=stone,
        name="tower_upper_stage",
    )
    tower.visual(
        Cylinder(radius=1.20, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 8.25)),
        material=stone,
        name="tower_top_stage",
    )
    tower.visual(
        Cylinder(radius=1.25, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 8.31)),
        material=weathered_wood,
        name="tower_curb",
    )
    tower.visual(
        Cylinder(radius=2.02, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="tower_footing",
    )
    tower.visual(
        Box((0.10, 0.95, 1.75)),
        origin=Origin(xyz=(1.86, 0.0, 0.875)),
        material=weathered_wood,
        name="front_door",
    )
    for idx, z in enumerate((3.0, 5.1, 6.8), start=1):
        tower.visual(
            Box((0.06, 0.52, 0.74)),
            origin=Origin(xyz=(1.28, 0.0, z)),
            material=weathered_wood,
            name=f"window_{idx}",
        )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.95, length=8.55),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 4.275)),
    )

    cap = model.part("cap")
    cap.visual(
        Box((2.46, 2.18, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=weathered_wood,
        name="cap_base",
    )
    cap.visual(
        Box((2.12, 2.04, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=weathered_wood,
        name="cap_body",
    )
    cap.visual(
        Box((1.55, 2.20, 0.14)),
        origin=Origin(xyz=(0.32, 0.0, 0.73), rpy=(0.0, -0.42, 0.0)),
        material=dark_roof,
        name="front_roof",
    )
    cap.visual(
        Box((1.55, 2.20, 0.14)),
        origin=Origin(xyz=(-0.38, 0.0, 0.72), rpy=(0.0, 0.36, 0.0)),
        material=dark_roof,
        name="rear_roof",
    )
    cap.visual(
        Box((0.38, 2.14, 0.28)),
        origin=Origin(xyz=(0.00, 0.0, 0.80)),
        material=dark_roof,
        name="roof_ridge",
    )
    cap.visual(
        Box((0.22, 2.05, 0.70)),
        origin=Origin(xyz=(1.16, 0.0, 0.48)),
        material=dark_roof,
        name="front_face",
    )
    cap.visual(
        Box((0.24, 2.05, 0.62)),
        origin=Origin(xyz=(-1.12, 0.0, 0.42)),
        material=dark_roof,
        name="rear_face",
    )
    cap.visual(
        Cylinder(radius=1.34, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=weathered_wood,
        name="turn_ring",
    )
    cap.visual(
        Cylinder(radius=0.26, length=0.46),
        origin=Origin(xyz=(1.42, 0.0, 0.63), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="front_bearing",
    )
    cap.visual(
        Box((0.14, 0.72, 0.48)),
        origin=Origin(xyz=(1.15, 0.0, 0.50)),
        material=weathered_wood,
        name="bearing_cheeks",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.70, 2.70, 1.45)),
        mass=480.0,
        origin=Origin(xyz=(0.20, 0.0, 0.72)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.3),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="rear_collar",
    )
    hub.visual(
        Cylinder(radius=0.34, length=0.28),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="hub_body",
    )
    hub.visual(
        Cylinder(radius=0.14, length=0.12),
        origin=Origin(xyz=(0.41, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="nose_cap",
    )
    hub.visual(
        Box((0.18, 0.22, 3.10)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=weathered_wood,
        name="vertical_stock",
    )
    hub.visual(
        Box((0.18, 3.10, 0.22)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=weathered_wood,
        name="horizontal_stock",
    )

    def blade_xyz(angle: float, y_local: float, z_local: float, x_local: float = 0.22) -> tuple[float, float, float]:
        c = math.cos(angle)
        s = math.sin(angle)
        return (x_local, y_local * c - z_local * s, y_local * s + z_local * c)

    slat_positions = (1.78, 2.18, 2.58, 2.98, 3.38, 3.78, 4.18, 4.58)
    shutter_positions = (2.00, 2.80, 3.60, 4.40)
    blade_angles = (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)

    for blade_index, angle in enumerate(blade_angles):
        base_rpy = (angle, 0.0, 0.0)
        hub.visual(
            Box((0.06, 0.08, 3.60)),
            origin=Origin(xyz=blade_xyz(angle, -0.27, 3.15), rpy=base_rpy),
            material=weathered_wood,
            name=f"blade_{blade_index}_rail_left",
        )
        hub.visual(
            Box((0.06, 0.08, 3.60)),
            origin=Origin(xyz=blade_xyz(angle, 0.27, 3.15), rpy=base_rpy),
            material=weathered_wood,
            name=f"blade_{blade_index}_rail_right",
        )
        hub.visual(
            Box((0.08, 0.62, 0.18)),
            origin=Origin(xyz=blade_xyz(angle, 0.0, 1.40), rpy=base_rpy),
            material=weathered_wood,
            name=f"blade_{blade_index}_root",
        )
        hub.visual(
            Box((0.06, 0.62, 0.16)),
            origin=Origin(xyz=blade_xyz(angle, 0.0, 4.96), rpy=base_rpy),
            material=weathered_wood,
            name=f"blade_{blade_index}_tip",
        )
        for slat_index, z_local in enumerate(slat_positions):
            hub.visual(
                Box((0.03, 0.62, 0.05)),
                origin=Origin(xyz=blade_xyz(angle, 0.0, z_local), rpy=base_rpy),
                material=weathered_wood,
                name=f"blade_{blade_index}_slat_{slat_index}",
            )
        for shutter_index, z_local in enumerate(shutter_positions):
            hub.visual(
                Box((0.015, 0.56, 0.30)),
                origin=Origin(xyz=blade_xyz(angle, 0.0, z_local), rpy=base_rpy),
                material=aged_canvas,
                name=f"blade_{blade_index}_shutter_{shutter_index}",
            )
    hub.inertial = Inertial.from_geometry(
        Box((0.60, 10.20, 10.20)),
        mass=240.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(1.65, 0.0, 0.63)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    cap_turn = object_model.get_articulation("tower_to_cap")
    hub_spin = object_model.get_articulation("cap_to_hub")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="turn_ring",
        negative_elem="tower_curb",
        max_gap=0.001,
        max_penetration=0.0,
        name="cap turn ring seats on the tower curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        elem_a="turn_ring",
        elem_b="tower_curb",
        min_overlap=2.4,
        name="cap stays centered over the tower top",
    )
    ctx.expect_gap(
        hub,
        cap,
        axis="x",
        positive_elem="rear_collar",
        negative_elem="front_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub collar seats directly against the front bearing",
    )
    ctx.expect_origin_distance(
        hub,
        cap,
        axes="x",
        min_dist=1.60,
        max_dist=1.70,
        name="hub axis stays compact and close to the cap support",
    )

    rest_hub_pos = ctx.part_world_position(hub)
    rest_tip_aabb = ctx.part_element_world_aabb(hub, elem="blade_0_tip")
    rest_tip_center = aabb_center(rest_tip_aabb)

    with ctx.pose({hub_spin: math.pi / 2.0}):
        spun_tip_aabb = ctx.part_element_world_aabb(hub, elem="blade_0_tip")
        spun_tip_center = aabb_center(spun_tip_aabb)

    ctx.check(
        "hub spin carries the upper sail tip into a side position",
        rest_tip_center is not None
        and spun_tip_center is not None
        and rest_tip_center[2] > 13.8
        and spun_tip_center[1] < -4.8
        and abs(spun_tip_center[2] - 9.03) < 0.4,
        details=f"rest_tip_center={rest_tip_center}, spun_tip_center={spun_tip_center}",
    )

    with ctx.pose({cap_turn: math.pi / 2.0}):
        turned_hub_pos = ctx.part_world_position(hub)

    ctx.check(
        "cap rotation swings the hub around the vertical tower axis",
        rest_hub_pos is not None
        and turned_hub_pos is not None
        and rest_hub_pos[0] > 1.6
        and abs(rest_hub_pos[1]) < 0.05
        and abs(turned_hub_pos[0]) < 0.05
        and turned_hub_pos[1] > 1.6
        and abs(turned_hub_pos[2] - rest_hub_pos[2]) < 1e-6,
        details=f"rest_hub_pos={rest_hub_pos}, turned_hub_pos={turned_hub_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
