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
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_about_x(y_value: float, z_value: float, angle: float) -> tuple[float, float]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        y_value * cos_a - z_value * sin_a,
        y_value * sin_a + z_value * cos_a,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    masonry = model.material("masonry", rgba=(0.78, 0.75, 0.69, 1.0))
    cap_green = model.material("cap_green", rgba=(0.23, 0.31, 0.24, 1.0))
    timber = model.material("timber", rgba=(0.46, 0.33, 0.20, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.28, 0.20, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.23, 0.24, 0.26, 1.0))
    white_trim = model.material("white_trim", rgba=(0.90, 0.91, 0.89, 1.0))

    tower_profile = [
        (0.0, 0.0),
        (2.72, 0.0),
        (2.66, 0.22),
        (2.48, 1.60),
        (2.28, 4.60),
        (2.02, 8.10),
        (1.80, 10.70),
        (1.73, 11.35),
        (1.71, 11.62),
        (0.0, 11.62),
    ]
    cap_profile = [
        (0.0, 0.0),
        (1.78, 0.0),
        (1.73, 0.18),
        (1.56, 0.72),
        (1.24, 1.45),
        (0.78, 2.20),
        (0.18, 2.62),
        (0.0, 2.70),
    ]

    tower = model.part("tower")
    tower.visual(
        _save_mesh("windmill_tower_shell", LatheGeometry(tower_profile, segments=72)),
        material=masonry,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.86, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_timber,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=1.86, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 11.67)),
        material=dark_timber,
        name="cap_track_ring",
    )
    tower.visual(
        Box((0.95, 0.34, 2.28)),
        origin=Origin(xyz=(2.14, 0.0, 1.14)),
        material=dark_timber,
        name="front_door",
    )
    tower.visual(
        Box((0.18, 1.30, 1.90)),
        origin=Origin(xyz=(2.44, 0.0, 1.16)),
        material=white_trim,
        name="door_trim",
    )
    tower.visual(
        Box((0.18, 0.40, 8.20)),
        origin=Origin(xyz=(2.18, 0.0, 5.26)),
        material=white_trim,
        name="front_trim_spine",
    )
    for index, z in enumerate((3.10, 5.75, 8.55)):
        width = 0.84 - 0.08 * index
        frame_x = (1.90, 1.84, 1.82)[index]
        dark_x = (1.88, 1.82, 1.80)[index]
        tower.visual(
            Box((0.52, width, 1.02)),
            origin=Origin(xyz=(frame_x, 0.0, z)),
            material=white_trim,
            name=f"window_frame_{index}",
        )
        tower.visual(
            Box((0.34, width * 0.72, 0.72)),
            origin=Origin(xyz=(dark_x, 0.0, z)),
            material=iron,
            name=f"window_dark_{index}",
        )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.72, length=11.90),
        mass=26000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.95)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.66, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_timber,
        name="cap_base_ring",
    )
    cap.visual(
        _save_mesh("windmill_cap_shell", LatheGeometry(cap_profile, segments=64)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=cap_green,
        name="cap_shell",
    )
    cap.visual(
        Box((1.38, 2.18, 0.56)),
        origin=Origin(xyz=(0.20, 0.0, 0.90)),
        material=cap_green,
        name="roof_saddle",
    )
    cap.visual(
        Box((1.02, 1.08, 0.20)),
        origin=Origin(xyz=(1.60, 0.0, 1.36)),
        material=dark_timber,
        name="front_bearing_top",
    )
    cap.visual(
        Box((1.02, 1.08, 0.20)),
        origin=Origin(xyz=(1.60, 0.0, 0.88)),
        material=dark_timber,
        name="front_bearing_bottom",
    )
    cap.visual(
        Box((1.02, 0.22, 0.48)),
        origin=Origin(xyz=(1.60, 0.43, 1.12)),
        material=dark_timber,
        name="front_bearing_left_cheek",
    )
    cap.visual(
        Box((1.02, 0.22, 0.48)),
        origin=Origin(xyz=(1.60, -0.43, 1.12)),
        material=dark_timber,
        name="front_bearing_right_cheek",
    )
    cap.visual(
        Box((0.24, 0.12, 0.72)),
        origin=Origin(xyz=(2.00, 0.58, 1.12)),
        material=dark_timber,
        name="front_bearing_left_stanchion",
    )
    cap.visual(
        Box((0.24, 0.12, 0.72)),
        origin=Origin(xyz=(2.00, -0.58, 1.12)),
        material=dark_timber,
        name="front_bearing_right_stanchion",
    )
    cap.visual(
        Box((0.36, 0.58, 0.10)),
        origin=Origin(xyz=(2.00, 0.0, 1.56)),
        material=dark_timber,
        name="front_bearing_upper_brow",
    )
    cap.visual(
        Box((0.36, 0.58, 0.10)),
        origin=Origin(xyz=(2.00, 0.0, 0.68)),
        material=dark_timber,
        name="front_bearing_lower_brow",
    )
    cap.visual(
        _save_mesh(
            "windmill_service_ring",
            TorusGeometry(radius=0.48, tube=0.04, radial_segments=18, tubular_segments=44).rotate_y(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(1.80, 0.0, 1.12)),
        material=iron,
        name="service_ring",
    )
    cap.visual(
        Box((0.12, 0.82, 0.12)),
        origin=Origin(xyz=(1.80, 0.0, 1.46), rpy=(math.pi / 4.0, 0.0, 0.0)),
        material=iron,
        name="service_ring_brace_upper",
    )
    cap.visual(
        Box((0.12, 0.82, 0.12)),
        origin=Origin(xyz=(1.80, 0.0, 0.78), rpy=(-math.pi / 4.0, 0.0, 0.0)),
        material=iron,
        name="service_ring_brace_lower",
    )
    cap.visual(
        Box((0.16, 0.92, 0.16)),
        origin=Origin(xyz=(1.28, 0.0, 1.12)),
        material=dark_timber,
        name="front_bearing_housing",
    )
    cap.visual(
        _save_mesh(
            "windmill_front_bearing_ring",
            TorusGeometry(radius=0.27, tube=0.06, radial_segments=18, tubular_segments=44).rotate_y(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(1.98, 0.0, 1.12)),
        material=iron,
        name="front_bearing_ring",
    )
    cap.visual(
        Box((2.20, 0.24, 0.18)),
        origin=Origin(xyz=(-1.20, 0.0, 0.98)),
        material=timber,
        name="tail_beam",
    )
    cap.visual(
        Box((1.10, 0.18, 1.44)),
        origin=Origin(xyz=(-2.08, 0.0, 1.12)),
        material=timber,
        name="tail_post",
    )
    cap.inertial = Inertial.from_geometry(
        Box((4.60, 3.80, 2.90)),
        mass=3800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.14, length=1.44),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="windshaft",
    )
    hub.visual(
        Cylinder(radius=0.34, length=0.58),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_timber,
        name="hub_barrel",
    )
    hub.visual(
        Cylinder(radius=0.49, length=0.16),
        origin=Origin(xyz=(0.56, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_face",
    )
    for blade_index, blade_angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stock_y, stock_z = _rotate_about_x(0.0, 2.10, blade_angle)
        hub.visual(
            Box((0.12, 0.16, 4.20)),
            origin=Origin(xyz=(0.44, stock_y, stock_z), rpy=(blade_angle, 0.0, 0.0)),
            material=dark_timber,
            name=f"blade_stock_{blade_index}",
        )
        rail_front_y, rail_front_z = _rotate_about_x(0.32, 2.35, blade_angle)
        hub.visual(
            Box((0.08, 0.10, 3.00)),
            origin=Origin(xyz=(0.43, rail_front_y, rail_front_z), rpy=(blade_angle, 0.0, 0.0)),
            material=timber,
            name=f"blade_rail_front_{blade_index}",
        )
        rail_rear_y, rail_rear_z = _rotate_about_x(-0.32, 2.35, blade_angle)
        hub.visual(
            Box((0.08, 0.10, 3.00)),
            origin=Origin(xyz=(0.43, rail_rear_y, rail_rear_z), rpy=(blade_angle, 0.0, 0.0)),
            material=timber,
            name=f"blade_rail_rear_{blade_index}",
        )
        for rung_index, rung_z_local in enumerate((0.92, 1.50, 2.08, 2.66, 3.24, 3.82)):
            rung_y, rung_z = _rotate_about_x(0.0, rung_z_local, blade_angle)
            hub.visual(
                Box((0.06, 0.74, 0.08)),
                origin=Origin(xyz=(0.43, rung_y, rung_z), rpy=(blade_angle, 0.0, 0.0)),
                material=white_trim,
                name=f"blade_rung_{blade_index}_{rung_index}",
            )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.52, length=1.60),
        mass=1600.0,
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 11.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30000.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(2.32, 0.0, 1.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=1.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    cap_joint = object_model.get_articulation("tower_to_cap")

    ctx.expect_origin_gap(
        hub,
        cap,
        axis="x",
        min_gap=1.8,
        name="hub projects forward from the cap face",
    )
    ctx.expect_origin_distance(
        cap,
        tower,
        axes="xy",
        max_dist=0.001,
        name="cap rotation axis stays centered on the tower",
    )

    hub_rest = ctx.part_world_position(hub)
    with ctx.pose({cap_joint: math.pi / 2.0}):
        hub_turned = ctx.part_world_position(hub)
    ctx.check(
        "cap yaw swings the front stage around the tower axis",
        hub_rest is not None
        and hub_turned is not None
        and hub_rest[0] > 1.5
        and abs(hub_rest[1]) < 0.05
        and hub_turned[1] > 1.5
        and abs(hub_turned[0]) < 0.05,
        details=f"rest={hub_rest}, turned={hub_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
