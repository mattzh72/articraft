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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_satellite")

    blanket_gold = model.material("blanket_gold", rgba=(0.78, 0.66, 0.20, 1.0))
    radiator_black = model.material("radiator_black", rgba=(0.12, 0.13, 0.15, 1.0))
    solar_blue = model.material("solar_blue", rgba=(0.08, 0.17, 0.36, 1.0))
    frame_aluminum = model.material("frame_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    antenna_white = model.material("antenna_white", rgba=(0.90, 0.92, 0.95, 1.0))
    sensor_white = model.material("sensor_white", rgba=(0.83, 0.86, 0.90, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))

    apogee_nozzle_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.12, 0.0),
                (0.16, -0.10),
                (0.25, -0.28),
                (0.31, -0.40),
            ],
            inner_profile=[
                (0.08, 0.0),
                (0.12, -0.10),
                (0.20, -0.28),
                (0.26, -0.40),
            ],
            segments=52,
        ),
        "satellite_apogee_nozzle",
    )
    antenna_dish_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.00, -0.12),
                (0.07, -0.115),
                (0.18, -0.098),
                (0.30, -0.064),
                (0.40, -0.026),
                (0.45, 0.0),
            ],
            inner_profile=[
                (0.00, -0.104),
                (0.06, -0.100),
                (0.17, -0.085),
                (0.29, -0.054),
                (0.39, -0.020),
                (0.43, 0.0),
            ],
            segments=64,
        ),
        "satellite_high_gain_dish",
    )
    feed_strut_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.74, 0.0, 0.38),
                (0.86, 0.0, 0.22),
                (0.97, 0.0, 0.0),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "satellite_feed_strut",
    )

    bus = model.part("bus")
    bus.visual(
        Box((1.10, 0.98, 1.52)),
        material=blanket_gold,
        name="main_bus",
    )
    bus.visual(
        Box((1.18, 1.06, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=frame_aluminum,
        name="top_deck",
    )
    bus.visual(
        Box((1.18, 1.06, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.83)),
        material=frame_aluminum,
        name="bottom_deck",
    )
    bus.visual(
        Box((0.05, 0.88, 1.34)),
        origin=Origin(xyz=(0.575, 0.0, 0.0)),
        material=radiator_black,
        name="radiator_plus_x",
    )
    bus.visual(
        Box((0.05, 0.88, 1.34)),
        origin=Origin(xyz=(-0.575, 0.0, 0.0)),
        material=radiator_black,
        name="radiator_minus_x",
    )
    bus.visual(
        Box((0.12, 0.26, 0.36)),
        origin=Origin(xyz=(0.61, 0.0, 0.04)),
        material=frame_aluminum,
        name="antenna_mount",
    )
    bus.visual(
        Box((0.12, 0.06, 0.72)),
        origin=Origin(xyz=(0.0, 0.52, 0.0)),
        material=frame_aluminum,
        name="left_array_clevis",
    )
    bus.visual(
        Box((0.12, 0.06, 0.72)),
        origin=Origin(xyz=(0.0, -0.52, 0.0)),
        material=frame_aluminum,
        name="right_array_clevis",
    )
    bus.visual(
        Box((0.30, 0.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=frame_aluminum,
        name="sensor_plinth",
    )
    bus.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=sensor_white,
        name="sensor_mast",
    )
    bus.visual(
        Sphere(radius=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        material=sensor_white,
        name="star_tracker_dome",
    )
    bus.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.96)),
        material=dark_metal,
        name="engine_collar",
    )
    bus.visual(
        apogee_nozzle_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
        material=dark_metal,
        name="apogee_nozzle",
    )
    bus.inertial = Inertial.from_geometry(
        Box((1.18, 1.06, 2.66)),
        mass=860.0,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
    )

    def add_solar_wing(part_name: str) -> None:
        wing = model.part(part_name)
        wing.visual(
            Cylinder(radius=0.028, length=0.72),
            material=dark_metal,
            name="hinge_barrel",
        )
        wing.visual(
            Box((0.06, 0.08, 0.82)),
            origin=Origin(xyz=(0.0, 0.04, 0.0)),
            material=frame_aluminum,
            name="root_yoke",
        )
        wing.visual(
            Box((0.018, 0.96, 3.06)),
            origin=Origin(xyz=(0.0, 0.56, 0.0)),
            material=frame_aluminum,
            name="inner_panel",
        )
        wing.visual(
            Box((0.022, 0.10, 3.06)),
            origin=Origin(xyz=(0.0, 1.09, 0.0)),
            material=frame_aluminum,
            name="mid_hinge_strip",
        )
        wing.visual(
            Box((0.018, 0.92, 3.06)),
            origin=Origin(xyz=(0.0, 1.60, 0.0)),
            material=frame_aluminum,
            name="outer_panel",
        )
        wing.visual(
            Box((0.024, 1.98, 0.07)),
            origin=Origin(xyz=(0.0, 1.07, 1.495)),
            material=frame_aluminum,
            name="top_frame_rail",
        )
        wing.visual(
            Box((0.024, 1.98, 0.07)),
            origin=Origin(xyz=(0.0, 1.07, -1.495)),
            material=frame_aluminum,
            name="bottom_frame_rail",
        )
        wing.visual(
            Box((0.024, 0.08, 3.06)),
            origin=Origin(xyz=(0.0, 0.08, 0.0)),
            material=frame_aluminum,
            name="inner_frame_rail",
        )
        wing.visual(
            Box((0.024, 0.08, 3.06)),
            origin=Origin(xyz=(0.0, 2.06, 0.0)),
            material=frame_aluminum,
            name="outer_frame_rail",
        )
        cell_specs = (
            ("inner_upper_cells", 0.56, 0.76),
            ("inner_lower_cells", 0.56, -0.76),
            ("outer_upper_cells", 1.60, 0.76),
            ("outer_lower_cells", 1.60, -0.76),
        )
        for cell_name, center_y, center_z in cell_specs:
            wing.visual(
                Box((0.004, 0.82, 1.36)),
                origin=Origin(xyz=(0.011, center_y, center_z)),
                material=solar_blue,
                name=cell_name,
            )
        wing.inertial = Inertial.from_geometry(
            Box((0.07, 2.10, 3.12)),
            mass=42.0,
            origin=Origin(xyz=(0.0, 1.05, 0.0)),
        )

    add_solar_wing("left_solar_wing")
    add_solar_wing("right_solar_wing")

    high_gain_antenna = model.part("high_gain_antenna")
    high_gain_antenna.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="mount_collar",
    )
    high_gain_antenna.visual(
        Cylinder(radius=0.055, length=0.38),
        origin=Origin(xyz=(0.19, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="support_boom",
    )
    high_gain_antenna.visual(
        Box((0.20, 0.18, 0.18)),
        origin=Origin(xyz=(0.24, 0.0, -0.09)),
        material=radiator_black,
        name="transceiver_box",
    )
    high_gain_antenna.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="dish_hub",
    )
    high_gain_antenna.visual(
        Cylinder(radius=0.08, length=0.12),
        origin=Origin(xyz=(0.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="dish_neck",
    )
    high_gain_antenna.visual(
        antenna_dish_mesh,
        origin=Origin(xyz=(0.74, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_white,
        name="dish_shell",
    )
    high_gain_antenna.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(0.93, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="feed_ring",
    )
    high_gain_antenna.visual(
        Cylinder(radius=0.015, length=0.34),
        origin=Origin(xyz=(0.785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="feed_support_boom",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        high_gain_antenna.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=frame_aluminum,
            name=f"feed_strut_{index}",
        )
    high_gain_antenna.visual(
        Cylinder(radius=0.04, length=0.16),
        origin=Origin(xyz=(0.98, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_white,
        name="feed_horn",
    )
    high_gain_antenna.inertial = Inertial.from_geometry(
        Box((1.10, 0.92, 0.92)),
        mass=34.0,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
    )

    model.articulation(
        "bus_to_left_solar_wing",
        ArticulationType.REVOLUTE,
        parent=bus,
        child="left_solar_wing",
        origin=Origin(xyz=(0.0, 0.578, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "bus_to_right_solar_wing",
        ArticulationType.REVOLUTE,
        parent=bus,
        child="right_solar_wing",
        origin=Origin(xyz=(0.0, -0.578, 0.0), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "bus_to_high_gain_antenna",
        ArticulationType.FIXED,
        parent=bus,
        child=high_gain_antenna,
        origin=Origin(xyz=(0.67, 0.0, 0.04)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bus = object_model.get_part("bus")
    left_wing = object_model.get_part("left_solar_wing")
    right_wing = object_model.get_part("right_solar_wing")
    antenna = object_model.get_part("high_gain_antenna")
    left_hinge = object_model.get_articulation("bus_to_left_solar_wing")
    right_hinge = object_model.get_articulation("bus_to_right_solar_wing")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_wing,
            bus,
            name="left solar wing is mounted tightly to the bus",
        )
        ctx.expect_contact(
            bus,
            right_wing,
            name="right solar wing is mounted tightly to the bus",
        )

        bus_pos = ctx.part_world_position(bus)
        left_pos = ctx.part_world_position(left_wing)
        right_pos = ctx.part_world_position(right_wing)
        antenna_pos = ctx.part_world_position(antenna)
        left_rest_aabb = ctx.part_world_aabb(left_wing)
        right_rest_aabb = ctx.part_world_aabb(right_wing)

        ctx.check(
            "deployed wings sit on opposite sides of the bus",
            bus_pos is not None
            and left_pos is not None
            and right_pos is not None
            and left_pos[1] > bus_pos[1] + 0.55
            and right_pos[1] < bus_pos[1] - 0.55,
            details=f"bus={bus_pos}, left={left_pos}, right={right_pos}",
        )
        ctx.check(
            "high gain antenna projects forward of the bus",
            bus_pos is not None
            and antenna_pos is not None
            and antenna_pos[0] > bus_pos[0] + 0.60,
            details=f"bus={bus_pos}, antenna={antenna_pos}",
        )

    with ctx.pose({left_hinge: 1.20}):
        left_folded_aabb = ctx.part_world_aabb(left_wing)
        ctx.check(
            "left solar wing folds inward around its root hinge",
            left_rest_aabb is not None
            and left_folded_aabb is not None
            and left_folded_aabb[1][1] < left_rest_aabb[1][1] - 0.75,
            details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
        )

    with ctx.pose({right_hinge: 1.20}):
        right_folded_aabb = ctx.part_world_aabb(right_wing)
        ctx.check(
            "right solar wing folds inward around its root hinge",
            right_rest_aabb is not None
            and right_folded_aabb is not None
            and right_folded_aabb[0][1] > right_rest_aabb[0][1] + 0.75,
            details=f"rest={right_rest_aabb}, folded={right_folded_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
