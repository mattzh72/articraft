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
    superellipse_profile,
)


def _ellipse_point(a: float, b: float, angle: float) -> tuple[float, float]:
    return (a * math.cos(angle), b * math.sin(angle))


def _ellipse_tangent_yaw(a: float, b: float, angle: float) -> float:
    tangent_x = -a * math.sin(angle)
    tangent_y = b * math.cos(angle)
    return math.atan2(tangent_y, tangent_x)


def _add_wing(
    model: ArticulatedObject,
    *,
    name: str,
    frame_material,
    glass_material,
) -> None:
    wing = model.part(name)

    panel_height = 2.50
    frame_depth = 0.045
    inner_stile_width = 0.05
    outer_stile_width = 0.05
    rail_height = 0.05
    bottom_z = 0.06

    inner_stile_center_x = 0.145
    outer_stile_center_x = 1.095
    rail_center_x = (inner_stile_center_x + outer_stile_center_x) * 0.5
    rail_length = outer_stile_center_x - inner_stile_center_x

    wing.visual(
        Box((inner_stile_width, frame_depth, panel_height)),
        origin=Origin(xyz=(inner_stile_center_x, 0.0, bottom_z + panel_height * 0.5)),
        material=frame_material,
        name="inner_stile",
    )
    wing.visual(
        Box((outer_stile_width, frame_depth, panel_height)),
        origin=Origin(xyz=(outer_stile_center_x, 0.0, bottom_z + panel_height * 0.5)),
        material=frame_material,
        name="outer_stile",
    )
    wing.visual(
        Box((rail_length, frame_depth, rail_height)),
        origin=Origin(xyz=(rail_center_x, 0.0, bottom_z + rail_height * 0.5)),
        material=frame_material,
        name="bottom_rail",
    )
    wing.visual(
        Box((rail_length, frame_depth, rail_height)),
        origin=Origin(
            xyz=(rail_center_x, 0.0, bottom_z + panel_height - rail_height * 0.5)
        ),
        material=frame_material,
        name="top_rail",
    )
    wing.visual(
        Box((0.90, 0.012, 2.40)),
        origin=Origin(xyz=(rail_center_x, 0.0, bottom_z + panel_height * 0.5)),
        material=glass_material,
        name="glass_panel",
    )

    wing.inertial = Inertial.from_geometry(
        Box((1.13, 0.05, 2.50)),
        mass=35.0,
        origin=Origin(xyz=(0.565, 0.0, bottom_z + panel_height * 0.5)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_revolving_door")

    anodized_aluminum = model.material(
        "anodized_aluminum",
        rgba=(0.72, 0.74, 0.77, 1.0),
    )
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    graphite_floor = model.material("graphite_floor", rgba=(0.19, 0.19, 0.20, 1.0))
    drum_glass = model.material("drum_glass", rgba=(0.70, 0.83, 0.88, 0.25))
    wing_glass = model.material("wing_glass", rgba=(0.78, 0.90, 0.95, 0.30))

    floor_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            superellipse_profile(3.34, 2.58, exponent=2.05, segments=72),
            0.06,
        ),
        "revolving_door_floor_plate",
    )
    canopy_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            superellipse_profile(3.46, 2.70, exponent=2.05, segments=72),
            0.18,
        ),
        "revolving_door_canopy",
    )

    housing = model.part("drum_housing")
    housing.visual(
        floor_plate_mesh,
        material=graphite_floor,
        name="floor_plinth",
    )
    housing.visual(
        canopy_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.72)),
        material=anodized_aluminum,
        name="canopy_shell",
    )

    wall_height = 2.66
    wall_center_z = 0.06 + wall_height * 0.5
    glass_a = 1.56
    glass_b = 1.23
    open_half_angle = 0.62
    segment_count = 4

    for side_name, angle_start, angle_end in (
        ("left", open_half_angle, math.pi - open_half_angle),
        ("right", -math.pi + open_half_angle, -open_half_angle),
    ):
        boundary_angles = [
            angle_start + (angle_end - angle_start) * index / segment_count
            for index in range(segment_count + 1)
        ]
        for index in range(segment_count):
            a0 = boundary_angles[index]
            a1 = boundary_angles[index + 1]
            mid_angle = 0.5 * (a0 + a1)
            p0x, p0y = _ellipse_point(glass_a, glass_b, a0)
            p1x, p1y = _ellipse_point(glass_a, glass_b, a1)
            cx, cy = _ellipse_point(glass_a, glass_b, mid_angle)
            width = math.hypot(p1x - p0x, p1y - p0y) * 0.84
            yaw = _ellipse_tangent_yaw(glass_a, glass_b, mid_angle)
            housing.visual(
                Box((width, 0.028, wall_height)),
                origin=Origin(xyz=(cx, cy, wall_center_z), rpy=(0.0, 0.0, yaw)),
                material=drum_glass,
                name=f"{side_name}_glass_panel_{index}",
            )

    post_a = 1.58
    post_b = 1.25
    for name, angle in (
        ("front_left_post", open_half_angle),
        ("rear_left_post", math.pi - open_half_angle),
        ("front_right_post", -open_half_angle),
        ("rear_right_post", -math.pi + open_half_angle),
    ):
        px, py = _ellipse_point(post_a, post_b, angle)
        housing.visual(
            Box((0.11, 0.09, wall_height)),
            origin=Origin(
                xyz=(px, py, wall_center_z),
                rpy=(0.0, 0.0, _ellipse_tangent_yaw(post_a, post_b, angle)),
            ),
            material=anodized_aluminum,
            name=name,
        )

    housing.inertial = Inertial.from_geometry(
        Box((3.46, 2.70, 2.90)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )

    hub = model.part("central_hub")
    hub.visual(
        Cylinder(radius=0.12, length=2.52),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=dark_metal,
        name="hub_post",
    )
    hub.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=anodized_aluminum,
        name="bottom_bearing",
    )
    hub.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.60)),
        material=anodized_aluminum,
        name="top_bearing",
    )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=2.66),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 1.33)),
    )

    _add_wing(
        model,
        name="wing_pos_x",
        frame_material=anodized_aluminum,
        glass_material=wing_glass,
    )
    _add_wing(
        model,
        name="wing_pos_y",
        frame_material=anodized_aluminum,
        glass_material=wing_glass,
    )
    _add_wing(
        model,
        name="wing_neg_x",
        frame_material=anodized_aluminum,
        glass_material=wing_glass,
    )
    _add_wing(
        model,
        name="wing_neg_y",
        frame_material=anodized_aluminum,
        glass_material=wing_glass,
    )

    model.articulation(
        "housing_to_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5),
    )

    for wing_name, angle in (
        ("wing_pos_x", 0.0),
        ("wing_pos_y", math.pi / 2.0),
        ("wing_neg_x", math.pi),
        ("wing_neg_y", -math.pi / 2.0),
    ):
        model.articulation(
            f"hub_to_{wing_name}",
            ArticulationType.FIXED,
            parent=hub,
            child=wing_name,
            origin=Origin(rpy=(0.0, 0.0, angle)),
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
    housing = object_model.get_part("drum_housing")
    hub = object_model.get_part("central_hub")
    rotor_spin = object_model.get_articulation("housing_to_hub_spin")
    wings = [
        object_model.get_part("wing_pos_x"),
        object_model.get_part("wing_pos_y"),
        object_model.get_part("wing_neg_x"),
        object_model.get_part("wing_neg_y"),
    ]

    ctx.check(
        "hub uses a continuous vertical articulation",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 3) for value in rotor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={rotor_spin.articulation_type}, axis={rotor_spin.axis}",
    )

    for wing in wings:
        ctx.expect_contact(
            wing,
            hub,
            name=f"{wing.name} is mounted to the central hub",
        )
        ctx.expect_gap(
            wing,
            housing,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="floor_plinth",
            min_gap=0.0,
            max_gap=0.08,
            name=f"{wing.name} clears the floor slab",
        )
        ctx.expect_within(
            wing,
            housing,
            axes="xy",
            margin=0.05,
            name=f"{wing.name} stays within the drum footprint",
        )

    with ctx.pose({rotor_spin: math.pi / 4.0}):
        for wing in wings:
            ctx.expect_within(
                wing,
                housing,
                axes="xy",
                margin=0.05,
                name=f"{wing.name} stays within the drum footprint at 45 degrees",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
