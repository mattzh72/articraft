from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_bezel_mesh(
    *,
    width: float,
    height: float,
    corner_radius: float,
    opening_radius: float,
    fan_center_x: float,
    thickness: float,
) -> MeshGeometry:
    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=10)
    holes = [
        _circle_profile(opening_radius, center=(-fan_center_x, 0.0), segments=40),
        _circle_profile(opening_radius, center=(fan_center_x, 0.0), segments=40),
    ]
    return ExtrudeWithHolesGeometry(
        outer,
        holes,
        thickness,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(math.pi / 2.0)


def _build_rotor_mesh(
    *,
    outer_radius: float,
    hub_radius: float,
    hub_length: float,
    blade_count: int,
) -> MeshGeometry:
    hub = CylinderGeometry(radius=hub_radius, height=hub_length, radial_segments=40).rotate_x(
        math.pi / 2.0
    )
    rear_collar = CylinderGeometry(
        radius=hub_radius * 0.62,
        height=0.012,
        radial_segments=32,
    ).rotate_x(math.pi / 2.0).translate(0.0, -0.012, 0.0)
    front_cap = CylinderGeometry(
        radius=hub_radius * 0.55,
        height=0.012,
        radial_segments=32,
    ).rotate_x(math.pi / 2.0).translate(0.0, 0.014, 0.0)
    shaft = CylinderGeometry(radius=0.0075, height=0.028, radial_segments=24).rotate_x(
        math.pi / 2.0
    ).translate(0.0, -0.012, 0.0)

    blade_profile = [
        (hub_radius * 0.62, -0.011),
        (hub_radius * 0.90, -0.020),
        (outer_radius * 0.64, -0.017),
        (outer_radius * 0.94, -0.007),
        (outer_radius, 0.0),
        (outer_radius * 0.88, 0.013),
        (outer_radius * 0.48, 0.020),
        (hub_radius * 0.74, 0.013),
    ]
    blade = ExtrudeGeometry(
        blade_profile,
        0.0042,
        cap=True,
        center=True,
        closed=True,
    )
    blade.rotate_x(math.pi / 2.0)
    blade.rotate_y(math.radians(-6.0))
    blade.rotate_x(math.radians(22.0))
    blade.rotate_z(math.radians(14.0))

    blades = MeshGeometry()
    for index in range(blade_count):
        blades.merge(blade.copy().rotate_y((2.0 * math.pi * index) / blade_count))

    return _merge_meshes(hub, rear_collar, front_cap, shaft, blades)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_box_fan")

    housing_width = 0.620
    housing_height = 0.310
    housing_depth = 0.110
    bezel_thickness = 0.008
    side_wall_width = 0.050
    top_bottom_band = 0.035
    divider_width = 0.040
    fan_center_x = 0.140
    fan_center_z = housing_height * 0.5
    opening_radius = 0.120
    support_center_y = -0.040

    body_plastic = model.material("body_plastic", rgba=(0.90, 0.90, 0.86, 1.0))
    grille_plastic = model.material("grille_plastic", rgba=(0.80, 0.81, 0.78, 1.0))
    blade_grey = model.material("blade_grey", rgba=(0.34, 0.36, 0.39, 1.0))
    motor_grey = model.material("motor_grey", rgba=(0.26, 0.28, 0.30, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pointer_orange = model.material("pointer_orange", rgba=(0.88, 0.45, 0.16, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height + 0.020)),
        mass=3.9,
        origin=Origin(xyz=(0.0, 0.0, (housing_height + 0.020) * 0.5)),
    )

    front_bezel_mesh = mesh_from_geometry(
        _build_bezel_mesh(
            width=housing_width,
            height=housing_height,
            corner_radius=0.018,
            opening_radius=opening_radius,
            fan_center_x=fan_center_x,
            thickness=bezel_thickness,
        ),
        "fan_front_bezel",
    )
    rear_bezel_mesh = mesh_from_geometry(
        _build_bezel_mesh(
            width=housing_width,
            height=housing_height,
            corner_radius=0.018,
            opening_radius=opening_radius,
            fan_center_x=fan_center_x,
            thickness=bezel_thickness,
        ),
        "fan_rear_bezel",
    )

    housing.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.0, (housing_depth * 0.5) - (bezel_thickness * 0.5), fan_center_z)),
        material=body_plastic,
        name="front_bezel",
    )
    housing.visual(
        rear_bezel_mesh,
        origin=Origin(xyz=(0.0, -(housing_depth * 0.5) + (bezel_thickness * 0.5), fan_center_z)),
        material=body_plastic,
        name="rear_bezel",
    )

    housing.visual(
        Box((side_wall_width, housing_depth - (2.0 * bezel_thickness), housing_height)),
        origin=Origin(
            xyz=(
                -(housing_width * 0.5) + (side_wall_width * 0.5),
                0.0,
                fan_center_z,
            )
        ),
        material=body_plastic,
        name="left_wall",
    )
    housing.visual(
        Box((side_wall_width, housing_depth - (2.0 * bezel_thickness), housing_height)),
        origin=Origin(
            xyz=(
                (housing_width * 0.5) - (side_wall_width * 0.5),
                0.0,
                fan_center_z,
            )
        ),
        material=body_plastic,
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - (2.0 * side_wall_width), housing_depth - (2.0 * bezel_thickness), top_bottom_band)),
        origin=Origin(xyz=(0.0, 0.0, top_bottom_band * 0.5)),
        material=body_plastic,
        name="bottom_band",
    )
    housing.visual(
        Box((housing_width - (2.0 * side_wall_width), housing_depth - (2.0 * bezel_thickness), top_bottom_band)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - (top_bottom_band * 0.5))),
        material=body_plastic,
        name="top_band",
    )
    housing.visual(
        Box((divider_width, housing_depth - (2.0 * bezel_thickness), housing_height)),
        origin=Origin(xyz=(0.0, 0.0, fan_center_z)),
        material=body_plastic,
        name="center_divider",
    )

    control_pod = housing.visual(
        Box((0.160, 0.075, 0.018)),
        origin=Origin(xyz=(0.205, 0.0, housing_height + 0.009)),
        material=body_plastic,
        name="control_pod",
    )
    housing.visual(
        Box((0.185, 0.020, 0.006)),
        origin=Origin(xyz=(0.205, 0.0, housing_height + 0.003)),
        material=grille_plastic,
        name="control_pod_skirt",
    )

    for side_name, x_center in (("left", -fan_center_x), ("right", fan_center_x)):
        housing.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(x_center, support_center_y, fan_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=motor_grey,
            name=f"{side_name}_support_post",
        )
        housing.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(
                xyz=(x_center, support_center_y - 0.004, fan_center_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=motor_grey,
            name=f"{side_name}_motor_cap",
        )

        guard_y_positions = (
            ("front", (housing_depth * 0.5) - bezel_thickness - 0.003),
            ("rear", -(housing_depth * 0.5) + bezel_thickness + 0.003),
        )
        for side_prefix, guard_y in guard_y_positions:
            for index, x_offset in enumerate((-0.048, 0.0, 0.048)):
                housing.visual(
                    Box((0.010, 0.006, 0.248)),
                    origin=Origin(xyz=(x_center + x_offset, guard_y, fan_center_z)),
                    material=grille_plastic,
                    name=f"{side_name}_{side_prefix}_vertical_{index}",
                )
            for index, z_offset in enumerate((-0.046, 0.0, 0.046)):
                housing.visual(
                    Box((0.248, 0.006, 0.010)),
                    origin=Origin(xyz=(x_center, guard_y, fan_center_z + z_offset)),
                    material=grille_plastic,
                    name=f"{side_name}_{side_prefix}_horizontal_{index}",
                )

    rotor_mesh = mesh_from_geometry(
        _build_rotor_mesh(
            outer_radius=0.104,
            hub_radius=0.032,
            hub_length=0.024,
            blade_count=5,
        ),
        "fan_rotor",
    )

    left_rotor = model.part("left_rotor")
    left_rotor.visual(rotor_mesh, material=blade_grey, name="left_rotor_mesh")
    left_rotor.inertial = Inertial.from_geometry(
        Box((0.220, 0.050, 0.220)),
        mass=0.28,
        origin=Origin(),
    )

    right_rotor = model.part("right_rotor")
    right_rotor.visual(rotor_mesh, material=blade_grey, name="right_rotor_mesh")
    right_rotor.inertial = Inertial.from_geometry(
        Box((0.220, 0.050, 0.220)),
        mass=0.28,
        origin=Origin(),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=knob_black,
        name="knob_base",
    )
    control_knob.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=knob_black,
        name="knob_body",
    )
    control_knob.visual(
        Box((0.004, 0.014, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, 0.0235)),
        material=pointer_orange,
        name="knob_pointer",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.028)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "housing_to_left_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_rotor,
        origin=Origin(xyz=(-fan_center_x, 0.0, fan_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )
    model.articulation(
        "housing_to_right_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_rotor,
        origin=Origin(xyz=(fan_center_x, 0.0, fan_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )
    model.articulation(
        "housing_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=control_knob,
        origin=place_on_surface(
            control_knob,
            control_pod,
            point_hint=(0.205, 0.0, housing_height + 0.020),
            child_axis="+z",
            clearance=0.0,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=3.5,
            lower=-0.15,
            upper=math.radians(275.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")
    control_knob = object_model.get_part("control_knob")

    left_spin = object_model.get_articulation("housing_to_left_rotor")
    right_spin = object_model.get_articulation("housing_to_right_rotor")
    knob_joint = object_model.get_articulation("housing_to_control_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_rotor, housing, elem_b="left_support_post")
    ctx.expect_contact(right_rotor, housing, elem_b="right_support_post")
    ctx.expect_contact(control_knob, housing, elem_b="control_pod")

    ctx.expect_gap(
        housing,
        left_rotor,
        axis="y",
        positive_elem="front_bezel",
        min_gap=0.020,
        name="left_rotor_front_clearance",
    )
    ctx.expect_gap(
        left_rotor,
        housing,
        axis="y",
        negative_elem="rear_bezel",
        min_gap=0.018,
        name="left_rotor_rear_clearance",
    )
    ctx.expect_gap(
        housing,
        right_rotor,
        axis="y",
        positive_elem="front_bezel",
        min_gap=0.020,
        name="right_rotor_front_clearance",
    )
    ctx.expect_gap(
        right_rotor,
        housing,
        axis="y",
        negative_elem="rear_bezel",
        min_gap=0.018,
        name="right_rotor_rear_clearance",
    )

    ctx.check(
        "rotor_joint_axes",
        left_spin.axis == (0.0, 1.0, 0.0) and right_spin.axis == (0.0, 1.0, 0.0),
        f"left={left_spin.axis}, right={right_spin.axis}",
    )
    ctx.check(
        "knob_joint_axis",
        knob_joint.axis == (0.0, 0.0, 1.0),
        f"axis={knob_joint.axis}",
    )
    ctx.check(
        "rotor_joint_types_and_limits",
        left_spin.motion_limits is not None
        and right_spin.motion_limits is not None
        and left_spin.motion_limits.lower is None
        and left_spin.motion_limits.upper is None
        and right_spin.motion_limits.lower is None
        and right_spin.motion_limits.upper is None,
        "Rotor joints should be continuous without lower/upper bounds.",
    )

    with ctx.pose({left_spin: 1.15, right_spin: -0.85}):
        ctx.expect_contact(left_rotor, housing, elem_b="left_support_post")
        ctx.expect_contact(right_rotor, housing, elem_b="right_support_post")
        ctx.expect_gap(
            housing,
            left_rotor,
            axis="y",
            positive_elem="front_bezel",
            min_gap=0.020,
            name="left_rotor_front_clearance_spun",
        )
        ctx.expect_gap(
            housing,
            right_rotor,
            axis="y",
            positive_elem="front_bezel",
            min_gap=0.020,
            name="right_rotor_front_clearance_spun",
        )

    with ctx.pose({knob_joint: math.radians(120.0)}):
        ctx.expect_contact(control_knob, housing, elem_b="control_pod")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
