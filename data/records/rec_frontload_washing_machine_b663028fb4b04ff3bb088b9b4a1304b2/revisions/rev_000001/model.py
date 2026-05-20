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
    CylinderGeometry,
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


def _lathed_ring_mesh(name: str, outer_profile, inner_profile):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
        ).rotate_x(math.pi / 2.0),
    )


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_front_load_laundry_machine")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    darker_stainless = model.material("darker_stainless", rgba=(0.62, 0.64, 0.67, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.70, 0.78, 0.28))
    control_black = model.material("control_black", rgba=(0.06, 0.07, 0.08, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.12, 0.10, 1.0))

    cabinet_width = 0.92
    cabinet_depth = 0.86
    cabinet_height = 1.30
    opening_z = 0.68

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.04, cabinet_depth, 1.15)),
        origin=Origin(xyz=(-0.44, 0.0, 0.665)),
        material=stainless,
        name="left_wall",
    )
    cabinet.visual(
        Box((0.04, cabinet_depth, 1.15)),
        origin=Origin(xyz=(0.44, 0.0, 0.665)),
        material=stainless,
        name="right_wall",
    )
    cabinet.visual(
        Box((0.84, 0.035, 1.15)),
        origin=Origin(xyz=(0.0, -0.4125, 0.665)),
        material=darker_stainless,
        name="rear_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - 0.03)),
        material=stainless,
        name="top_cap",
    )
    cabinet.visual(
        Box((0.84, 0.06, 0.33)),
        origin=Origin(xyz=(0.0, 0.40, 0.255)),
        material=stainless,
        name="front_lower_panel",
    )
    cabinet.visual(
        Box((0.12, 0.06, 0.76)),
        origin=Origin(xyz=(-0.36, 0.40, 0.74)),
        material=stainless,
        name="front_left_stile",
    )
    cabinet.visual(
        Box((0.12, 0.06, 0.76)),
        origin=Origin(xyz=(0.36, 0.40, 0.74)),
        material=stainless,
        name="front_right_stile",
    )
    cabinet.visual(
        Box((0.84, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, 0.40, 1.13)),
        material=stainless,
        name="front_header",
    )
    cabinet.visual(
        _save_mesh(
            "washer_front_shroud",
            CylinderGeometry(radius=0.31, height=0.15, radial_segments=56, closed=False).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.0, 0.355, opening_z)),
        material=charcoal,
        name="door_shroud",
    )
    cabinet.visual(
        _save_mesh(
            "washer_opening_trim",
            TorusGeometry(radius=0.335, tube=0.018, radial_segments=18, tubular_segments=72).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.0, 0.414, opening_z)),
        material=stainless,
        name="opening_trim",
    )
    cabinet.visual(
        _save_mesh(
            "washer_boot_rim",
            TorusGeometry(radius=0.31, tube=0.016, radial_segments=18, tubular_segments=72).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.0, 0.414, opening_z)),
        material=rubber,
        name="boot_rim",
    )
    cabinet.visual(
        Cylinder(radius=0.07, length=0.066),
        origin=Origin(xyz=(0.0, -0.406, opening_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_bearing_housing",
    )
    cabinet.visual(
        Box((0.31, 0.014, 0.08)),
        origin=Origin(xyz=(0.09, 0.437, 1.13)),
        material=control_black,
        name="control_display",
    )
    cabinet.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(-0.30, 0.436, 1.13), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=signal_red,
        name="emergency_stop",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(-0.22, 0.437, 1.13), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="selector_knob",
    )
    for hinge_index, hinge_z in enumerate((0.88, 0.49)):
        cabinet.visual(
            Box((0.065, 0.04, 0.16)),
            origin=Origin(xyz=(-0.395, 0.427, hinge_z)),
            material=darker_stainless,
            name=f"body_hinge_plate_{hinge_index}",
        )
        cabinet.visual(
            Cylinder(radius=0.015, length=0.16),
            origin=Origin(xyz=(-0.34, 0.445, hinge_z)),
            material=darker_stainless,
            name=f"body_hinge_knuckle_{hinge_index}",
        )
    cabinet.visual(
        Box((0.132, 0.018, 0.018)),
        origin=Origin(xyz=(0.29, 0.421, 0.040)),
        material=darker_stainless,
        name="filter_hinge_carrier",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.257, 0.423, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_stainless,
        name="filter_hinge_barrel_left",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.323, 0.423, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_stainless,
        name="filter_hinge_barrel_right",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        ((-0.35, 0.31), (0.35, 0.31), (-0.35, -0.31), (0.35, -0.31))
    ):
        cabinet.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(xyz=(foot_x, foot_y, 0.025)),
            material=charcoal,
            name=f"foot_{foot_index}",
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        _save_mesh(
            "washer_drum_shell",
            CylinderGeometry(radius=0.30, height=0.54, radial_segments=56, closed=False).rotate_x(
                math.pi / 2.0
            ),
        ),
        material=darker_stainless,
        name="drum_shell",
    )
    drum.visual(
        _save_mesh(
            "washer_drum_front_hoop",
            TorusGeometry(radius=0.286, tube=0.014, radial_segments=16, tubular_segments=64).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.0, 0.27, 0.0)),
        material=stainless,
        name="front_hoop",
    )
    drum.visual(
        _save_mesh(
            "washer_drum_rear_hoop",
            TorusGeometry(radius=0.286, tube=0.014, radial_segments=16, tubular_segments=64).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.0, -0.27, 0.0)),
        material=stainless,
        name="rear_hoop",
    )
    drum.visual(
        Cylinder(radius=0.29, length=0.006),
        origin=Origin(xyz=(0.0, -0.267, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_stainless,
        name="rear_wall",
    )
    drum.visual(
        Cylinder(radius=0.08, length=0.040),
        origin=Origin(xyz=(0.0, -0.247, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.035, length=0.106),
        origin=Origin(xyz=(0.0, -0.320, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="drive_shaft",
    )
    for lifter_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.058, 0.48, 0.026)),
            origin=Origin(
                xyz=(0.271 * math.cos(angle), 0.0, 0.271 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=stainless,
            name=f"drum_lifter_{lifter_index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=0.62),
        mass=24.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _lathed_ring_mesh(
            "washer_door_bezel",
            outer_profile=(
                (0.300, -0.040),
                (0.326, -0.028),
                (0.342, -0.010),
                (0.346, 0.010),
                (0.336, 0.028),
                (0.314, 0.040),
            ),
            inner_profile=(
                (0.228, -0.032),
                (0.236, -0.018),
                (0.244, 0.000),
                (0.240, 0.018),
                (0.232, 0.032),
            ),
        ),
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        material=stainless,
        name="door_bezel",
    )
    door.visual(
        _lathed_ring_mesh(
            "washer_door_inner_retainer",
            outer_profile=(
                (0.252, -0.022),
                (0.257, -0.010),
                (0.258, 0.010),
                (0.252, 0.022),
            ),
            inner_profile=(
                (0.218, -0.018),
                (0.221, -0.008),
                (0.221, 0.008),
                (0.218, 0.018),
            ),
        ),
        origin=Origin(xyz=(0.34, 0.008, 0.0)),
        material=darker_stainless,
        name="door_retainer",
    )
    door.visual(
        Cylinder(radius=0.221, length=0.030),
        origin=Origin(xyz=(0.34, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_window",
    )
    door.visual(
        _save_mesh(
            "washer_door_gasket",
            TorusGeometry(radius=0.31, tube=0.016, radial_segments=18, tubular_segments=72).rotate_x(
                math.pi / 2.0
            ),
        ),
        origin=Origin(xyz=(0.34, -0.026, 0.0)),
        material=rubber,
        name="door_gasket",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(0.0, 0.002, 0.175)),
        material=darker_stainless,
        name="door_hinge_knuckle_top",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(0.0, 0.002, -0.190)),
        material=darker_stainless,
        name="door_hinge_knuckle_bottom",
    )
    door.visual(
        Box((0.130, 0.020, 0.055)),
        origin=Origin(xyz=(0.065, 0.018, 0.175)),
        material=darker_stainless,
        name="door_hinge_arm_top",
    )
    door.visual(
        Box((0.130, 0.020, 0.055)),
        origin=Origin(xyz=(0.065, 0.018, -0.190)),
        material=darker_stainless,
        name="door_hinge_arm_bottom",
    )
    door.visual(
        Box((0.060, 0.048, 0.028)),
        origin=Origin(xyz=(0.598, 0.034, 0.030)),
        material=charcoal,
        name="door_handle_grip",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.048),
        origin=Origin(xyz=(0.615, 0.052, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="door_handle_front",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.70, 0.11, 0.72)),
        mass=8.5,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
    )

    filter_cap = model.part("lint_filter_cap")
    filter_cap.visual(
        Box((0.16, 0.022, 0.096)),
        origin=Origin(xyz=(0.0, 0.029, 0.061)),
        material=stainless,
        name="filter_cap_panel",
    )
    filter_cap.visual(
        Box((0.142, 0.022, 0.024)),
        origin=Origin(xyz=(0.0, 0.011, 0.012)),
        material=darker_stainless,
        name="filter_cap_lip",
    )
    filter_cap.visual(
        Box((0.075, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.046, 0.083)),
        material=charcoal,
        name="filter_cap_pull",
    )
    filter_cap.inertial = Inertial.from_geometry(
        Box((0.16, 0.048, 0.105)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.024, 0.0525)),
    )

    drum_joint = model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, opening_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=24.0),
    )
    door_joint = model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.34, 0.472, opening_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    filter_joint = model.articulation(
        "cabinet_to_filter_cap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=filter_cap,
        origin=Origin(xyz=(0.29, 0.430, 0.040)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    assert drum_joint is not None and door_joint is not None and filter_joint is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    filter_cap = object_model.get_part("lint_filter_cap")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    filter_joint = object_model.get_articulation("cabinet_to_filter_cap")

    boot_rim = cabinet.get_visual("boot_rim")
    bearing = cabinet.get_visual("rear_bearing_housing")
    door_gasket = door.get_visual("door_gasket")
    door_window = door.get_visual("door_window")
    drum_shaft = drum.get_visual("drive_shaft")
    drum_lifter = drum.get_visual("drum_lifter_0")
    filter_panel = filter_cap.get_visual("filter_cap_panel")

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

    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        positive_elem=door_gasket,
        negative_elem=boot_rim,
        min_gap=0.0,
        max_gap=0.001,
    )
    ctx.expect_contact(drum, cabinet, elem_a=drum_shaft, elem_b=bearing)
    ctx.expect_contact(filter_cap, cabinet)
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.0)
    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.62)

    assert drum_joint.axis == (0.0, 1.0, 0.0)
    assert door_joint.axis == (0.0, 0.0, 1.0)
    assert filter_joint.axis == (-1.0, 0.0, 0.0)

    door_window_rest = ctx.part_element_world_aabb(door, elem=door_window)
    assert door_window_rest is not None
    door_window_rest_center = _aabb_center(door_window_rest)
    with ctx.pose({door_joint: math.radians(100.0)}):
        door_window_open = ctx.part_element_world_aabb(door, elem=door_window)
        assert door_window_open is not None
        door_window_open_center = _aabb_center(door_window_open)
        assert door_window_open_center[0] < door_window_rest_center[0] - 0.22
        assert door_window_open_center[1] > door_window_rest_center[1] + 0.28

    filter_rest = ctx.part_element_world_aabb(filter_cap, elem=filter_panel)
    assert filter_rest is not None
    filter_rest_center = _aabb_center(filter_rest)
    with ctx.pose({filter_joint: math.radians(65.0)}):
        filter_open = ctx.part_element_world_aabb(filter_cap, elem=filter_panel)
        assert filter_open is not None
        filter_open_center = _aabb_center(filter_open)
        assert filter_open_center[1] > filter_rest_center[1] + 0.03
        assert filter_open_center[2] < filter_rest_center[2] - 0.04

    lifter_rest = ctx.part_element_world_aabb(drum, elem=drum_lifter)
    assert lifter_rest is not None
    lifter_rest_center = _aabb_center(lifter_rest)
    with ctx.pose({drum_joint: 1.25}):
        lifter_spun = ctx.part_element_world_aabb(drum, elem=drum_lifter)
        assert lifter_spun is not None
        lifter_spun_center = _aabb_center(lifter_spun)
        assert abs(lifter_spun_center[0] - lifter_rest_center[0]) > 0.06
        assert abs(lifter_spun_center[2] - lifter_rest_center[2]) > 0.06

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
