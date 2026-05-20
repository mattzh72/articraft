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


def _add_y_knuckles(
    part,
    *,
    x: float,
    z: float,
    y_centers: tuple[float, ...],
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    for index, y_center in enumerate(y_centers):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y_center, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_z_knuckles(
    part,
    *,
    x: float,
    y: float,
    z_centers: tuple[float, ...],
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    for index, z_center in enumerate(z_centers):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, z_center)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _build_door_ring_mesh():
    outer_profile = [
        (0.110, -0.008),
        (0.116, -0.004),
        (0.116, 0.004),
        (0.110, 0.008),
    ]
    inner_profile = [
        (0.094, -0.006),
        (0.094, 0.006),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ).rotate_x(math.pi / 2.0),
        "travel_dryer_door_ring",
    )


def _build_drum_mesh():
    outer_profile = [
        (0.016, -0.090),
        (0.110, -0.086),
        (0.114, -0.040),
        (0.114, 0.074),
        (0.108, 0.090),
    ]
    inner_profile = [
        (0.000, -0.080),
        (0.095, -0.074),
        (0.103, -0.028),
        (0.103, 0.078),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
        ).rotate_x(math.pi / 2.0),
        "travel_dryer_drum_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_dryer")

    shell_dark = model.material("shell_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    frame_silver = model.material("frame_silver", rgba=(0.73, 0.75, 0.77, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.11, 0.12, 0.13, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    mesh_fabric = model.material("mesh_fabric", rgba=(0.44, 0.46, 0.49, 0.92))
    glass = model.material("door_glass", rgba=(0.63, 0.77, 0.88, 0.28))
    gasket = model.material("gasket", rgba=(0.18, 0.19, 0.20, 1.0))

    door_ring_mesh = _build_door_ring_mesh()
    drum_mesh = _build_drum_mesh()

    rear_module = model.part("rear_module")
    rear_module.visual(
        Box((0.320, 0.170, 0.020)),
        origin=Origin(xyz=(0.000, 0.030, 0.010)),
        material=shell_dark,
        name="base_floor",
    )
    rear_module.visual(
        Box((0.304, 0.020, 0.024)),
        origin=Origin(xyz=(0.000, 0.115, 0.012)),
        material=frame_silver,
        name="bottom_rail",
    )
    rear_module.visual(
        Box((0.320, 0.060, 0.024)),
        origin=Origin(xyz=(0.000, -0.024, 0.294)),
        material=shell_dark,
        name="top_bridge",
    )
    rear_module.visual(
        Box((0.220, 0.050, 0.048)),
        origin=Origin(xyz=(0.000, -0.010, 0.306)),
        material=shell_dark,
        name="control_pod",
    )
    rear_module.visual(
        Box((0.056, 0.030, 0.036)),
        origin=Origin(xyz=(0.000, -0.040, 0.319)),
        material=shell_dark,
        name="upper_column",
    )
    rear_module.visual(
        Box((0.140, 0.012, 0.018)),
        origin=Origin(xyz=(0.000, 0.010, 0.330)),
        material=frame_silver,
        name="carry_handle",
    )
    rear_module.visual(
        Box((0.018, 0.030, 0.032)),
        origin=Origin(xyz=(-0.058, -0.002, 0.315)),
        material=frame_silver,
        name="handle_post_left",
    )
    rear_module.visual(
        Box((0.018, 0.030, 0.032)),
        origin=Origin(xyz=(0.058, -0.002, 0.315)),
        material=frame_silver,
        name="handle_post_right",
    )
    rear_module.visual(
        Box((0.040, 0.092, 0.264)),
        origin=Origin(xyz=(-0.150, -0.009, 0.152)),
        material=shell_dark,
        name="left_bearing_tower",
    )
    rear_module.visual(
        Box((0.040, 0.092, 0.264)),
        origin=Origin(xyz=(0.150, -0.009, 0.152)),
        material=shell_dark,
        name="right_bearing_tower",
    )
    rear_module.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.000, -0.070, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="drum_spindle",
    )
    for name, xyz in (
        ("foot_fl", (-0.110, 0.090, 0.004)),
        ("foot_fr", (0.110, 0.090, 0.004)),
        ("foot_rl", (-0.110, -0.010, 0.004)),
        ("foot_rr", (0.110, -0.010, 0.004)),
    ):
        rear_module.visual(
            Box((0.028, 0.028, 0.008)),
            origin=Origin(xyz=xyz),
            material=gasket,
            name=name,
        )
    rear_module.visual(
        Box((0.034, 0.020, 0.024)),
        origin=Origin(xyz=(-0.170, 0.108, 0.018)),
        material=frame_silver,
        name="bottom_left_hinge_block",
    )
    _add_y_knuckles(
        rear_module,
        x=-0.170,
        z=0.030,
        y_centers=(0.104, 0.116),
        radius=0.012,
        length=0.007,
        material=hinge_black,
        prefix="bottom_left_knuckle",
    )
    rear_module.inertial = Inertial.from_geometry(
        Box((0.400, 0.240, 0.340)),
        mass=6.5,
        origin=Origin(xyz=(0.000, -0.010, 0.170)),
    )

    left_frame = model.part("left_frame")
    left_frame.visual(
        Box((0.020, 0.020, 0.246)),
        origin=Origin(xyz=(0.024, 0.000, 0.123)),
        material=frame_silver,
        name="left_rail",
    )
    left_frame.visual(
        Box((0.004, 0.110, 0.200)),
        origin=Origin(xyz=(0.018, -0.055, 0.140)),
        material=mesh_fabric,
        name="left_mesh_panel",
    )
    left_frame.visual(
        Box((0.012, 0.016, 0.200)),
        origin=Origin(xyz=(0.012, 0.013, 0.140)),
        material=frame_silver,
        name="door_hinge_strap",
    )
    _add_y_knuckles(
        left_frame,
        x=0.008,
        z=0.260,
        y_centers=(-0.0075, 0.0075),
        radius=0.012,
        length=0.007,
        material=hinge_black,
        prefix="top_left_parent",
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((0.040, 0.130, 0.270)),
        mass=0.38,
        origin=Origin(xyz=(0.006, -0.030, 0.135)),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.300, 0.020, 0.024)),
        origin=Origin(xyz=(0.170, 0.000, 0.012)),
        material=frame_silver,
        name="top_rail",
    )
    top_frame.visual(
        Box((0.300, 0.100, 0.002)),
        origin=Origin(xyz=(0.170, -0.050, 0.000)),
        material=mesh_fabric,
        name="top_mesh_panel",
    )
    _add_y_knuckles(
        top_frame,
        x=0.016,
        z=0.000,
        y_centers=(0.000,),
        radius=0.012,
        length=0.008,
        material=hinge_black,
        prefix="top_left_mid",
    )
    _add_y_knuckles(
        top_frame,
        x=0.324,
        z=0.000,
        y_centers=(-0.0075, 0.0075),
        radius=0.012,
        length=0.007,
        material=hinge_black,
        prefix="top_right_parent",
    )
    top_frame.inertial = Inertial.from_geometry(
        Box((0.340, 0.120, 0.040)),
        mass=0.32,
        origin=Origin(xyz=(0.170, -0.030, 0.000)),
    )

    right_frame = model.part("right_frame")
    right_frame.visual(
        Box((0.020, 0.020, 0.216)),
        origin=Origin(xyz=(-0.024, 0.000, -0.122)),
        material=frame_silver,
        name="right_rail",
    )
    right_frame.visual(
        Box((0.004, 0.110, 0.200)),
        origin=Origin(xyz=(-0.018, -0.055, -0.140)),
        material=mesh_fabric,
        name="right_mesh_panel",
    )
    _add_y_knuckles(
        right_frame,
        x=-0.008,
        z=0.000,
        y_centers=(0.000,),
        radius=0.012,
        length=0.008,
        material=hinge_black,
        prefix="top_right_mid",
    )
    _add_y_knuckles(
        right_frame,
        x=-0.008,
        z=-0.260,
        y_centers=(-0.0075, 0.0075),
        radius=0.012,
        length=0.007,
        material=hinge_black,
        prefix="bottom_right_parent",
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((0.040, 0.130, 0.270)),
        mass=0.38,
        origin=Origin(xyz=(-0.006, -0.030, -0.135)),
    )

    bottom_right_corner = model.part("bottom_right_corner")
    bottom_right_corner.visual(
        Box((0.020, 0.010, 0.020)),
        origin=Origin(xyz=(-0.004, 0.020, -0.014)),
        material=frame_silver,
        name="corner_latch_tab",
    )
    _add_y_knuckles(
        bottom_right_corner,
        x=0.000,
        z=0.000,
        y_centers=(0.000,),
        radius=0.012,
        length=0.008,
        material=hinge_black,
        prefix="bottom_right_mid",
    )
    bottom_right_corner.inertial = Inertial.from_geometry(
        Box((0.026, 0.024, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(-0.004, 0.000, -0.010)),
    )

    drum = model.part("drum")
    drum.visual(
        drum_mesh,
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.000, -0.078, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="rear_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.034, 0.150, 0.018)),
            origin=Origin(
                xyz=(0.000, 0.000, 0.095),
                rpy=(0.0, angle, 0.0),
            ),
            material=drum_steel,
            name=f"paddle_{index}",
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.180),
        mass=1.4,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(0.132, 0.000, 0.000)),
        material=frame_silver,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.094, length=0.004),
        origin=Origin(xyz=(0.132, -0.001, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.022, 0.012, 0.045)),
        origin=Origin(xyz=(0.216, 0.006, 0.000)),
        material=gasket,
        name="door_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.236, 0.022, 0.236)),
        mass=0.32,
        origin=Origin(xyz=(0.132, 0.000, 0.000)),
    )

    model.articulation(
        "rear_module_to_left_frame",
        ArticulationType.REVOLUTE,
        parent=rear_module,
        child=left_frame,
        origin=Origin(xyz=(-0.170, 0.115, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=-1.55,
            upper=0.15,
        ),
    )
    model.articulation(
        "left_frame_to_top_frame",
        ArticulationType.REVOLUTE,
        parent=left_frame,
        child=top_frame,
        origin=Origin(xyz=(0.000, 0.000, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=-1.55,
            upper=0.15,
        ),
    )
    model.articulation(
        "top_frame_to_right_frame",
        ArticulationType.REVOLUTE,
        parent=top_frame,
        child=right_frame,
        origin=Origin(xyz=(0.340, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=-1.55,
            upper=0.15,
        ),
    )
    model.articulation(
        "right_frame_to_bottom_right_corner",
        ArticulationType.REVOLUTE,
        parent=right_frame,
        child=bottom_right_corner,
        origin=Origin(xyz=(0.000, 0.000, -0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-1.55,
            upper=0.20,
        ),
    )
    model.articulation(
        "rear_module_to_drum",
        ArticulationType.CONTINUOUS,
        parent=rear_module,
        child=drum,
        origin=Origin(xyz=(0.000, 0.010, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=10.0,
        ),
    )
    model.articulation(
        "left_frame_to_door",
        ArticulationType.REVOLUTE,
        parent=left_frame,
        child=door,
        origin=Origin(xyz=(0.000, 0.026, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.85,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
    ctx.allow_overlap(
        "drum",
        "rear_module",
        elem_a="rear_hub",
        elem_b="drum_spindle",
        reason="The drum's rear hub is intentionally modeled as a concentric bushing seated around the fixed spindle axle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    rear_module = object_model.get_part("rear_module")
    left_frame = object_model.get_part("left_frame")
    top_frame = object_model.get_part("top_frame")
    right_frame = object_model.get_part("right_frame")
    bottom_right_corner = object_model.get_part("bottom_right_corner")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")

    left_hinge = object_model.get_articulation("rear_module_to_left_frame")
    top_left_hinge = object_model.get_articulation("left_frame_to_top_frame")
    top_right_hinge = object_model.get_articulation("top_frame_to_right_frame")
    bottom_right_hinge = object_model.get_articulation("right_frame_to_bottom_right_corner")
    drum_spin = object_model.get_articulation("rear_module_to_drum")
    door_hinge = object_model.get_articulation("left_frame_to_door")

    for joint_name, joint in (
        ("rear_module_to_left_frame", left_hinge),
        ("left_frame_to_top_frame", top_left_hinge),
        ("top_frame_to_right_frame", top_right_hinge),
        ("right_frame_to_bottom_right_corner", bottom_right_hinge),
    ):
        ctx.check(
            f"{joint_name} axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint_name} axis was {joint.axis}, expected (0, 1, 0).",
        )
    ctx.check(
        "rear_module_to_drum axis",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0),
        f"drum axis was {drum_spin.axis}, expected front-to-back rotation.",
    )
    ctx.check(
        "left_frame_to_door axis",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"door axis was {door_hinge.axis}, expected vertical hinge axis.",
    )

    ctx.expect_contact(left_frame, rear_module, contact_tol=5e-4, name="left frame mounted to rear module")
    ctx.expect_contact(top_frame, left_frame, contact_tol=5e-4, name="top frame mounted to left frame")
    ctx.expect_contact(right_frame, top_frame, contact_tol=5e-4, name="right frame mounted to top frame")
    ctx.expect_contact(
        bottom_right_corner,
        right_frame,
        contact_tol=5e-4,
        name="bottom-right hinge knuckle contact",
    )
    ctx.expect_contact(door, left_frame, contact_tol=5e-4, name="door mounted to left frame")
    ctx.expect_contact(drum, rear_module, contact_tol=5e-4, name="drum seated on spindle")

    ctx.expect_gap(
        door,
        drum,
        axis="y",
        min_gap=0.006,
        max_gap=0.035,
        name="door clears drum face",
    )
    ctx.expect_overlap(
        door,
        drum,
        axes="xz",
        min_overlap=0.190,
        name="door centered over drum opening",
    )

    right_frame_rest = ctx.part_world_position(right_frame)
    assert right_frame_rest is not None
    with ctx.pose({top_left_hinge: -1.0}):
        right_frame_folded = ctx.part_world_position(right_frame)
        assert right_frame_folded is not None
        ctx.check(
            "top-left hinge folds frame chain",
            right_frame_folded[0] < right_frame_rest[0] - 0.12
            and abs(right_frame_folded[1] - right_frame_rest[1]) < 1e-5,
            f"right frame rest={right_frame_rest}, folded={right_frame_folded}",
        )

    door_rest_aabb = ctx.part_world_aabb(door)
    assert door_rest_aabb is not None
    with ctx.pose({door_hinge: 1.20}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.check(
            "door swings outward",
            door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.055,
            f"door rest aabb={door_rest_aabb}, open aabb={door_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
