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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _blade_mesh():
    blade_profile = [
        (-0.032, 0.0008),
        (-0.028, 0.0000),
        (0.053, 0.0000),
        (0.089, 0.0066),
        (0.046, 0.0088),
        (-0.032, 0.0088),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(blade_profile, 0.0008, cap=True, closed=True),
        "utility_knife_blade",
    )


def _lofted_rounded_block_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    top_scale: float = 0.72,
    mid_scale: float = 0.88,
    top_x_shift: float = 0.0,
) :
    base_radius = min(width, depth) * 0.18
    mid_radius = min(width * mid_scale, depth * mid_scale) * 0.18
    top_radius = min(width * top_scale, depth * top_scale) * 0.18

    def section(w: float, d: float, z: float, dx: float = 0.0):
        return [(x + dx, y, z) for x, y in rounded_rect_profile(w, d, radius=min(base_radius, w * 0.45, d * 0.45))]

    geom = LoftGeometry(
        [
            section(width, depth, 0.0, 0.0),
            section(width * mid_scale, depth * mid_scale, height * 0.55, top_x_shift * 0.45),
            section(width * top_scale, depth * top_scale, height, top_x_shift),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _slider_button_mesh():
    return _lofted_rounded_block_mesh(
        "utility_slider_button",
        width=0.018,
        depth=0.010,
        height=0.005,
        top_scale=0.68,
        mid_scale=0.86,
        top_x_shift=0.0012,
    )


def _rear_top_bridge_mesh():
    return _lofted_rounded_block_mesh(
        "utility_rear_top_bridge",
        width=0.062,
        depth=0.018,
        height=0.003,
        top_scale=0.86,
        mid_scale=0.93,
        top_x_shift=0.0015,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.92, 0.77, 0.16, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.12, 1.0))
    track_grey = model.material("track_grey", rgba=(0.40, 0.42, 0.44, 1.0))
    slider_black = model.material("slider_black", rgba=(0.14, 0.14, 0.15, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.44, 0.47, 0.50, 1.0))

    body_shell = model.part("body_shell")
    body_shell.visual(
        Box((0.168, 0.026, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=body_yellow,
        name="bottom_pan",
    )
    body_shell.visual(
        Box((0.156, 0.003, 0.016)),
        origin=Origin(xyz=(-0.002, -0.014, 0.010)),
        material=body_yellow,
        name="left_wall",
    )
    body_shell.visual(
        Box((0.156, 0.003, 0.016)),
        origin=Origin(xyz=(-0.002, 0.014, 0.010)),
        material=body_yellow,
        name="right_wall",
    )
    body_shell.visual(
        Box((0.124, 0.010, 0.004)),
        origin=Origin(xyz=(-0.006, -0.0105, 0.0165)),
        material=body_yellow,
        name="left_top_rail",
    )
    body_shell.visual(
        Box((0.124, 0.010, 0.004)),
        origin=Origin(xyz=(-0.006, 0.0105, 0.0165)),
        material=body_yellow,
        name="right_top_rail",
    )
    body_shell.visual(
        Box((0.010, 0.026, 0.016)),
        origin=Origin(xyz=(-0.079, 0.0, 0.010)),
        material=body_yellow,
        name="rear_cap",
    )
    body_shell.visual(
        Box((0.016, 0.003, 0.012)),
        origin=Origin(xyz=(0.076, -0.014, 0.011)),
        material=body_yellow,
        name="nose_left_cheek",
    )
    body_shell.visual(
        Box((0.016, 0.003, 0.012)),
        origin=Origin(xyz=(0.076, 0.014, 0.011)),
        material=body_yellow,
        name="nose_right_cheek",
    )
    body_shell.visual(
        Box((0.024, 0.008, 0.003)),
        origin=Origin(xyz=(0.067, -0.0095, 0.017)),
        material=body_yellow,
        name="nose_left_roof",
    )
    body_shell.visual(
        Box((0.024, 0.008, 0.003)),
        origin=Origin(xyz=(0.067, 0.0095, 0.017)),
        material=body_yellow,
        name="nose_right_roof",
    )
    body_shell.visual(
        Box((0.090, 0.0014, 0.008)),
        origin=Origin(xyz=(-0.008, -0.0159, 0.0095)),
        material=grip_black,
        name="left_grip",
    )
    body_shell.visual(
        Box((0.050, 0.0014, 0.008)),
        origin=Origin(xyz=(-0.034, 0.0159, 0.0095)),
        material=grip_black,
        name="right_rear_grip",
    )
    body_shell.visual(
        Box((0.030, 0.0014, 0.008)),
        origin=Origin(xyz=(0.056, 0.0159, 0.0095)),
        material=grip_black,
        name="right_front_grip",
    )
    body_shell.visual(
        _rear_top_bridge_mesh(),
        origin=Origin(xyz=(-0.043, 0.0, 0.0155)),
        material=body_yellow,
        name="rear_top_bridge",
    )
    body_shell.inertial = Inertial.from_geometry(
        Box((0.170, 0.031, 0.021)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.082, 0.013, 0.003)),
        origin=Origin(xyz=(0.041, 0.0, 0.0015)),
        material=track_grey,
        name="carrier_rail",
    )
    blade_carrier.visual(
        Box((0.026, 0.013, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.005)),
        material=track_grey,
        name="carrier_block",
    )
    blade_carrier.visual(
        Box((0.018, 0.011, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.004)),
        material=track_grey,
        name="front_shoe",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.082, 0.013, 0.010)),
        mass=0.03,
        origin=Origin(xyz=(0.041, 0.0, 0.005)),
    )

    blade = model.part("blade")
    blade.visual(
        _blade_mesh(),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="blade_plate",
    )
    for score_index, score_x in enumerate((-0.016, -0.003, 0.010, 0.023, 0.036)):
        blade.visual(
            Box((0.0005, 0.00018, 0.0080)),
            origin=Origin(
                xyz=(score_x, -0.00033, 0.0044),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"score_line_{score_index}",
        )
    blade.inertial = Inertial.from_geometry(
        Box((0.121, 0.0008, 0.0088)),
        mass=0.01,
        origin=Origin(xyz=(0.0285, 0.0, 0.0044)),
    )

    thumb_slider = model.part("thumb_slider")
    thumb_slider.visual(
        Box((0.005, 0.003, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=slider_black,
        name="slider_stem",
    )
    thumb_slider.visual(
        _slider_button_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=slider_black,
        name="slider_button",
    )
    for ridge_index, ridge_x in enumerate((-0.005, 0.0, 0.005)):
        thumb_slider.visual(
            Box((0.0014, 0.009, 0.0012)),
            origin=Origin(xyz=(ridge_x, 0.0, 0.0136)),
            material=dark_steel,
            name=f"slider_ridge_{ridge_index}",
        )
    thumb_slider.inertial = Inertial.from_geometry(
        Box((0.018, 0.010, 0.014)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    lock_wheel = model.part("lock_wheel")
    lock_wheel.visual(
        Cylinder(radius=0.007, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="wheel_disc",
    )
    lock_wheel.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_hub",
    )
    lock_wheel.visual(
        Box((0.0035, 0.0042, 0.0022)),
        origin=Origin(xyz=(0.0080, 0.0044, 0.0)),
        material=dark_steel,
        name="wheel_fin",
    )
    lock_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.007, length=0.005),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_carrier",
        ArticulationType.PRISMATIC,
        parent=body_shell,
        child=blade_carrier,
        origin=Origin(xyz=(-0.030, 0.0, 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.032,
        ),
    )
    model.articulation(
        "carrier_to_blade",
        ArticulationType.FIXED,
        parent=blade_carrier,
        child=blade,
        origin=Origin(xyz=(0.028, 0.0, 0.0032)),
    )
    model.articulation(
        "carrier_to_slider",
        ArticulationType.FIXED,
        parent=blade_carrier,
        child=thumb_slider,
        origin=Origin(xyz=(0.034, 0.0, 0.003)),
    )
    model.articulation(
        "body_to_lock_wheel",
        ArticulationType.REVOLUTE,
        parent=body_shell,
        child=lock_wheel,
        origin=Origin(xyz=(0.022, 0.0155, 0.0135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=8.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body_shell = object_model.get_part("body_shell")
    blade_carrier = object_model.get_part("blade_carrier")
    blade = object_model.get_part("blade")
    thumb_slider = object_model.get_part("thumb_slider")
    lock_wheel = object_model.get_part("lock_wheel")

    body_to_carrier = object_model.get_articulation("body_to_carrier")
    body_to_lock_wheel = object_model.get_articulation("body_to_lock_wheel")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "carrier_joint_type",
        body_to_carrier.articulation_type == ArticulationType.PRISMATIC,
        f"Expected PRISMATIC carrier joint, got {body_to_carrier.articulation_type!r}",
    )
    ctx.check(
        "carrier_joint_axis",
        tuple(body_to_carrier.axis) == (1.0, 0.0, 0.0),
        f"Expected carrier axis (1, 0, 0), got {body_to_carrier.axis!r}",
    )
    ctx.check(
        "wheel_joint_axis",
        tuple(body_to_lock_wheel.axis) == (0.0, 1.0, 0.0),
        f"Expected lock wheel axis (0, 1, 0), got {body_to_lock_wheel.axis!r}",
    )

    ctx.expect_contact(blade_carrier, body_shell, name="carrier_guided_contact")
    ctx.expect_contact(blade, blade_carrier, name="blade_seated_on_carrier")
    ctx.expect_contact(thumb_slider, blade_carrier, name="slider_attached_to_carrier")
    ctx.expect_contact(lock_wheel, body_shell, name="wheel_seated_on_shell")
    ctx.expect_within(blade, body_shell, axes="yz", margin=0.0012, name="blade_within_shell_opening")
    ctx.expect_within(thumb_slider, body_shell, axes="y", margin=0.0005, name="slider_centered_in_slot")

    body_aabb = ctx.part_world_aabb(body_shell)
    ctx.check(
        "body_size_realistic",
        body_aabb is not None
        and 0.160 <= (body_aabb[1][0] - body_aabb[0][0]) <= 0.172
        and 0.028 <= (body_aabb[1][1] - body_aabb[0][1]) <= 0.034
        and 0.018 <= (body_aabb[1][2] - body_aabb[0][2]) <= 0.022,
        f"Unexpected body AABB: {body_aabb!r}",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    carrier_limits = body_to_carrier.motion_limits
    assert carrier_limits is not None
    assert carrier_limits.lower is not None
    assert carrier_limits.upper is not None

    with ctx.pose({body_to_carrier: carrier_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="carrier_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="carrier_lower_no_floating")
        ctx.expect_contact(blade_carrier, body_shell, name="carrier_lower_contact")
        ctx.expect_contact(blade, blade_carrier, name="blade_lower_contact")
        ctx.expect_contact(thumb_slider, blade_carrier, name="slider_lower_contact")
        lower_body_aabb = ctx.part_world_aabb(body_shell)
        lower_blade_aabb = ctx.part_world_aabb(blade)
        lower_slider_pos = ctx.part_world_position(thumb_slider)
        lower_exposed = None
        if lower_body_aabb is not None and lower_blade_aabb is not None:
            lower_exposed = lower_blade_aabb[1][0] - lower_body_aabb[1][0]
        ctx.check(
            "blade_rest_exposure",
            lower_exposed is not None and 0.0005 <= lower_exposed <= 0.0030,
            f"Rest blade exposure should be slight, got {lower_exposed!r}",
        )

    with ctx.pose({body_to_carrier: carrier_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="carrier_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="carrier_upper_no_floating")
        ctx.expect_contact(blade_carrier, body_shell, name="carrier_upper_contact")
        ctx.expect_contact(blade, blade_carrier, name="blade_upper_contact")
        ctx.expect_contact(thumb_slider, blade_carrier, name="slider_upper_contact")
        upper_body_aabb = ctx.part_world_aabb(body_shell)
        upper_blade_aabb = ctx.part_world_aabb(blade)
        upper_slider_pos = ctx.part_world_position(thumb_slider)
        upper_exposed = None
        if upper_body_aabb is not None and upper_blade_aabb is not None:
            upper_exposed = upper_blade_aabb[1][0] - upper_body_aabb[1][0]
        ctx.check(
            "blade_extended_exposure",
            upper_exposed is not None and 0.028 <= upper_exposed <= 0.036,
            f"Extended blade exposure should be about one segment set, got {upper_exposed!r}",
        )

    ctx.check(
        "slider_advances_with_carrier",
        lower_slider_pos is not None
        and upper_slider_pos is not None
        and (upper_slider_pos[0] - lower_slider_pos[0]) >= 0.030,
        f"Slider positions did not advance enough: lower={lower_slider_pos!r}, upper={upper_slider_pos!r}",
    )

    wheel_limits = body_to_lock_wheel.motion_limits
    assert wheel_limits is not None
    assert wheel_limits.lower is not None
    assert wheel_limits.upper is not None

    with ctx.pose({body_to_lock_wheel: 0.0}):
        wheel_rest_aabb = ctx.part_world_aabb(lock_wheel)
        wheel_fin_rest_aabb = ctx.part_element_world_aabb(lock_wheel, elem="wheel_fin")
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_rest_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_rest_no_floating")
        ctx.expect_contact(lock_wheel, body_shell, name="wheel_rest_contact")

    with ctx.pose({body_to_lock_wheel: math.pi / 2.0}):
        wheel_turned_aabb = ctx.part_world_aabb(lock_wheel)
        wheel_fin_turned_aabb = ctx.part_element_world_aabb(lock_wheel, elem="wheel_fin")
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_turned_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_turned_no_floating")
        ctx.expect_contact(lock_wheel, body_shell, name="wheel_turned_contact")

    wheel_pose_changes = False
    if wheel_fin_rest_aabb is not None and wheel_fin_turned_aabb is not None:
        wheel_pose_changes = (
            abs(wheel_fin_turned_aabb[1][0] - wheel_fin_rest_aabb[1][0]) >= 0.002
            or abs(wheel_fin_turned_aabb[1][2] - wheel_fin_rest_aabb[1][2]) >= 0.002
            or abs(wheel_fin_turned_aabb[0][0] - wheel_fin_rest_aabb[0][0]) >= 0.002
            or abs(wheel_fin_turned_aabb[0][2] - wheel_fin_rest_aabb[0][2]) >= 0.002
        )
    ctx.check(
        "wheel_rotation_changes_pose",
        wheel_pose_changes,
        (
            "Wheel marker should move after rotation: "
            f"part_rest={wheel_rest_aabb!r}, part_turned={wheel_turned_aabb!r}, "
            f"marker_rest={wheel_fin_rest_aabb!r}, marker_turned={wheel_fin_turned_aabb!r}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
