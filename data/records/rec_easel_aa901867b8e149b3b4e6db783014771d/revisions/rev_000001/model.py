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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_tube(outer_radius: float, inner_radius: float, length: float, *, segments: int = 40):
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _crown_plate_profile() -> list[tuple[float, float]]:
    return [
        (0.086, 0.000),
        (0.040, 0.055),
        (-0.052, 0.072),
        (-0.082, 0.000),
        (-0.052, -0.072),
        (0.040, -0.055),
    ]


def _leg_mount(phi: float, radius: float = 0.053) -> tuple[float, float, float]:
    return (radius * math.cos(phi), radius * math.sin(phi), -0.014)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plein_air_travel_easel")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.10, 1.0))

    crown_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_crown_plate_profile(), 0.010, cap=True),
        "crown_plate",
    )
    upper_tube_mesh = mesh_from_geometry(_shell_tube(0.0110, 0.0095, 0.88), "upper_leg_tube")
    hinge_sleeve_mesh = mesh_from_geometry(_shell_tube(0.0090, 0.0060, 0.040), "hinge_sleeve")
    guide_sleeve_mesh = mesh_from_geometry(_shell_tube(0.0095, 0.0087, 0.180), "guide_sleeve")

    crown = model.part("crown")
    crown.visual(crown_plate_mesh, material=dark_polymer, name="crown_plate")
    crown.visual(
        Box((0.034, 0.034, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=dark_polymer,
        name="mast_socket",
    )
    crown.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_polymer,
        name="center_boss",
    )

    leg_specs = [
        ("front", 0.0, math.radians(20.0)),
        ("rear_left", math.radians(140.0), math.radians(24.0)),
        ("rear_right", math.radians(-140.0), math.radians(24.0)),
    ]
    for name, phi, _tilt in leg_specs:
        mx, my, mz = _leg_mount(phi)
        crown.visual(
            Box((0.024, 0.018, 0.020)),
            origin=Origin(xyz=(mx, my, mz)),
            material=dark_polymer,
            name=f"{name}_hinge_block",
        )
        crown.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(
                xyz=(mx, my, mz - 0.002),
                rpy=(-math.pi / 2.0, 0.0, phi),
            ),
            material=aluminum,
            name=f"{name}_hinge_pin",
        )

    crown.visual(
        Box((0.026, 0.020, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, -0.022)),
        material=dark_polymer,
        name="clip_hinge_block",
    )
    crown.visual(
        Cylinder(radius=0.0055, length=0.044),
        origin=Origin(xyz=(-0.018, 0.0, -0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="clip_hinge_pin",
    )
    crown.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.08)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    for name, _phi, _tilt in leg_specs:
        upper_leg = model.part(f"{name}_upper_leg")
        upper_leg.visual(
            upper_tube_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.44)),
            material=aluminum,
            name="tube_shell",
        )
        upper_leg.visual(
            hinge_sleeve_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hinge_sleeve",
        )
        upper_leg.visual(
            Box((0.018, 0.020, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=dark_polymer,
            name="hinge_yoke",
        )
        upper_leg.visual(
            Cylinder(radius=0.013, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.82)),
            material=dark_polymer,
            name="twist_lock",
        )
        upper_leg.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.92)),
            mass=0.42,
            origin=Origin(xyz=(0.0, 0.0, -0.44)),
        )

        lower_leg = model.part(f"{name}_lower_leg")
        lower_leg.visual(
            Cylinder(radius=0.0087, length=0.78),
            origin=Origin(xyz=(0.0, 0.0, -0.39)),
            material=aluminum,
            name="inner_tube",
        )
        lower_leg.visual(
            guide_sleeve_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.09)),
            material=aluminum,
            name="guide_sleeve",
        )
        lower_leg.visual(
            Cylinder(radius=0.013, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.764)),
            material=rubber,
            name="foot",
        )
        lower_leg.inertial = Inertial.from_geometry(
            Cylinder(radius=0.010, length=0.82),
            mass=0.28,
            origin=Origin(xyz=(0.0, 0.0, -0.41)),
        )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        mesh_from_geometry(_shell_tube(0.0080, 0.0055, 0.034), "clip_hinge_sleeve"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_sleeve",
    )
    clip_bar.visual(
        Box((0.014, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=aluminum,
        name="hanger_link",
    )
    clip_bar.visual(
        Box((0.018, 0.010, 0.360)),
        origin=Origin(xyz=(0.0, 0.0, -0.228)),
        material=aluminum,
        name="bar_body",
    )
    clip_bar.visual(
        Box((0.072, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.455)),
        material=dark_polymer,
        name="clip_pad",
    )
    clip_bar.visual(
        Box((0.034, 0.020, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -0.428)),
        material=dark_polymer,
        name="clip_body",
    )
    clip_bar.inertial = Inertial.from_geometry(
        Box((0.08, 0.03, 0.47)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.232)),
    )

    for name, phi, tilt in leg_specs:
        upper_leg = model.get_part(f"{name}_upper_leg")
        lower_leg = model.get_part(f"{name}_lower_leg")

        model.articulation(
            f"{name}_crown_hinge",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=upper_leg,
            origin=Origin(
                xyz=_leg_mount(phi),
                rpy=(0.0, -tilt, phi),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.0,
                lower=-0.12,
                upper=tilt - math.radians(5.0),
            ),
        )

        model.articulation(
            f"{name}_leg_extension",
            ArticulationType.PRISMATIC,
            parent=upper_leg,
            child=lower_leg,
            origin=Origin(xyz=(0.0, 0.0, -0.52)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.10,
                lower=0.0,
                upper=0.22,
            ),
        )

    model.articulation(
        "clip_bar_hinge",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=clip_bar,
        origin=Origin(xyz=(-0.018, 0.0, -0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-0.20,
            upper=1.10,
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

    crown = object_model.get_part("crown")
    clip_bar = object_model.get_part("clip_bar")
    front_upper = object_model.get_part("front_upper_leg")
    rear_left_upper = object_model.get_part("rear_left_upper_leg")
    rear_right_upper = object_model.get_part("rear_right_upper_leg")
    front_lower = object_model.get_part("front_lower_leg")
    rear_left_lower = object_model.get_part("rear_left_lower_leg")
    rear_right_lower = object_model.get_part("rear_right_lower_leg")

    front_hinge = object_model.get_articulation("front_crown_hinge")
    rear_left_hinge = object_model.get_articulation("rear_left_crown_hinge")
    rear_right_hinge = object_model.get_articulation("rear_right_crown_hinge")
    front_extend = object_model.get_articulation("front_leg_extension")
    rear_left_extend = object_model.get_articulation("rear_left_leg_extension")
    rear_right_extend = object_model.get_articulation("rear_right_leg_extension")
    clip_hinge = object_model.get_articulation("clip_bar_hinge")

    ctx.allow_overlap(
        crown,
        front_upper,
        reason="Front leg hinge sleeve is captured inside the molded crown clevis.",
    )
    ctx.allow_overlap(
        crown,
        rear_left_upper,
        reason="Rear-left leg hinge sleeve is captured inside the molded crown clevis.",
    )
    ctx.allow_overlap(
        crown,
        rear_right_upper,
        reason="Rear-right leg hinge sleeve is captured inside the molded crown clevis.",
    )
    ctx.allow_overlap(
        crown,
        clip_bar,
        reason="Clip bar hinge sleeve wraps the crown hinge pin inside the clip bracket.",
    )
    ctx.allow_overlap(
        front_upper,
        front_lower,
        reason="The front telescoping lower tube rides inside the upper aluminum leg shell.",
    )
    ctx.allow_overlap(
        rear_left_upper,
        rear_left_lower,
        reason="The rear-left telescoping lower tube rides inside the upper aluminum leg shell.",
    )
    ctx.allow_overlap(
        rear_right_upper,
        rear_right_lower,
        reason="The rear-right telescoping lower tube rides inside the upper aluminum leg shell.",
    )

    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("part_count", len(object_model.parts) == 8, f"expected 8 parts, found {len(object_model.parts)}")
    ctx.check(
        "articulation_count",
        len(object_model.articulations) == 7,
        f"expected 7 articulations, found {len(object_model.articulations)}",
    )

    for joint_name in (
        "front_crown_hinge",
        "rear_left_crown_hinge",
        "rear_right_crown_hinge",
        "clip_bar_hinge",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name}_axis", tuple(joint.axis) == (0.0, 1.0, 0.0), f"{joint_name} axis is {joint.axis}")

    for joint_name in ("front_leg_extension", "rear_left_leg_extension", "rear_right_leg_extension"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name}_axis", tuple(joint.axis) == (0.0, 0.0, -1.0), f"{joint_name} axis is {joint.axis}")

    for a, b, name in (
        (crown, front_upper, "front_hinge_contact"),
        (crown, rear_left_upper, "rear_left_hinge_contact"),
        (crown, rear_right_upper, "rear_right_hinge_contact"),
        (front_upper, front_lower, "front_telescoping_contact"),
        (rear_left_upper, rear_left_lower, "rear_left_telescoping_contact"),
        (rear_right_upper, rear_right_lower, "rear_right_telescoping_contact"),
        (crown, clip_bar, "clip_bar_hinge_contact"),
    ):
        ctx.expect_contact(a, b, name=name)

    crown_aabb = ctx.part_world_aabb(crown)
    front_lower_aabb = ctx.part_world_aabb(front_lower)
    rear_left_lower_aabb = ctx.part_world_aabb(rear_left_lower)
    rear_right_lower_aabb = ctx.part_world_aabb(rear_right_lower)
    assert crown_aabb is not None
    assert front_lower_aabb is not None
    assert rear_left_lower_aabb is not None
    assert rear_right_lower_aabb is not None

    def _center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _center_y(aabb):
        return 0.5 * (aabb[0][1] + aabb[1][1])

    travel_height = crown_aabb[1][2] - min(
        front_lower_aabb[0][2],
        rear_left_lower_aabb[0][2],
        rear_right_lower_aabb[0][2],
    )
    ctx.check(
        "easel_height_realistic",
        1.10 <= travel_height <= 1.55,
        f"overall height is {travel_height:.3f} m",
    )
    ctx.check(
        "front_leg_forward",
        _center_x(front_lower_aabb) > 0.18,
        f"front leg x center is {_center_x(front_lower_aabb):.3f}",
    )
    ctx.check(
        "rear_legs_behind_crown",
        _center_x(rear_left_lower_aabb) < -0.06 and _center_x(rear_right_lower_aabb) < -0.06,
        f"rear leg x centers are {_center_x(rear_left_lower_aabb):.3f}, {_center_x(rear_right_lower_aabb):.3f}",
    )
    ctx.check(
        "rear_legs_split_sideways",
        _center_y(rear_left_lower_aabb) > 0.10 and _center_y(rear_right_lower_aabb) < -0.10,
        f"rear leg y centers are {_center_y(rear_left_lower_aabb):.3f}, {_center_y(rear_right_lower_aabb):.3f}",
    )

    clip_rest_aabb = ctx.part_world_aabb(clip_bar)
    clip_bar_body_aabb = ctx.part_element_world_aabb(clip_bar, elem="bar_body")
    assert clip_rest_aabb is not None
    assert clip_bar_body_aabb is not None
    ctx.check(
        "clip_bar_hangs_below_crown",
        clip_bar_body_aabb[1][2] < crown_aabb[0][2] + 0.01
        and clip_bar_body_aabb[0][2] < crown_aabb[0][2] - 0.35,
        f"clip bar body aabb {clip_bar_body_aabb} vs crown underside {crown_aabb[0][2]:.3f}",
    )

    front_extend_limits = front_extend.motion_limits
    rear_left_extend_limits = rear_left_extend.motion_limits
    rear_right_extend_limits = rear_right_extend.motion_limits
    front_hinge_limits = front_hinge.motion_limits
    clip_limits = clip_hinge.motion_limits
    assert front_extend_limits is not None
    assert rear_left_extend_limits is not None
    assert rear_right_extend_limits is not None
    assert front_hinge_limits is not None
    assert clip_limits is not None
    assert front_extend_limits.upper is not None
    assert rear_left_extend_limits.upper is not None
    assert rear_right_extend_limits.upper is not None
    assert front_hinge_limits.upper is not None
    assert clip_limits.upper is not None

    front_lower_pos_rest = ctx.part_world_position(front_lower)
    assert front_lower_pos_rest is not None
    with ctx.pose({front_extend: front_extend_limits.upper}):
        front_lower_pos_extended = ctx.part_world_position(front_lower)
        assert front_lower_pos_extended is not None
        ctx.check(
            "front_leg_extension_motion",
            front_lower_pos_extended[2] < front_lower_pos_rest[2] - 0.18
            and front_lower_pos_extended[0] > front_lower_pos_rest[0] + 0.05,
            f"front lower leg moved from {front_lower_pos_rest} to {front_lower_pos_extended}",
        )
        ctx.expect_contact(front_upper, front_lower, name="front_extension_contact_at_max")

    rear_left_pos_rest = ctx.part_world_position(rear_left_lower)
    rear_right_pos_rest = ctx.part_world_position(rear_right_lower)
    assert rear_left_pos_rest is not None
    assert rear_right_pos_rest is not None
    with ctx.pose(
        {
            rear_left_extend: rear_left_extend_limits.upper,
            rear_right_extend: rear_right_extend_limits.upper,
        }
    ):
        rear_left_pos_extended = ctx.part_world_position(rear_left_lower)
        rear_right_pos_extended = ctx.part_world_position(rear_right_lower)
        assert rear_left_pos_extended is not None
        assert rear_right_pos_extended is not None
        ctx.check(
            "rear_legs_extend_downward",
            rear_left_pos_extended[2] < rear_left_pos_rest[2] - 0.16
            and rear_right_pos_extended[2] < rear_right_pos_rest[2] - 0.16,
            f"rear leg positions moved to {rear_left_pos_extended} and {rear_right_pos_extended}",
        )
        ctx.expect_contact(rear_left_upper, rear_left_lower, name="rear_left_extension_contact_at_max")
        ctx.expect_contact(rear_right_upper, rear_right_lower, name="rear_right_extension_contact_at_max")

    with ctx.pose({front_hinge: front_hinge_limits.upper}):
        front_lower_aabb_folded = ctx.part_world_aabb(front_lower)
        assert front_lower_aabb_folded is not None
        ctx.check(
            "front_leg_folds_toward_center",
            _center_x(front_lower_aabb_folded) < _center_x(front_lower_aabb) - 0.10
            and front_lower_aabb_folded[1][0] < front_lower_aabb[1][0] - 0.20,
            f"front leg rest aabb {front_lower_aabb} folded aabb {front_lower_aabb_folded}",
        )
        ctx.expect_contact(crown, front_upper, name="front_hinge_contact_folded")

    with ctx.pose({clip_hinge: clip_limits.upper}):
        clip_open_aabb = ctx.part_world_aabb(clip_bar)
        assert clip_open_aabb is not None
        ctx.check(
            "clip_bar_swings_through_arc",
            abs(_center_x(clip_open_aabb) - _center_x(clip_rest_aabb)) > 0.10
            and clip_open_aabb[0][2] > clip_rest_aabb[0][2] + 0.12,
            f"clip bar rest aabb {clip_rest_aabb} open aabb {clip_open_aabb}",
        )
        ctx.expect_contact(crown, clip_bar, name="clip_bar_contact_open")

    for joint in (
        front_hinge,
        rear_left_hinge,
        rear_right_hinge,
        front_extend,
        rear_left_extend,
        rear_right_extend,
        clip_hinge,
    ):
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
