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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_band_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
    segments: int = 72,
):
    half_depth = depth * 0.5
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half_depth),
            (outer_radius, half_depth),
        ],
        [
            (inner_radius, -half_depth),
            (inner_radius, half_depth),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _door_glass_geometry() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.148, 0.0),
            (0.146, 0.004),
            (0.138, 0.012),
            (0.120, 0.026),
            (0.092, 0.045),
            (0.050, 0.058),
            (0.0, 0.065),
        ],
        segments=72,
    )


def _drum_shell_geometry() -> LatheGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.060, -0.205),
            (0.150, -0.205),
            (0.215, -0.198),
            (0.238, -0.186),
            (0.244, -0.154),
            (0.244, 0.165),
            (0.252, 0.190),
        ],
        [
            (0.045, -0.193),
            (0.175, -0.184),
            (0.212, -0.166),
            (0.220, 0.151),
            (0.205, 0.178),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_dryer")

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.71, 0.74, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.46, 0.56, 0.62, 0.34))
    drum_steel = model.material("drum_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))

    cabinet_width = 0.60
    cabinet_depth = 0.58
    cabinet_height = 0.85
    door_center_z = 0.43

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_white,
        name="base_plinth",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - 0.0125)),
        material=body_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.022, cabinet_depth, 0.82)),
        origin=Origin(xyz=(-0.289, 0.0, 0.43)),
        material=body_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.022, cabinet_depth, 0.82)),
        origin=Origin(xyz=(0.289, 0.0, 0.43)),
        material=body_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((0.556, 0.020, 0.82)),
        origin=Origin(xyz=(0.0, -0.290, 0.43)),
        material=body_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.60, 0.028, 0.24)),
        origin=Origin(xyz=(0.0, 0.286, 0.12)),
        material=body_white,
        name="front_lower_panel",
    )
    cabinet.visual(
        Box((0.60, 0.028, 0.15)),
        origin=Origin(xyz=(0.0, 0.286, 0.775)),
        material=body_white,
        name="front_upper_panel",
    )
    cabinet.visual(
        Box((0.070, 0.028, 0.46)),
        origin=Origin(xyz=(-0.265, 0.286, 0.47)),
        material=body_white,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((0.070, 0.028, 0.46)),
        origin=Origin(xyz=(0.265, 0.286, 0.47)),
        material=body_white,
        name="front_right_jamb",
    )
    cabinet.visual(
        _mesh(
            "cabinet_front_seal_ring",
            _ring_band_geometry(outer_radius=0.235, inner_radius=0.168, depth=0.008),
        ),
        origin=Origin(
            xyz=(0.0, 0.296, door_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="front_seal_ring",
    )
    cabinet.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(
            xyz=(0.0, -0.269, door_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="rear_bearing_boss",
    )
    cabinet.visual(
        Box((0.52, 0.008, 0.078)),
        origin=Origin(xyz=(0.0, 0.302, 0.745)),
        material=panel_black,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.145, 0.006, 0.038)),
        origin=Origin(xyz=(-0.145, 0.304, 0.748)),
        material=glass_tint,
        name="display_window",
    )
    cabinet.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(
            xyz=(0.205, 0.295, 0.744),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="selector_knob",
    )
    cabinet.visual(
        Box((0.022, 0.010, 0.016)),
        origin=Origin(xyz=(0.060, 0.302, 0.744)),
        material=trim_silver,
        name="button_left",
    )
    cabinet.visual(
        Box((0.022, 0.010, 0.016)),
        origin=Origin(xyz=(0.095, 0.302, 0.744)),
        material=trim_silver,
        name="button_right",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    drum = model.part("drum")
    drum.visual(
        _mesh("drum_shell", _drum_shell_geometry()),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        _mesh(
            "drum_front_rim",
            TorusGeometry(radius=0.210, tube=0.010, radial_segments=16, tubular_segments=56),
        ),
        origin=Origin(
            xyz=(0.0, 0.176, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=drum_steel,
        name="front_rim",
    )
    drum.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(
            xyz=(0.0, -0.196, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=drum_steel,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.022, length=0.058),
        origin=Origin(
            xyz=(0.0, -0.225, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="spindle",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.25, length=0.42),
        mass=8.5,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _mesh(
            "door_frame_ring",
            _ring_band_geometry(outer_radius=0.225, inner_radius=0.160, depth=0.048),
        ),
        origin=Origin(
            xyz=(0.225, 0.034, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_white,
        name="door_frame",
    )
    door.visual(
        _mesh(
            "door_outer_trim",
            TorusGeometry(radius=0.175, tube=0.015, radial_segments=16, tubular_segments=56),
        ),
        origin=Origin(
            xyz=(0.225, 0.057, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="outer_trim",
    )
    door.visual(
        _mesh(
            "door_inner_ring",
            _ring_band_geometry(outer_radius=0.175, inner_radius=0.145, depth=0.010),
        ),
        origin=Origin(
            xyz=(0.225, 0.005, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="door_inner_ring",
    )
    door.visual(
        _mesh("door_glass_lens", _door_glass_geometry()),
        origin=Origin(
            xyz=(0.225, 0.007, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.034, 0.018, 0.290)),
        origin=Origin(xyz=(0.024, 0.028, 0.0)),
        material=trim_silver,
        name="hinge_strap",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.082),
        origin=Origin(xyz=(0.012, 0.030, 0.103)),
        material=trim_silver,
        name="hinge_barrel_upper",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.082),
        origin=Origin(xyz=(0.012, 0.030, -0.103)),
        material=trim_silver,
        name="hinge_barrel_lower",
    )
    door.visual(
        Box((0.020, 0.018, 0.056)),
        origin=Origin(xyz=(0.392, 0.042, 0.0)),
        material=trim_silver,
        name="handle_mount",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.104),
        origin=Origin(xyz=(0.408, 0.064, 0.0)),
        material=trim_silver,
        name="latch_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.47, 0.09, 0.47)),
        mass=3.5,
        origin=Origin(xyz=(0.225, 0.04, 0.0)),
    )

    model.articulation(
        "drum_axle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.225, 0.300, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_axle = object_model.get_articulation("drum_axle")
    door_hinge = object_model.get_articulation("door_hinge")

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

    ctx.check(
        "door hinge axis is vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "drum axle runs front to back",
        tuple(drum_axle.axis) == (0.0, 1.0, 0.0),
        f"drum axle axis was {drum_axle.axis}",
    )

    door_limits = door_hinge.motion_limits
    ctx.check(
        "door opens like a front porthole door",
        (
            door_limits is not None
            and door_limits.lower is not None
            and door_limits.upper is not None
            and door_limits.lower <= 0.0
            and door_limits.upper >= math.radians(105.0)
        ),
        f"door limits were {door_limits}",
    )

    drum_limits = drum_axle.motion_limits
    ctx.check(
        "drum has usable rotation range",
        (
            drum_limits is not None
            and drum_limits.lower is not None
            and drum_limits.upper is not None
            and drum_limits.lower < 0.0
            and drum_limits.upper > 1.0
        ),
        f"drum limits were {drum_limits}",
    )

    ctx.expect_contact(
        drum,
        cabinet,
        elem_a="spindle",
        elem_b="rear_bearing_boss",
        name="drum spindle seats in rear bearing",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="door_inner_ring",
            negative_elem="front_seal_ring",
            name="door closes against front seal ring",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.34,
            name="door covers the front opening",
        )
        ctx.expect_overlap(
            door,
            drum,
            axes="xz",
            min_overlap=0.28,
            name="door and drum align on the front opening",
        )
        ctx.expect_within(
            drum,
            cabinet,
            axes="xz",
            margin=0.06,
            name="drum stays inside cabinet footprint",
        )

        handle_aabb = ctx.part_element_world_aabb(door, elem="latch_handle")
        glass_aabb = ctx.part_element_world_aabb(door, elem="door_glass")
        if handle_aabb is not None and glass_aabb is not None:
            glass_center_x = (glass_aabb[0][0] + glass_aabb[1][0]) * 0.5
            ctx.check(
                "latch handle is on the right side of the door",
                handle_aabb[0][0] > glass_center_x,
                f"handle min x {handle_aabb[0][0]:.4f} was not right of glass center {glass_center_x:.4f}",
            )
        else:
            ctx.fail("latch handle geometry exists", "missing latch handle or door glass AABB")

        rest_door_aabb = ctx.part_world_aabb(door)
        if rest_door_aabb is None:
            ctx.fail("door rest aabb exists", "door AABB was unavailable in closed pose")
        else:
            rest_center_x = (rest_door_aabb[0][0] + rest_door_aabb[1][0]) * 0.5
            rest_center_y = (rest_door_aabb[0][1] + rest_door_aabb[1][1]) * 0.5

            with ctx.pose({door_hinge: math.radians(110.0)}):
                open_door_aabb = ctx.part_world_aabb(door)
                if open_door_aabb is None:
                    ctx.fail("door open aabb exists", "door AABB was unavailable in open pose")
                else:
                    open_center_x = (open_door_aabb[0][0] + open_door_aabb[1][0]) * 0.5
                    open_center_y = (open_door_aabb[0][1] + open_door_aabb[1][1]) * 0.5
                    ctx.check(
                        "door swings outward from its left hinge",
                        open_center_x < (rest_center_x - 0.18) and open_center_y > (rest_center_y + 0.14),
                        (
                            f"rest center=({rest_center_x:.4f}, {rest_center_y:.4f}) "
                            f"open center=({open_center_x:.4f}, {open_center_y:.4f})"
                        ),
                    )

    with ctx.pose({drum_axle: 1.8}):
        ctx.expect_contact(
            drum,
            cabinet,
            elem_a="spindle",
            elem_b="rear_bearing_boss",
            name="drum spindle stays seated while rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
