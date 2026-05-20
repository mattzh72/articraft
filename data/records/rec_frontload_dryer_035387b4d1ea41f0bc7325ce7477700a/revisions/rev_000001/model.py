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


def _ring_shell_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    depth: float,
    segments: int = 64,
) -> object:
    bevel = min(0.006, depth * 0.22, max(0.0015, (outer_radius - inner_radius) * 0.45))
    outer_profile = [
        (outer_radius - bevel, 0.0),
        (outer_radius, bevel),
        (outer_radius, depth - bevel),
        (outer_radius - bevel, depth),
    ]
    inner_profile = [
        (inner_radius, max(0.0015, bevel * 0.8)),
        (inner_radius, depth - max(0.0015, bevel * 0.8)),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _drum_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 72,
) -> object:
    outer_profile = [
        (0.0, 0.0),
        (outer_radius * 0.24, 0.012),
        (outer_radius * 0.95, 0.032),
        (outer_radius, 0.050),
        (outer_radius, length - 0.016),
        (outer_radius * 1.015, length),
    ]
    inner_profile = [
        (0.0, 0.010),
        (inner_radius * 0.40, 0.018),
        (inner_radius, 0.036),
        (inner_radius, length - 0.010),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_compact_dryer")

    appliance_white = model.material("appliance_white", rgba=(0.94, 0.95, 0.96, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.36, 0.43, 0.48, 0.38))

    width = 0.595
    depth = 0.560
    height = 0.845
    wall = 0.018
    front_skin = 0.032
    plinth_height = 0.105
    drawer_panel_width = 0.430
    drawer_cover_width = 0.410
    drawer_opening_width = 0.380
    door_center_z = 0.470
    door_outer_radius = 0.206
    drum_length = 0.357
    drum_outer_radius = 0.206
    drum_inner_radius = 0.190
    drum_joint_y = -0.187

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=appliance_white,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=appliance_white,
        name="right_side",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=appliance_white,
        name="top_cap",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), wall, height - (2.0 * wall))),
        origin=Origin(
            xyz=(0.0, -depth / 2.0 + wall / 2.0, height / 2.0),
        ),
        material=appliance_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), 0.370, 0.010)),
        origin=Origin(xyz=(0.0, -0.095, 0.005)),
        material=warm_gray,
        name="base_rear_floor",
    )
    side_block_width = (width - drawer_opening_width) / 2.0
    cabinet.visual(
        Box((side_block_width, 0.120, plinth_height)),
        origin=Origin(
            xyz=(
                -drawer_opening_width / 2.0 - side_block_width / 2.0,
                depth / 2.0 - 0.060,
                plinth_height / 2.0,
            )
        ),
        material=appliance_white,
        name="plinth_left_block",
    )
    cabinet.visual(
        Box((side_block_width, 0.120, plinth_height)),
        origin=Origin(
            xyz=(
                drawer_opening_width / 2.0 + side_block_width / 2.0,
                depth / 2.0 - 0.060,
                plinth_height / 2.0,
            )
        ),
        material=appliance_white,
        name="plinth_right_block",
    )
    cabinet.visual(
        Box((drawer_cover_width, 0.120, 0.022)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.060, plinth_height - 0.011)),
        material=appliance_white,
        name="plinth_top_bridge",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), front_skin, 0.140)),
        origin=Origin(xyz=(0.0, depth / 2.0 - front_skin / 2.0, 0.180)),
        material=appliance_white,
        name="lower_front_rail",
    )
    cabinet.visual(
        Box((0.115, front_skin, 0.580)),
        origin=Origin(
            xyz=(
                -width / 2.0 + 0.0575,
                depth / 2.0 - front_skin / 2.0,
                0.411,
            )
        ),
        material=appliance_white,
        name="left_front_stile",
    )
    cabinet.visual(
        Box((0.115, front_skin, 0.580)),
        origin=Origin(
            xyz=(
                width / 2.0 - 0.0575,
                depth / 2.0 - front_skin / 2.0,
                0.411,
            )
        ),
        material=appliance_white,
        name="right_front_stile",
    )
    cabinet.visual(
        Box((width - (2.0 * wall), front_skin, 0.186)),
        origin=Origin(xyz=(0.0, depth / 2.0 - front_skin / 2.0, 0.752)),
        material=appliance_white,
        name="upper_front_header",
    )
    cabinet.visual(
        Box((0.460, 0.014, 0.094)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.007, 0.752)),
        material=dark_trim,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.096, 0.004, 0.030)),
        origin=Origin(xyz=(-0.150, depth / 2.0 + 0.016, 0.754)),
        material=smoked_glass,
        name="display_window",
    )
    cabinet.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(
            xyz=(0.175, depth / 2.0 + 0.010, 0.752),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="program_knob",
    )
    cabinet.visual(
        _ring_shell_mesh(
            "dryer_front_gasket_ring",
            inner_radius=0.197,
            outer_radius=0.228,
            depth=0.028,
        ),
        origin=Origin(
            xyz=(0.0, depth / 2.0 - 0.028, door_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="front_gasket_ring",
    )
    cabinet.visual(
        Box((0.140, 0.090, 0.120)),
        origin=Origin(xyz=(0.0, -0.232, door_center_z)),
        material=warm_gray,
        name="rear_support_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(
            xyz=(0.0, drum_joint_y - 0.012, door_center_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="rear_bearing_boss",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _drum_shell_mesh(
            "dryer_drum_shell",
            outer_radius=drum_outer_radius,
            inner_radius=drum_inner_radius,
            length=drum_length,
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_hub",
    )
    drum.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.197, tube=0.006, radial_segments=18, tubular_segments=64).rotate_x(
                -math.pi / 2.0
            ),
            "dryer_drum_front_band",
        ),
        origin=Origin(xyz=(0.0, drum_length - 0.010, 0.0)),
        material=drum_steel,
        name="front_band",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_outer_radius, length=drum_length),
        mass=7.5,
        origin=Origin(xyz=(0.0, drum_length / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        _ring_shell_mesh(
            "dryer_door_frame",
            inner_radius=0.145,
            outer_radius=door_outer_radius,
            depth=0.056,
        ),
        origin=Origin(
            xyz=(door_outer_radius, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=appliance_white,
        name="door_frame",
    )
    door.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.177, tube=0.010, radial_segments=18, tubular_segments=64).rotate_x(
                -math.pi / 2.0
            ),
            "dryer_door_chrome_bezel",
        ),
        origin=Origin(xyz=(door_outer_radius, 0.047, 0.0)),
        material=chrome,
        name="chrome_bezel",
    )
    door.visual(
        Cylinder(radius=0.150, length=0.004),
        origin=Origin(
            xyz=(door_outer_radius, 0.019, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.028, 0.046, 0.380)),
        origin=Origin(xyz=(0.014, 0.023, 0.0)),
        material=warm_gray,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(0.0, 0.023, 0.124)),
        material=chrome,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(0.0, 0.023, -0.124)),
        material=chrome,
        name="lower_knuckle",
    )
    door.visual(
        Box((0.026, 0.028, 0.058)),
        origin=Origin(xyz=((2.0 * door_outer_radius) - 0.018, 0.024, 0.0)),
        material=dark_trim,
        name="latch_block",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.430, 0.070, 0.430)),
        mass=1.8,
        origin=Origin(xyz=(0.215, 0.028, 0.0)),
    )

    lint_drawer = model.part("lint_drawer")
    lint_drawer.visual(
        Box((drawer_panel_width, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, 0.007, 0.044)),
        material=appliance_white,
        name="front_panel",
    )
    lint_drawer.visual(
        Box((0.140, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.055)),
        material=chrome,
        name="pull_handle",
    )
    lint_drawer.visual(
        Box((0.340, 0.170, 0.006)),
        origin=Origin(xyz=(0.0, -0.085, 0.014)),
        material=warm_gray,
        name="tray_bottom",
    )
    lint_drawer.visual(
        Box((0.008, 0.170, 0.050)),
        origin=Origin(xyz=(-0.166, -0.085, 0.035)),
        material=warm_gray,
        name="left_wall",
    )
    lint_drawer.visual(
        Box((0.008, 0.170, 0.050)),
        origin=Origin(xyz=(0.166, -0.085, 0.035)),
        material=warm_gray,
        name="right_wall",
    )
    lint_drawer.visual(
        Box((0.340, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.166, 0.035)),
        material=warm_gray,
        name="tray_back_wall",
    )
    lint_drawer.inertial = Inertial.from_geometry(
        Box((0.430, 0.190, 0.090)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.070, 0.045)),
    )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_joint_y, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=16.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_outer_radius, depth / 2.0, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "cabinet_to_lint_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lint_drawer,
        origin=Origin(xyz=(0.0, depth / 2.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    lint_drawer = object_model.get_part("lint_drawer")

    drum_spin = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_lint_drawer")

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
        "dryer uses the requested articulation stack",
        drum_spin.articulation_type == ArticulationType.CONTINUOUS
        and door_hinge.articulation_type == ArticulationType.REVOLUTE
        and drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"drum={drum_spin.articulation_type}, "
            f"door={door_hinge.articulation_type}, "
            f"drawer={drawer_slide.articulation_type}"
        ),
    )
    ctx.check(
        "joint axes match the mechanism layout",
        tuple(drum_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=(
            f"drum_axis={drum_spin.axis}, "
            f"door_axis={door_hinge.axis}, "
            f"drawer_axis={drawer_slide.axis}"
        ),
    )

    ctx.expect_gap(
        drum,
        cabinet,
        axis="y",
        positive_elem="rear_hub",
        negative_elem="rear_bearing_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="drum hub seats on the rear bearing boss",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        positive_elem="door_frame",
        negative_elem="front_gasket_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="door closes flush against the front gasket ring",
    )
    ctx.expect_gap(
        lint_drawer,
        cabinet,
        axis="y",
        positive_elem="front_panel",
        negative_elem="plinth_top_bridge",
        max_gap=0.001,
        max_penetration=0.0,
        name="lint drawer front panel seats flush at the plinth",
    )

    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else None
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None

    if door_upper is not None:
        with ctx.pose({door_hinge: door_upper * 0.82}):
            ctx.expect_gap(
                door,
                cabinet,
                axis="y",
                positive_elem="latch_block",
                negative_elem="front_gasket_ring",
                min_gap=0.120,
                name="door latch edge swings outward when opened",
            )

    drawer_rest = ctx.part_world_position(lint_drawer)
    if drawer_upper is not None:
        with ctx.pose({drawer_slide: drawer_upper}):
            ctx.expect_gap(
                lint_drawer,
                cabinet,
                axis="y",
                positive_elem="front_panel",
                negative_elem="plinth_top_bridge",
                min_gap=0.095,
                name="lint drawer pulls forward for filter access",
            )
            drawer_open = ctx.part_world_position(lint_drawer)
        ctx.check(
            "lint drawer origin translates forward",
            drawer_rest is not None
            and drawer_open is not None
            and drawer_open[1] > drawer_rest[1] + 0.09,
            details=f"rest={drawer_rest}, open={drawer_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
