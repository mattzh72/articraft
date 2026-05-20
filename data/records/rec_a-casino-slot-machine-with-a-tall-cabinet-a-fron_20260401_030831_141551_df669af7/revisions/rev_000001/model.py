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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casino_slot_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.49, 0.05, 0.08, 1.0))
    cabinet_black = model.material("cabinet_black", rgba=(0.09, 0.10, 0.11, 1.0))
    brass_trim = model.material("brass_trim", rgba=(0.76, 0.62, 0.22, 1.0))
    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    reel_cream = model.material("reel_cream", rgba=(0.93, 0.92, 0.86, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((0.70, 0.44, 1.52)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )
    cabinet.visual(
        Box((0.02, 0.44, 1.52)),
        origin=Origin(xyz=(-0.34, 0.0, 0.76)),
        material=cabinet_red,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.02, 0.44, 1.52)),
        origin=Origin(xyz=(0.34, 0.0, 0.76)),
        material=cabinet_red,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((0.66, 0.02, 1.52)),
        origin=Origin(xyz=(0.0, -0.21, 0.76)),
        material=cabinet_red,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.66, 0.40, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=cabinet_black,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.66, 0.42, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
        material=cabinet_black,
        name="roof_panel",
    )
    cabinet.visual(
        Box((0.66, 0.02, 0.60)),
        origin=Origin(xyz=(0.0, 0.21, 0.30)),
        material=cabinet_red,
        name="front_lower_panel",
    )
    cabinet.visual(
        Box((0.66, 0.02, 0.08)),
        origin=Origin(xyz=(0.0, 0.21, 0.64)),
        material=brass_trim,
        name="window_sill",
    )
    cabinet.visual(
        Box((0.05, 0.02, 0.40)),
        origin=Origin(xyz=(-0.305, 0.21, 0.88)),
        material=brass_trim,
        name="left_jamb",
    )
    cabinet.visual(
        Box((0.05, 0.02, 0.40)),
        origin=Origin(xyz=(0.305, 0.21, 0.88)),
        material=brass_trim,
        name="right_jamb",
    )
    cabinet.visual(
        Box((0.014, 0.016, 0.40)),
        origin=Origin(xyz=(-0.093, 0.208, 0.88)),
        material=brass_trim,
        name="separator_left",
    )
    cabinet.visual(
        Box((0.014, 0.016, 0.40)),
        origin=Origin(xyz=(0.093, 0.208, 0.88)),
        material=brass_trim,
        name="separator_right",
    )
    cabinet.visual(
        Box((0.66, 0.02, 0.09)),
        origin=Origin(xyz=(0.0, 0.21, 1.125)),
        material=brass_trim,
        name="window_brow",
    )
    cabinet.visual(
        Box((0.66, 0.12, 0.34)),
        origin=Origin(xyz=(0.0, 0.16, 1.35)),
        material=cabinet_red,
        name="upper_front_marquee",
    )
    cabinet.visual(
        Box((0.42, 0.012, 0.18)),
        origin=Origin(xyz=(0.0, 0.214, 1.34)),
        material=cabinet_black,
        name="marquee_panel",
    )
    cabinet.visual(
        Box((0.028, 0.12, 0.16)),
        origin=Origin(xyz=(0.364, 0.02, 0.94)),
        material=dark_steel,
        name="handle_mount_pad",
    )
    for bearing_name, x_pos in (
        ("left_reel_bearing", -0.19),
        ("center_reel_bearing", 0.0),
        ("right_reel_bearing", 0.19),
    ):
        cabinet.visual(
            Box((0.05, 0.08, 0.05)),
            origin=Origin(xyz=(x_pos, -0.16, 0.88)),
            material=dark_steel,
            name=bearing_name,
        )

    coin_tray = model.part("coin_tray")
    coin_tray.inertial = Inertial.from_geometry(
        Box((0.32, 0.13, 0.06)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.055, 0.025)),
    )
    coin_tray.visual(
        Box((0.32, 0.01, 0.05)),
        origin=Origin(xyz=(0.0, -0.005, 0.025)),
        material=chrome,
        name="mount_flange",
    )
    coin_tray.visual(
        Box((0.32, 0.12, 0.01)),
        origin=Origin(xyz=(0.0, 0.06, 0.005)),
        material=chrome,
        name="tray_floor",
    )
    coin_tray.visual(
        Box((0.01, 0.12, 0.04)),
        origin=Origin(xyz=(-0.155, 0.06, 0.02)),
        material=chrome,
        name="left_tray_wall",
    )
    coin_tray.visual(
        Box((0.01, 0.12, 0.04)),
        origin=Origin(xyz=(0.155, 0.06, 0.02)),
        material=chrome,
        name="right_tray_wall",
    )
    coin_tray.visual(
        Box((0.32, 0.01, 0.03)),
        origin=Origin(xyz=(0.0, 0.115, 0.015)),
        material=chrome,
        name="front_lip",
    )

    handle = model.part("pull_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.52)),
        mass=3.0,
        origin=Origin(xyz=(0.05, 0.0, 0.24)),
    )
    handle.visual(
        Box((0.028, 0.09, 0.06)),
        origin=Origin(xyz=(0.014, 0.0, 0.03)),
        material=dark_steel,
        name="hub_block",
    )
    handle_arm = tube_from_spline_points(
        [
            (0.018, 0.0, 0.02),
            (0.030, 0.0, 0.11),
            (0.050, 0.0, 0.24),
            (0.074, 0.0, 0.35),
            (0.095, 0.0, 0.40),
        ],
        radius=0.013,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        _mesh("slot_handle_arm", handle_arm),
        material=chrome,
        name="handle_arm",
    )
    handle.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.095, 0.0, 0.445)),
        material=knob_black,
        name="handle_knob",
    )

    def add_reel(part_name: str) -> None:
        reel = model.part(part_name)
        reel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.079, length=0.215),
            mass=2.8,
            origin=Origin(xyz=(0.0, -0.0725, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        reel.visual(
            Cylinder(radius=0.079, length=0.07),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=reel_cream,
            name="drum",
        )
        reel.visual(
            Cylinder(radius=0.010, length=0.145),
            origin=Origin(xyz=(0.0, -0.1075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="axle",
        )
        reel.visual(
            Cylinder(radius=0.074, length=0.01),
            origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass_trim,
            name="front_ring",
        )

    add_reel("left_reel")
    add_reel("center_reel")
    add_reel("right_reel")

    model.articulation(
        "cabinet_to_coin_tray",
        ArticulationType.FIXED,
        parent=cabinet,
        child=coin_tray,
        origin=Origin(xyz=(0.0, 0.23, 0.34)),
    )
    model.articulation(
        "cabinet_to_pull_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(0.378, 0.02, 0.94)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.05),
    )
    for joint_name, child_name, x_pos in (
        ("left_reel_spin", "left_reel", -0.19),
        ("center_reel_spin", "center_reel", 0.0),
        ("right_reel_spin", "right_reel", 0.19),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=child_name,
            origin=Origin(xyz=(x_pos, 0.06, 0.88)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    coin_tray = object_model.get_part("coin_tray")
    handle = object_model.get_part("pull_handle")
    left_reel = object_model.get_part("left_reel")
    center_reel = object_model.get_part("center_reel")
    right_reel = object_model.get_part("right_reel")
    handle_joint = object_model.get_articulation("cabinet_to_pull_handle")
    left_reel_spin = object_model.get_articulation("left_reel_spin")
    center_reel_spin = object_model.get_articulation("center_reel_spin")
    right_reel_spin = object_model.get_articulation("right_reel_spin")

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

    ctx.expect_contact(
        coin_tray,
        cabinet,
        elem_a="mount_flange",
        elem_b="front_lower_panel",
        name="coin tray is mounted to the cabinet front",
    )
    ctx.expect_contact(
        handle,
        cabinet,
        elem_a="hub_block",
        elem_b="handle_mount_pad",
        name="pull handle is mounted on the side pad",
    )

    ctx.expect_contact(
        left_reel,
        cabinet,
        elem_a="axle",
        elem_b="left_reel_bearing",
        name="left reel axle reaches its rear bearing",
    )
    ctx.expect_contact(
        center_reel,
        cabinet,
        elem_a="axle",
        elem_b="center_reel_bearing",
        name="center reel axle reaches its rear bearing",
    )
    ctx.expect_contact(
        right_reel,
        cabinet,
        elem_a="axle",
        elem_b="right_reel_bearing",
        name="right reel axle reaches its rear bearing",
    )

    for reel_name, reel_part, left_barrier, right_barrier in (
        ("left reel", left_reel, "left_jamb", "separator_left"),
        ("center reel", center_reel, "separator_left", "separator_right"),
        ("right reel", right_reel, "separator_right", "right_jamb"),
    ):
        ctx.expect_gap(
            reel_part,
            cabinet,
            axis="z",
            positive_elem="drum",
            negative_elem="window_sill",
            min_gap=0.10,
            name=f"{reel_name} stays above the window sill",
        )
        ctx.expect_gap(
            cabinet,
            reel_part,
            axis="z",
            positive_elem="window_brow",
            negative_elem="drum",
            min_gap=0.10,
            name=f"{reel_name} stays below the window brow",
        )
        ctx.expect_gap(
            reel_part,
            cabinet,
            axis="x",
            positive_elem="drum",
            negative_elem=left_barrier,
            min_gap=0.006,
            name=f"{reel_name} clears its left divider",
        )
        ctx.expect_gap(
            cabinet,
            reel_part,
            axis="x",
            positive_elem=right_barrier,
            negative_elem="drum",
            min_gap=0.006,
            name=f"{reel_name} clears its right divider",
        )

    ctx.check(
        "handle joint axis runs front-to-back",
        handle_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "reel spin axes run front-to-back",
        left_reel_spin.axis == (0.0, 1.0, 0.0)
        and center_reel_spin.axis == (0.0, 1.0, 0.0)
        and right_reel_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"left={left_reel_spin.axis}, center={center_reel_spin.axis}, "
            f"right={right_reel_spin.axis}"
        ),
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb is not None else None

    rest_knob = ctx.part_element_world_aabb(handle, elem="handle_knob")
    with ctx.pose({handle_joint: 1.05}):
        pulled_knob = ctx.part_element_world_aabb(handle, elem="handle_knob")
    rest_center = aabb_center(rest_knob)
    pulled_center = aabb_center(pulled_knob)
    ctx.check(
        "pull handle swings outward and downward",
        rest_center is not None
        and pulled_center is not None
        and pulled_center[0] > rest_center[0] + 0.10
        and pulled_center[2] < rest_center[2] - 0.10,
        details=f"rest={rest_center}, pulled={pulled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
