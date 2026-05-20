from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_bar_visual(
    part,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    z: float,
    width: float,
    thickness: float,
    material,
    name: str,
) -> None:
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    length = hypot(dx, dy)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=((start_xy[0] + end_xy[0]) * 0.5, (start_xy[1] + end_xy[1]) * 0.5, z),
            rpy=(0.0, 0.0, atan2(dy, dx)),
        ),
        material=material,
        name=name,
    )


def _build_wiper_assembly(part, *, side: str, metal, rubber) -> None:
    sign = 1.0 if side == "left" else -1.0
    blade_y = 0.017 if side == "left" else 0.006
    prefix = f"{side}_"

    part.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=metal,
        name=f"{prefix}collar",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=metal,
        name=f"{prefix}spindle",
    )
    part.visual(
        Box((0.014, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=metal,
        name=f"{prefix}hub",
    )

    _add_bar_visual(
        part,
        (0.0, 0.0),
        (0.046 * sign, 0.008),
        z=0.012,
        width=0.006,
        thickness=0.004,
        material=metal,
        name=f"{prefix}arm",
    )
    part.visual(
        Box((0.050, 0.004, 0.004)),
        origin=Origin(xyz=(0.040 * sign, blade_y, 0.012)),
        material=metal,
        name=f"{prefix}blade_spine",
    )
    part.visual(
        Box((0.012, abs(blade_y - 0.008) + 0.004, 0.004)),
        origin=Origin(xyz=(0.040 * sign, 0.5 * (blade_y + 0.008), 0.011)),
        material=metal,
        name=f"{prefix}blade_bridge",
    )
    part.visual(
        Box((0.050, 0.009, 0.006)),
        origin=Origin(xyz=(0.040 * sign, blade_y, 0.0088)),
        material=rubber,
        name=f"{prefix}blade",
    )

    _add_bar_visual(
        part,
        (0.0, 0.0),
        (0.012 * sign, -0.020),
        z=0.009,
        width=0.006,
        thickness=0.004,
        material=metal,
        name=f"{prefix}bellcrank",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.012 * sign, -0.020, 0.010)),
        material=metal,
        name=f"{prefix}drive_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_windshield_wiper_assembly")

    cowl_gray = model.material("cowl_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    acrylic = model.material("acrylic", rgba=(0.70, 0.86, 0.95, 0.38))

    base = model.part("base_module")
    base.inertial = Inertial.from_geometry(
        Box((0.32, 0.15, 0.05)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    base.visual(
        Box((0.32, 0.15, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cowl_gray,
        name="deck",
    )
    base.visual(
        Box((0.32, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.067, 0.010)),
        material=dark_metal,
        name="front_fascia",
    )
    base.visual(
        Box((0.30, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.066, 0.012)),
        material=dark_metal,
        name="rear_cowl",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.028, 0.022)),
        material=dark_metal,
        name="motor_pod",
    )
    base.visual(
        Box((0.24, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.005, 0.022)),
        material=cowl_gray,
        name="front_panel_support",
    )
    base.visual(
        Box((0.24, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.055, 0.022)),
        material=cowl_gray,
        name="rear_panel_support",
    )
    base.visual(
        Box((0.24, 0.060, 0.003)),
        origin=Origin(xyz=(0.0, 0.025, 0.0335)),
        material=acrylic,
        name="wipe_panel",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(-0.055, -0.002, 0.022)),
        material=dark_metal,
        name="left_pedestal",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.055, -0.002, 0.022)),
        material=dark_metal,
        name="right_pedestal",
    )

    motor_crank = model.part("motor_crank")
    motor_crank.inertial = Inertial.from_geometry(
        Box((0.045, 0.040, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(-0.008, 0.0, 0.007)),
    )
    motor_crank.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_metal,
        name="hub_collar",
    )
    motor_crank.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_metal,
        name="hub_disk",
    )
    _add_bar_visual(
        motor_crank,
        (0.0, 0.0),
        (-0.018, 0.0),
        z=0.008,
        width=0.010,
        thickness=0.004,
        material=satin_metal,
        name="crank_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(-0.018, 0.0, 0.010)),
        material=satin_metal,
        name="crank_pin",
    )

    drive_link = model.part("drive_link")
    drive_link.inertial = Inertial.from_geometry(
        Box((0.030, 0.012, 0.006)),
        mass=0.03,
        origin=Origin(xyz=(-0.0125, 0.003, 0.003)),
    )
    drive_link.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="crank_eye",
    )
    _add_bar_visual(
        drive_link,
        (0.0, 0.0),
        (-0.025, 0.006),
        z=0.003,
        width=0.008,
        thickness=0.004,
        material=dark_metal,
        name="drive_bar",
    )
    drive_link.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.025, 0.006, 0.002)),
        material=dark_metal,
        name="left_capture",
    )

    left_wiper = model.part("left_wiper")
    left_wiper.inertial = Inertial.from_geometry(
        Box((0.10, 0.040, 0.022)),
        mass=0.10,
        origin=Origin(xyz=(0.022, 0.000, 0.011)),
    )
    _build_wiper_assembly(left_wiper, side="left", metal=satin_metal, rubber=rubber)

    transfer_link = model.part("transfer_link")
    transfer_link.inertial = Inertial.from_geometry(
        Box((0.088, 0.012, 0.006)),
        mass=0.04,
        origin=Origin(xyz=(0.042, 0.0005, 0.003)),
    )
    transfer_link.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="left_eye",
    )
    _add_bar_visual(
        transfer_link,
        (0.0, 0.0),
        (0.084, 0.001),
        z=0.003,
        width=0.008,
        thickness=0.004,
        material=dark_metal,
        name="transfer_bar",
    )
    transfer_link.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.084, 0.001, 0.002)),
        material=dark_metal,
        name="right_capture",
    )

    right_wiper = model.part("right_wiper")
    right_wiper.inertial = Inertial.from_geometry(
        Box((0.10, 0.040, 0.022)),
        mass=0.10,
        origin=Origin(xyz=(-0.022, 0.000, 0.011)),
    )
    _build_wiper_assembly(right_wiper, side="right", metal=satin_metal, rubber=rubber)

    model.articulation(
        "motor_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=motor_crank,
        origin=Origin(xyz=(0.0, -0.028, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(-0.018, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "left_wiper_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_wiper,
        origin=Origin(xyz=(-0.055, -0.002, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-0.18, upper=1.08),
    )
    model.articulation(
        "left_to_transfer_link",
        ArticulationType.REVOLUTE,
        parent=left_wiper,
        child=transfer_link,
        origin=Origin(xyz=(0.012, -0.020, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "right_wiper_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_wiper,
        origin=Origin(xyz=(0.055, -0.002, 0.032)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-0.18, upper=1.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_module")
    motor_crank = object_model.get_part("motor_crank")
    drive_link = object_model.get_part("drive_link")
    left_wiper = object_model.get_part("left_wiper")
    transfer_link = object_model.get_part("transfer_link")
    right_wiper = object_model.get_part("right_wiper")

    motor_rotation = object_model.get_articulation("motor_rotation")
    crank_to_drive_link = object_model.get_articulation("crank_to_drive_link")
    left_wiper_pivot = object_model.get_articulation("left_wiper_pivot")
    left_to_transfer_link = object_model.get_articulation("left_to_transfer_link")
    right_wiper_pivot = object_model.get_articulation("right_wiper_pivot")

    deck = base.get_visual("deck")
    wipe_panel = base.get_visual("wipe_panel")
    motor_pod = base.get_visual("motor_pod")
    left_pedestal = base.get_visual("left_pedestal")
    right_pedestal = base.get_visual("right_pedestal")

    hub_collar = motor_crank.get_visual("hub_collar")
    crank_pin = motor_crank.get_visual("crank_pin")
    crank_eye = drive_link.get_visual("crank_eye")
    left_capture = drive_link.get_visual("left_capture")

    left_collar = left_wiper.get_visual("left_collar")
    left_drive_pin = left_wiper.get_visual("left_drive_pin")
    left_blade = left_wiper.get_visual("left_blade")
    left_eye = transfer_link.get_visual("left_eye")
    transfer_bar = transfer_link.get_visual("transfer_bar")
    right_capture = transfer_link.get_visual("right_capture")

    right_collar = right_wiper.get_visual("right_collar")
    right_drive_pin = right_wiper.get_visual("right_drive_pin")
    right_blade = right_wiper.get_visual("right_blade")

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

    ctx.expect_contact(motor_crank, base, elem_a=hub_collar, elem_b=motor_pod, name="motor_supported_on_pod")
    ctx.expect_contact(left_wiper, base, elem_a=left_collar, elem_b=left_pedestal, name="left_spindle_supported")
    ctx.expect_contact(right_wiper, base, elem_a=right_collar, elem_b=right_pedestal, name="right_spindle_supported")
    ctx.expect_contact(drive_link, motor_crank, elem_a=crank_eye, elem_b=crank_pin, name="drive_link_pinned_to_crank")
    ctx.expect_contact(transfer_link, left_wiper, elem_a=left_eye, elem_b=left_drive_pin, name="transfer_link_pinned_to_left_wiper")
    ctx.expect_contact(drive_link, left_wiper, elem_a=left_capture, elem_b=left_drive_pin, name="drive_link_reaches_left_follower")
    ctx.expect_contact(transfer_link, right_wiper, elem_a=right_capture, elem_b=right_drive_pin, name="transfer_link_reaches_right_follower")

    ctx.expect_gap(
        left_wiper,
        base,
        axis="z",
        positive_elem=left_blade,
        negative_elem=wipe_panel,
        min_gap=0.002,
        max_gap=0.010,
        name="left_blade_stows_low_over_panel",
    )
    ctx.expect_gap(
        right_wiper,
        base,
        axis="z",
        positive_elem=right_blade,
        negative_elem=wipe_panel,
        min_gap=0.002,
        max_gap=0.010,
        name="right_blade_stows_low_over_panel",
    )
    ctx.expect_within(
        left_wiper,
        base,
        axes="xy",
        inner_elem=left_blade,
        outer_elem=wipe_panel,
        margin=0.006,
        name="left_blade_stays_over_panel_at_park",
    )
    ctx.expect_within(
        right_wiper,
        base,
        axes="xy",
        inner_elem=right_blade,
        outer_elem=wipe_panel,
        margin=0.006,
        name="right_blade_stays_over_panel_at_park",
    )

    def _center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_left = ctx.part_element_world_aabb(left_wiper, elem=left_blade)
    rest_right = ctx.part_element_world_aabb(right_wiper, elem=right_blade)

    with ctx.pose(
        {
            motor_rotation: 0.55,
            crank_to_drive_link: 0.12,
            left_wiper_pivot: 0.88,
            left_to_transfer_link: -0.08,
            right_wiper_pivot: 0.78,
        }
    ):
        ctx.expect_contact(motor_crank, base, elem_a=hub_collar, elem_b=motor_pod, name="motor_supported_in_sweep_pose")
        ctx.expect_contact(left_wiper, base, elem_a=left_collar, elem_b=left_pedestal, name="left_spindle_supported_in_sweep_pose")
        ctx.expect_contact(right_wiper, base, elem_a=right_collar, elem_b=right_pedestal, name="right_spindle_supported_in_sweep_pose")
        ctx.expect_contact(drive_link, motor_crank, elem_a=crank_eye, elem_b=crank_pin, name="drive_link_pinned_in_sweep_pose")
        ctx.expect_contact(transfer_link, left_wiper, elem_a=left_eye, elem_b=left_drive_pin, name="transfer_link_pinned_in_sweep_pose")
        ctx.expect_gap(
            left_wiper,
            base,
            axis="z",
            positive_elem=left_blade,
            negative_elem=wipe_panel,
            min_gap=0.002,
            max_gap=0.012,
            name="left_blade_clears_panel_in_sweep_pose",
        )
        ctx.expect_gap(
            right_wiper,
            base,
            axis="z",
            positive_elem=right_blade,
            negative_elem=wipe_panel,
            min_gap=0.002,
            max_gap=0.012,
            name="right_blade_clears_panel_in_sweep_pose",
        )
        ctx.expect_within(
            left_wiper,
            base,
            axes="xy",
            inner_elem=left_blade,
            outer_elem=wipe_panel,
            margin=0.010,
            name="left_blade_stays_over_panel_in_sweep_pose",
        )
        ctx.expect_within(
            right_wiper,
            base,
            axes="xy",
            inner_elem=right_blade,
            outer_elem=wipe_panel,
            margin=0.010,
            name="right_blade_stays_over_panel_in_sweep_pose",
        )
        ctx.expect_gap(
            drive_link,
            base,
            axis="z",
            positive_elem=drive_link.get_visual("drive_bar"),
            negative_elem=deck,
            min_gap=0.020,
            name="drive_link_keeps_clear_of_deck",
        )
        ctx.expect_gap(
            transfer_link,
            base,
            axis="z",
            positive_elem=transfer_bar,
            negative_elem=deck,
            min_gap=0.020,
            name="transfer_link_keeps_clear_of_deck",
        )

        swept_left = ctx.part_element_world_aabb(left_wiper, elem=left_blade)
        swept_right = ctx.part_element_world_aabb(right_wiper, elem=right_blade)
        left_rest_y = _center_y(rest_left)
        right_rest_y = _center_y(rest_right)
        left_swept_y = _center_y(swept_left)
        right_swept_y = _center_y(swept_right)
        ctx.check(
            "left_blade_sweeps_rearward",
            left_rest_y is not None and left_swept_y is not None and left_swept_y > left_rest_y + 0.020,
            details=f"rest_y={left_rest_y}, swept_y={left_swept_y}",
        )
        ctx.check(
            "right_blade_sweeps_rearward",
            right_rest_y is not None and right_swept_y is not None and right_swept_y > right_rest_y + 0.020,
            details=f"rest_y={right_rest_y}, swept_y={right_swept_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
