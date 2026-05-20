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


def _build_handle_arm_mesh():
    arm = tube_from_spline_points(
        [
            (0.028, -0.008, 0.000),
            (0.043, -0.020, 0.075),
            (0.053, -0.042, 0.180),
            (0.058, -0.064, 0.295),
        ],
        radius=0.0115,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(arm, "slot_machine_handle_arm")


def _build_reel_part(
    model: ArticulatedObject,
    *,
    name: str,
    ring_material,
    marker_material,
    drum_material,
    steel_material,
):
    reel = model.part(name)
    spin_rpy = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    reel.visual(
        Cylinder(radius=0.058, length=0.070),
        origin=spin_rpy,
        material=drum_material,
        name="drum_shell",
    )
    reel.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=spin_rpy,
        material=ring_material,
        name="symbol_band",
    )
    reel.visual(
        Cylinder(radius=0.062, length=0.008),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="left_rim",
    )
    reel.visual(
        Cylinder(radius=0.062, length=0.008),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_material,
        name="right_rim",
    )
    reel.visual(
        Cylinder(radius=0.009, length=0.104),
        origin=spin_rpy,
        material=steel_material,
        name="axle",
    )
    reel.visual(
        Box((0.018, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.054, 0.020)),
        material=marker_material,
        name="marker",
    )
    reel.visual(
        Box((0.018, 0.004, 0.020)),
        origin=Origin(xyz=(-0.020, 0.053, -0.010)),
        material=marker_material,
        name="marker_left",
    )
    reel.visual(
        Box((0.018, 0.004, 0.020)),
        origin=Origin(xyz=(0.020, 0.053, -0.010)),
        material=marker_material,
        name="marker_right",
    )
    reel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.104),
        mass=1.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    return reel


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_casino_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.62, 0.08, 0.10, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.17, 0.04, 0.05, 1.0))
    trim_gold = model.material("trim_gold", rgba=(0.79, 0.64, 0.24, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.58, 0.26, 1.0))
    reel_paper = model.material("reel_paper", rgba=(0.94, 0.93, 0.89, 1.0))
    reel_band = model.material("reel_band", rgba=(0.93, 0.84, 0.58, 1.0))
    cherry_red = model.material("cherry_red", rgba=(0.78, 0.10, 0.12, 1.0))
    cobalt = model.material("cobalt", rgba=(0.15, 0.33, 0.74, 1.0))
    emerald = model.material("emerald", rgba=(0.16, 0.58, 0.27, 1.0))
    ivory = model.material("ivory", rgba=(0.92, 0.88, 0.75, 1.0))

    handle_arm_mesh = _build_handle_arm_mesh()

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.46, 0.34, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=cabinet_shadow,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.02, 0.34, 1.36)),
        origin=Origin(xyz=(-0.22, 0.0, 0.78)),
        material=cabinet_red,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((0.02, 0.34, 1.36)),
        origin=Origin(xyz=(0.22, 0.0, 0.78)),
        material=cabinet_red,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((0.42, 0.02, 1.36)),
        origin=Origin(xyz=(0.0, -0.16, 0.78)),
        material=cabinet_red,
        name="back_wall",
    )
    cabinet.visual(
        Box((0.46, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.49)),
        material=cabinet_shadow,
        name="roof_cap",
    )
    cabinet.visual(
        Box((0.42, 0.03, 0.24)),
        origin=Origin(xyz=(0.0, 0.155, 1.27)),
        material=trim_gold,
        name="marquee_panel",
    )
    cabinet.visual(
        Box((0.44, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.155, 1.17)),
        material=trim_gold,
        name="window_header",
    )
    cabinet.visual(
        Box((0.44, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.155, 0.80)),
        material=trim_gold,
        name="window_sill",
    )
    cabinet.visual(
        Box((0.07, 0.03, 0.34)),
        origin=Origin(xyz=(-0.185, 0.155, 0.985)),
        material=trim_gold,
        name="window_left_jamb",
    )
    cabinet.visual(
        Box((0.07, 0.03, 0.34)),
        origin=Origin(xyz=(0.185, 0.155, 0.985)),
        material=trim_gold,
        name="window_right_jamb",
    )
    cabinet.visual(
        Box((0.42, 0.04, 0.18)),
        origin=Origin(xyz=(0.0, 0.150, 0.615)),
        material=cabinet_red,
        name="control_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(-0.090, 0.181, 0.635), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cherry_red,
        name="button_left",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.000, 0.181, 0.635), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cobalt,
        name="button_center",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.090, 0.181, 0.635), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=emerald,
        name="button_right",
    )
    cabinet.visual(
        Box((0.14, 0.015, 0.038)),
        origin=Origin(xyz=(0.0, 0.1775, 0.535)),
        material=dark_trim,
        name="coin_slot_bezel",
    )
    cabinet.visual(
        Box((0.06, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.186, 0.535)),
        material=steel,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.42, 0.03, 0.09)),
        origin=Origin(xyz=(0.0, 0.155, 0.42)),
        material=trim_gold,
        name="door_header",
    )
    cabinet.visual(
        Box((0.07, 0.03, 0.25)),
        origin=Origin(xyz=(-0.185, 0.155, 0.245)),
        material=trim_gold,
        name="door_left_jamb",
    )
    cabinet.visual(
        Box((0.07, 0.03, 0.25)),
        origin=Origin(xyz=(0.185, 0.155, 0.245)),
        material=trim_gold,
        name="door_right_jamb",
    )
    cabinet.visual(
        Box((0.42, 0.03, 0.01)),
        origin=Origin(xyz=(0.0, 0.155, 0.105)),
        material=trim_gold,
        name="door_sill",
    )
    cabinet.visual(
        Box((0.36, 0.08, 0.40)),
        origin=Origin(xyz=(0.0, -0.12, 0.985)),
        material=dark_trim,
        name="reel_backer",
    )
    reel_centers_x = (-0.10, 0.0, 0.10)
    for reel_index, reel_x in enumerate(reel_centers_x):
        for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
            cabinet.visual(
                Box((0.012, 0.181, 0.035)),
                origin=Origin(
                    xyz=(reel_x + side_sign * 0.050, -0.0695, 0.985),
                ),
                material=steel,
                name=f"reel_{reel_index + 1}_{side_name}_bracket",
            )
    cabinet.visual(
        Cylinder(radius=0.036, length=0.031),
        origin=Origin(xyz=(0.2455, -0.01, 0.98), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="handle_mount_boss",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 1.52)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )

    left_reel = _build_reel_part(
        model,
        name="left_reel",
        ring_material=reel_band,
        marker_material=cherry_red,
        drum_material=reel_paper,
        steel_material=steel,
    )
    center_reel = _build_reel_part(
        model,
        name="center_reel",
        ring_material=ivory,
        marker_material=cobalt,
        drum_material=reel_paper,
        steel_material=steel,
    )
    right_reel = _build_reel_part(
        model,
        name="right_reel",
        ring_material=reel_band,
        marker_material=emerald,
        drum_material=reel_paper,
        steel_material=steel,
    )

    handle = model.part("side_handle")
    handle.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_washer",
    )
    handle.visual(
        handle_arm_mesh,
        material=brass,
        name="handle_arm",
    )
    handle.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.058, -0.064, 0.318)),
        material=ivory,
        name="handle_knob",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.11, 0.10, 0.36)),
        mass=1.2,
        origin=Origin(xyz=(0.055, -0.040, 0.170)),
    )

    coin_door = model.part("coin_tray_door")
    coin_door.visual(
        Box((0.296, 0.016, 0.250)),
        origin=Origin(xyz=(0.0, -0.008, 0.125)),
        material=cabinet_shadow,
        name="door_panel",
    )
    coin_door.visual(
        Box((0.235, 0.010, 0.118)),
        origin=Origin(xyz=(0.0, -0.003, 0.170)),
        material=trim_gold,
        name="tray_recess",
    )
    coin_door.visual(
        Box((0.150, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.002, 0.220)),
        material=trim_gold,
        name="tray_lip",
    )
    coin_door.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.112, 0.001, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_lock",
    )
    coin_door.inertial = Inertial.from_geometry(
        Box((0.296, 0.020, 0.250)),
        mass=4.2,
        origin=Origin(xyz=(0.0, -0.006, 0.125)),
    )

    model.articulation(
        "left_reel_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=left_reel,
        origin=Origin(xyz=(-0.10, 0.03, 0.985)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    model.articulation(
        "center_reel_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=center_reel,
        origin=Origin(xyz=(0.0, 0.03, 0.985)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    model.articulation(
        "right_reel_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=right_reel,
        origin=Origin(xyz=(0.10, 0.03, 0.985)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )
    model.articulation(
        "handle_pull",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=handle,
        origin=Origin(xyz=(0.261, -0.01, 0.98)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "coin_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(xyz=(0.0, 0.159, 0.110)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    left_reel = object_model.get_part("left_reel")
    center_reel = object_model.get_part("center_reel")
    right_reel = object_model.get_part("right_reel")
    handle = object_model.get_part("side_handle")
    coin_door = object_model.get_part("coin_tray_door")

    left_spin = object_model.get_articulation("left_reel_spin")
    center_spin = object_model.get_articulation("center_reel_spin")
    right_spin = object_model.get_articulation("right_reel_spin")
    handle_pull = object_model.get_articulation("handle_pull")
    coin_door_hinge = object_model.get_articulation("coin_door_hinge")

    for part in (cabinet, left_reel, center_reel, right_reel, handle, coin_door):
        ctx.check(f"{part.name} present", part is not None, details="")

    for joint in (left_spin, center_spin, right_spin):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is continuous on cabinet-horizontal axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    left_marker_rest = _aabb_center(ctx.part_element_world_aabb(left_reel, elem="marker"))
    with ctx.pose({left_spin: 1.2}):
        left_marker_spun = _aabb_center(ctx.part_element_world_aabb(left_reel, elem="marker"))
    ctx.check(
        "left reel marker moves when reel spins",
        left_marker_rest is not None
        and left_marker_spun is not None
        and abs(left_marker_spun[2] - left_marker_rest[2]) > 0.020,
        details=f"rest={left_marker_rest}, spun={left_marker_spun}",
    )

    handle_knob_rest = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_knob"))
    with ctx.pose({handle_pull: handle_pull.motion_limits.upper}):
        handle_knob_pulled = _aabb_center(ctx.part_element_world_aabb(handle, elem="handle_knob"))
    ctx.check(
        "side handle pulls downward",
        handle_knob_rest is not None
        and handle_knob_pulled is not None
        and handle_knob_pulled[2] < handle_knob_rest[2] - 0.060,
        details=f"rest={handle_knob_rest}, pulled={handle_knob_pulled}",
    )

    door_rest = _aabb_center(ctx.part_element_world_aabb(coin_door, elem="door_panel"))
    with ctx.pose({coin_door_hinge: coin_door_hinge.motion_limits.upper}):
        door_open = _aabb_center(ctx.part_element_world_aabb(coin_door, elem="door_panel"))
    ctx.check(
        "coin tray door swings downward and outward",
        door_rest is not None
        and door_open is not None
        and door_open[2] < door_rest[2] - 0.080
        and door_open[1] > door_rest[1] + 0.060,
        details=f"rest={door_rest}, open={door_open}",
    )

    with ctx.pose({coin_door_hinge: 0.0}):
        ctx.expect_gap(
            coin_door,
            cabinet,
            axis="z",
            min_gap=-1e-6,
            max_gap=0.015,
            positive_elem="door_panel",
            negative_elem="door_sill",
            name="coin door sits just above the sill when closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
