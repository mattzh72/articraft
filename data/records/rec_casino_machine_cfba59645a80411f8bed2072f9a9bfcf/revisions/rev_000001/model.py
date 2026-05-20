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


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _build_reel(part, *, drum_material, accent_material, metal_material) -> None:
    part.visual(
        Cylinder(radius=0.082, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_material,
        name="drum_shell",
    )
    part.visual(
        Cylinder(radius=0.087, length=0.008),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="left_flange",
    )
    part.visual(
        Cylinder(radius=0.087, length=0.008),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="right_flange",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.122),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="shaft",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.122),
        mass=2.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_slot_machine")

    walnut = model.material("walnut", rgba=(0.29, 0.18, 0.09, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.10, 0.07, 0.05, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.64, 0.24, 1.0))
    chrome = model.material("chrome", rgba=(0.75, 0.78, 0.82, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.55, 0.72, 0.78, 0.28))
    off_white = model.material("off_white", rgba=(0.93, 0.91, 0.84, 1.0))
    reel_red = model.material("reel_red", rgba=(0.72, 0.12, 0.12, 1.0))
    reel_blue = model.material("reel_blue", rgba=(0.16, 0.34, 0.70, 1.0))
    reel_gold = model.material("reel_gold", rgba=(0.87, 0.71, 0.24, 1.0))
    handle_red = model.material("handle_red", rgba=(0.72, 0.06, 0.08, 1.0))
    tray_black = model.material("tray_black", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.58, 0.03, 1.52)),
        origin=Origin(xyz=(0.0, -0.375, 0.76)),
        material=walnut,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((0.58, 0.03, 1.52)),
        origin=Origin(xyz=(0.0, 0.375, 0.76)),
        material=walnut,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((0.03, 0.72, 1.48)),
        origin=Origin(xyz=(-0.285, 0.0, 0.74)),
        material=walnut,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.55, 0.72, 0.05)),
        origin=Origin(xyz=(-0.010, 0.0, 0.025)),
        material=dark_panel,
        name="base_floor",
    )
    cabinet.visual(
        Box((0.52, 0.72, 0.04)),
        origin=Origin(xyz=(-0.020, 0.0, 1.54)),
        material=walnut,
        name="roof_cap",
    )
    cabinet.visual(
        Box((0.16, 0.68, 0.30)),
        origin=Origin(xyz=(0.205, 0.0, 1.39)),
        material=dark_panel,
        name="marquee_box",
    )
    cabinet.visual(
        Box((0.012, 0.58, 0.18)),
        origin=Origin(xyz=(0.281, 0.0, 1.39)),
        material=brass,
        name="marquee_face_trim",
    )
    cabinet.visual(
        Box((0.025, 0.72, 0.09)),
        origin=Origin(xyz=(0.2775, 0.0, 1.185)),
        material=brass,
        name="window_top_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.72, 0.09)),
        origin=Origin(xyz=(0.2775, 0.0, 0.775)),
        material=brass,
        name="window_bottom_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.105, 0.40)),
        origin=Origin(xyz=(0.2775, -0.3075, 0.98)),
        material=brass,
        name="window_left_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.105, 0.40)),
        origin=Origin(xyz=(0.2775, 0.3075, 0.98)),
        material=brass,
        name="window_right_bezel",
    )
    cabinet.visual(
        Box((0.018, 0.015, 0.36)),
        origin=Origin(xyz=(0.257, -0.085, 0.98)),
        material=brass,
        name="window_mullion_left",
    )
    cabinet.visual(
        Box((0.018, 0.015, 0.36)),
        origin=Origin(xyz=(0.257, 0.085, 0.98)),
        material=brass,
        name="window_mullion_right",
    )
    cabinet.visual(
        Box((0.025, 0.72, 0.07)),
        origin=Origin(xyz=(0.2775, 0.0, 0.245)),
        material=brass,
        name="tray_top_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.18, 0.17)),
        origin=Origin(xyz=(0.2775, -0.27, 0.16)),
        material=brass,
        name="tray_left_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.18, 0.17)),
        origin=Origin(xyz=(0.2775, 0.27, 0.16)),
        material=brass,
        name="tray_right_bezel",
    )
    cabinet.visual(
        Box((0.025, 0.72, 0.42)),
        origin=Origin(xyz=(0.2775, 0.0, 0.49)),
        material=dark_panel,
        name="center_front_panel",
    )
    cabinet.visual(
        Box((0.03, 0.324, 0.15)),
        origin=Origin(xyz=(0.065, 0.0, 0.17)),
        material=tray_black,
        name="tray_back_wall",
    )
    cabinet.visual(
        Box((0.22, 0.324, 0.018)),
        origin=Origin(xyz=(0.175, 0.0, 0.094)),
        material=tray_black,
        name="tray_shelf",
    )
    cabinet.visual(
        Box((0.22, 0.018, 0.15)),
        origin=Origin(xyz=(0.175, -0.171, 0.17)),
        material=tray_black,
        name="tray_left_wall",
    )
    cabinet.visual(
        Box((0.22, 0.018, 0.15)),
        origin=Origin(xyz=(0.175, 0.171, 0.17)),
        material=tray_black,
        name="tray_right_wall",
    )
    cabinet.visual(
        Box((0.06, 0.58, 0.38)),
        origin=Origin(xyz=(0.025, 0.0, 0.98)),
        material=tray_black,
        name="reel_backdrop",
    )
    cabinet.visual(
        Box((0.08, 0.72, 0.04)),
        origin=Origin(xyz=(0.045, 0.0, 1.125)),
        material=dark_panel,
        name="reel_carrier_upper",
    )
    cabinet.visual(
        Box((0.08, 0.72, 0.04)),
        origin=Origin(xyz=(0.045, 0.0, 0.835)),
        material=dark_panel,
        name="reel_carrier_lower",
    )
    for suffix, center_y in (("left", -0.17), ("center", 0.0), ("right", 0.17)):
        cabinet.visual(
            Box((0.06, 0.018, 0.30)),
            origin=Origin(xyz=(0.095, center_y - 0.069, 0.98)),
            material=dark_panel,
            name=f"{suffix}_reel_bracket_left",
        )
        cabinet.visual(
            Box((0.06, 0.018, 0.30)),
            origin=Origin(xyz=(0.095, center_y + 0.069, 0.98)),
            material=dark_panel,
            name=f"{suffix}_reel_bracket_right",
        )
    cabinet.visual(
        Box((0.018, 0.02, 1.46)),
        origin=Origin(xyz=(0.286, -0.345, 0.73)),
        material=chrome,
        name="left_front_corner_trim",
    )
    cabinet.visual(
        Box((0.018, 0.02, 1.46)),
        origin=Origin(xyz=(0.286, 0.345, 0.73)),
        material=chrome,
        name="right_front_corner_trim",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.60, 0.78, 1.58)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
    )

    window_glass = model.part("window_glass")
    window_glass.visual(
        Box((0.006, 0.50, 0.28)),
        origin=Origin(xyz=(0.245, 0.0, 0.98)),
        material=smoked_glass,
        name="glass_panel",
    )
    window_glass.inertial = Inertial.from_geometry(
        Box((0.006, 0.50, 0.28)),
        mass=0.9,
        origin=Origin(xyz=(0.245, 0.0, 0.98)),
    )
    model.articulation(
        "cabinet_to_window_glass",
        ArticulationType.FIXED,
        parent=cabinet,
        child=window_glass,
        origin=Origin(),
    )

    reel_specs = (
        ("left_reel", -0.17, reel_red),
        ("center_reel", 0.0, reel_blue),
        ("right_reel", 0.17, reel_gold),
    )
    for part_name, y_pos, accent in reel_specs:
        reel = model.part(part_name)
        _build_reel(
            reel,
            drum_material=off_white,
            accent_material=accent,
            metal_material=chrome,
        )
        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(0.145, y_pos, 0.98)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_boss",
    )
    lever_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.060, 0.0, -0.015),
            (0.115, 0.0, -0.090),
            (0.165, 0.0, -0.205),
            (0.172, 0.0, -0.320),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    lever.visual(_mesh("side_lever_arm", lever_arm), material=chrome, name="lever_arm")
    lever.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.188, 0.0, -0.334), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_stem",
    )
    lever.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.232, 0.0, -0.334)),
        material=handle_red,
        name="handle_knob",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.28, 0.08, 0.44)),
        mass=1.6,
        origin=Origin(xyz=(0.11, 0.0, -0.18)),
    )
    model.articulation(
        "cabinet_to_lever",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lever,
        origin=Origin(xyz=(0.145, 0.402, 1.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    tray_flap = model.part("payout_tray_flap")
    tray_flap.visual(
        Box((0.012, 0.30, 0.10)),
        origin=Origin(xyz=(-0.006, 0.0, 0.050)),
        material=brass,
        name="tray_panel",
    )
    tray_flap.visual(
        Box((0.040, 0.26, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.012)),
        material=chrome,
        name="tray_lip",
    )
    tray_flap.inertial = Inertial.from_geometry(
        Box((0.05, 0.30, 0.10)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    model.articulation(
        "cabinet_to_payout_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=(0.267, 0.0, 0.112)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    cabinet = object_model.get_part("cabinet")
    window_glass = object_model.get_part("window_glass")
    side_lever = object_model.get_part("side_lever")
    payout_tray_flap = object_model.get_part("payout_tray_flap")
    left_reel = object_model.get_part("left_reel")
    center_reel = object_model.get_part("center_reel")
    right_reel = object_model.get_part("right_reel")

    lever_joint = object_model.get_articulation("cabinet_to_lever")
    flap_joint = object_model.get_articulation("cabinet_to_payout_tray_flap")
    reel_joints = [
        object_model.get_articulation("cabinet_to_left_reel"),
        object_model.get_articulation("cabinet_to_center_reel"),
        object_model.get_articulation("cabinet_to_right_reel"),
    ]

    ctx.check(
        "slot machine exposes three continuous reel shafts",
        len(reel_joints) == 3
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in reel_joints)
        and all(tuple(j.axis) == (0.0, 1.0, 0.0) for j in reel_joints),
        details=str([(j.name, j.articulation_type, j.axis) for j in reel_joints]),
    )
    ctx.check(
        "lever and tray use horizontal hinge axes",
        tuple(lever_joint.axis) == (0.0, 1.0, 0.0) and tuple(flap_joint.axis) == (0.0, 1.0, 0.0),
        details=f"lever_axis={lever_joint.axis}, flap_axis={flap_joint.axis}",
    )

    reel_positions = [
        ctx.part_world_position(left_reel),
        ctx.part_world_position(center_reel),
        ctx.part_world_position(right_reel),
    ]
    reel_y = [pos[1] if pos is not None else None for pos in reel_positions]
    ctx.check(
        "reels are laterally spaced across the front window",
        None not in reel_y
        and reel_y[0] < reel_y[1] < reel_y[2]
        and (reel_y[1] - reel_y[0]) > 0.14
        and (reel_y[2] - reel_y[1]) > 0.14,
        details=f"reel_y={reel_y}",
    )

    with ctx.pose():
        for reel in (left_reel, center_reel, right_reel):
            ctx.expect_overlap(
                reel,
                window_glass,
                axes="yz",
                min_overlap=0.10,
                name=f"{reel.name} stays within the reel window footprint",
            )
            ctx.expect_gap(
                window_glass,
                reel,
                axis="x",
                min_gap=0.006,
                max_gap=0.030,
                name=f"{reel.name} sits just behind the glass",
            )
        ctx.expect_contact(
            side_lever,
            cabinet,
            elem_a="pivot_boss",
            elem_b="right_side_panel",
            name="lever pivot boss mounts against cabinet side",
        )

    closed_knob_aabb = ctx.part_element_world_aabb(side_lever, elem="handle_knob")
    closed_panel_aabb = ctx.part_element_world_aabb(payout_tray_flap, elem="tray_panel")
    with ctx.pose({lever_joint: 0.85, flap_joint: 1.0}):
        pulled_knob_aabb = ctx.part_element_world_aabb(side_lever, elem="handle_knob")
        open_panel_aabb = ctx.part_element_world_aabb(payout_tray_flap, elem="tray_panel")

    closed_knob_center = _center_from_aabb(closed_knob_aabb)
    pulled_knob_center = _center_from_aabb(pulled_knob_aabb)
    closed_panel_center = _center_from_aabb(closed_panel_aabb)
    open_panel_center = _center_from_aabb(open_panel_aabb)

    ctx.check(
        "lever pull lowers the handle knob",
        closed_knob_center is not None
        and pulled_knob_center is not None
        and pulled_knob_center[2] < closed_knob_center[2] - 0.05,
        details=f"closed={closed_knob_center}, pulled={pulled_knob_center}",
    )
    ctx.check(
        "payout tray flap swings outward and downward",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[0] > closed_panel_center[0] + 0.04
        and open_panel_center[2] < closed_panel_center[2] - 0.015,
        details=f"closed={closed_panel_center}, open={open_panel_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
