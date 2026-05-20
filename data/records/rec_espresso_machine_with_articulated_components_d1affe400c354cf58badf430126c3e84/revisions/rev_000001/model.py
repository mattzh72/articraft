from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_group_espresso_machine")

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.17, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    portafilter_black = model.material("portafilter_black", rgba=(0.17, 0.12, 0.09, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.56, 0.54, 0.65)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )

    body.visual(
        Box((0.48, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, -0.11, 0.08)),
        material=dark_trim,
        name="base_block",
    )
    body.visual(
        Box((0.05, 0.36, 0.20)),
        origin=Origin(xyz=(-0.235, 0.0, 0.10)),
        material=dark_trim,
        name="left_skirt",
    )
    body.visual(
        Box((0.05, 0.36, 0.20)),
        origin=Origin(xyz=(0.235, 0.0, 0.10)),
        material=dark_trim,
        name="right_skirt",
    )
    body.visual(
        Box((0.42, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, -0.16, 0.08)),
        material=dark_trim,
        name="rear_cross",
    )
    body.visual(
        Box((0.035, 0.10, 0.42)),
        origin=Origin(xyz=(-0.245, 0.205, 0.32)),
        material=stainless,
        name="left_front_post",
    )
    body.visual(
        Box((0.035, 0.10, 0.42)),
        origin=Origin(xyz=(0.245, 0.205, 0.32)),
        material=stainless,
        name="right_front_post",
    )
    body.visual(
        Box((0.035, 0.10, 0.38)),
        origin=Origin(xyz=(-0.245, -0.205, 0.30)),
        material=stainless,
        name="left_rear_post",
    )
    body.visual(
        Box((0.035, 0.10, 0.38)),
        origin=Origin(xyz=(0.245, -0.205, 0.30)),
        material=stainless,
        name="right_rear_post",
    )
    body.visual(
        Box((0.52, 0.03, 0.46)),
        origin=Origin(xyz=(0.0, -0.255, 0.33)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((0.52, 0.44, 0.016)),
        origin=Origin(xyz=(0.0, -0.02, 0.592)),
        material=stainless,
        name="top_deck",
    )
    body.visual(
        Box((0.52, 0.07, 0.09)),
        origin=Origin(xyz=(0.0, 0.225, 0.545)),
        material=stainless,
        name="front_brow",
    )
    body.visual(
        Box((0.15, 0.014, 0.24)),
        origin=Origin(xyz=(-0.155, 0.230, 0.42)),
        material=stainless,
        name="left_fascia",
    )
    body.visual(
        Box((0.15, 0.014, 0.24)),
        origin=Origin(xyz=(0.155, 0.230, 0.42)),
        material=stainless,
        name="right_fascia",
    )
    body.visual(
        Box((0.11, 0.014, 0.154)),
        origin=Origin(xyz=(0.0, 0.230, 0.423)),
        material=stainless,
        name="center_mount_panel",
    )
    body.visual(
        Box((0.52, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.215, 0.215)),
        material=stainless,
        name="tray_bridge",
    )
    body.visual(
        Box((0.018, 0.34, 0.011)),
        origin=Origin(xyz=(-0.17, 0.045, 0.1105)),
        material=satin_steel,
        name="left_guide",
    )
    body.visual(
        Box((0.018, 0.34, 0.011)),
        origin=Origin(xyz=(0.17, 0.045, 0.1105)),
        material=satin_steel,
        name="right_guide",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.279, 0.244, 0.472)),
        material=satin_steel,
        name="steam_pivot_support",
    )
    body.visual(
        Box((0.07, 0.020, 0.05)),
        origin=Origin(xyz=(0.244, 0.220, 0.43)),
        material=stainless,
        name="steam_support_block",
    )

    cup_rail = tube_from_spline_points(
        [
            (-0.22, -0.16, 0.645),
            (-0.22, 0.15, 0.645),
            (0.22, 0.15, 0.645),
            (0.22, -0.16, 0.645),
        ],
        radius=0.005,
        samples_per_segment=18,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
    )
    body.visual(_mesh("cup_rail", cup_rail), material=satin_steel, name="cup_rail")
    for idx, (x, y) in enumerate(((-0.22, -0.16), (-0.22, 0.15), (0.22, 0.15), (0.22, -0.16))):
        body.visual(
            Cylinder(radius=0.006, length=0.045),
            origin=Origin(xyz=(x, y, 0.6225)),
            material=satin_steel,
            name=f"rail_post_{idx}",
        )

    group_head = model.part("group_head")
    group_head.inertial = Inertial.from_geometry(
        Box((0.13, 0.12, 0.12)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.02, 0.045)),
    )
    group_head.visual(
        Box((0.12, 0.050, 0.07)),
        origin=Origin(xyz=(0.0, -0.010, 0.055)),
        material=satin_steel,
        name="mount_block",
    )
    group_head.visual(
        Cylinder(radius=0.039, length=0.085),
        origin=Origin(xyz=(0.0, 0.04, 0.04), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="nose",
    )
    group_head.visual(
        Cylinder(radius=0.031, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stainless,
        name="brew_collar",
    )

    portafilter = model.part("portafilter")
    portafilter.inertial = Inertial.from_geometry(
        Box((0.09, 0.22, 0.07)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.09, -0.02)),
    )
    portafilter.visual(
        Cylinder(radius=0.041, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=satin_steel,
        name="lug_ring",
    )
    portafilter.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=satin_steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.018, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.045, -0.028)),
        material=satin_steel,
        name="neck",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, 0.125, -0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=portafilter_black,
        name="grip",
    )
    portafilter.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.0, 0.205, -0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="end_cap",
    )

    steam_wand = model.part("steam_wand")
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.08, 0.10, 0.30)),
        mass=0.35,
        origin=Origin(xyz=(-0.02, 0.03, -0.14)),
    )
    steam_wand.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=satin_steel,
        name="pivot_collar",
    )
    wand_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.024),
            (-0.006, 0.008, -0.050),
            (-0.016, 0.020, -0.090),
            (-0.029, 0.038, -0.160),
            (-0.041, 0.057, -0.245),
        ],
        radius=0.0045,
        samples_per_segment=18,
        radial_segments=16,
    )
    steam_wand.visual(_mesh("steam_wand_tube", wand_geom), material=satin_steel, name="tube")
    steam_wand.visual(
        Cylinder(radius=0.0038, length=0.026),
        origin=Origin(xyz=(-0.041, 0.057, -0.258), rpy=(0.5, 0.12, 0.0)),
        material=satin_steel,
        name="tip",
    )

    left_panel = model.part("left_panel")
    left_panel.inertial = Inertial.from_geometry(
        Box((0.02, 0.39, 0.34)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -0.195, 0.0)),
    )
    left_panel.visual(
        Box((0.012, 0.39, 0.34)),
        origin=Origin(xyz=(0.0, -0.195, 0.0)),
        material=stainless,
        name="panel",
    )
    left_panel.visual(
        Box((0.006, 0.29, 0.20)),
        origin=Origin(xyz=(0.003, -0.205, 0.0)),
        material=satin_steel,
        name="bead",
    )
    left_panel.visual(
        Cylinder(radius=0.005, length=0.10),
        origin=Origin(xyz=(-0.006, -0.09, 0.0)),
        material=black,
        name="pull",
    )

    right_panel = model.part("right_panel")
    right_panel.inertial = Inertial.from_geometry(
        Box((0.02, 0.39, 0.34)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -0.195, 0.0)),
    )
    right_panel.visual(
        Box((0.012, 0.39, 0.34)),
        origin=Origin(xyz=(0.0, -0.195, 0.0)),
        material=stainless,
        name="panel",
    )
    right_panel.visual(
        Box((0.006, 0.29, 0.20)),
        origin=Origin(xyz=(-0.003, -0.205, 0.0)),
        material=satin_steel,
        name="bead",
    )
    right_panel.visual(
        Cylinder(radius=0.005, length=0.10),
        origin=Origin(xyz=(0.006, -0.09, 0.0)),
        material=black,
        name="pull",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.40, 0.23, 0.05)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )
    tray.visual(
        Box((0.40, 0.23, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_trim,
        name="floor",
    )
    tray.visual(
        Box((0.40, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.110, 0.0175)),
        material=satin_steel,
        name="front_wall",
    )
    tray.visual(
        Box((0.40, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.110, 0.0105)),
        material=satin_steel,
        name="rear_wall",
    )
    tray.visual(
        Box((0.010, 0.21, 0.032)),
        origin=Origin(xyz=(-0.195, 0.0, 0.0135)),
        material=satin_steel,
        name="left_wall",
    )
    tray.visual(
        Box((0.010, 0.21, 0.032)),
        origin=Origin(xyz=(0.195, 0.0, 0.0135)),
        material=satin_steel,
        name="right_wall",
    )
    tray.visual(
        Box((0.42, 0.014, 0.015)),
        origin=Origin(xyz=(0.0, 0.119, 0.0375)),
        material=satin_steel,
        name="front_lip",
    )
    grate = SlotPatternPanelGeometry(
        (0.384, 0.212),
        0.003,
        slot_size=(0.032, 0.004),
        pitch=(0.050, 0.015),
        frame=0.012,
        corner_radius=0.008,
        center=False,
    )
    tray.visual(
        _mesh("tray_grate", grate),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="grate",
    )

    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.0, 0.272, 0.34)),
    )
    model.articulation(
        "group_head_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=0.12),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.279, 0.244, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-1.1, upper=1.1),
    )
    model.articulation(
        "body_to_left_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_panel,
        origin=Origin(xyz=(-0.2685, 0.185, 0.36)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.3),
    )
    model.articulation(
        "body_to_right_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_panel,
        origin=Origin(xyz=(0.2685, 0.185, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.3),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.135, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=0.17),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    group_head = object_model.get_part("group_head")
    tray = object_model.get_part("tray")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")

    tray_joint = object_model.get_articulation("body_to_tray")
    portafilter_joint = object_model.get_articulation("group_head_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    left_panel_joint = object_model.get_articulation("body_to_left_panel")
    right_panel_joint = object_model.get_articulation("body_to_right_panel")

    ctx.expect_overlap(
        tray,
        group_head,
        axes="x",
        min_overlap=0.10,
        name="drip tray spans beneath the brew group",
    )
    ctx.expect_gap(
        group_head,
        tray,
        axis="z",
        min_gap=0.14,
        max_gap=0.24,
        name="brew group sits above the drip tray",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.17}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="x",
            margin=0.015,
            name="tray stays between the side skirts when extended",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.06,
            name="extended tray retains guide engagement",
        )
    ctx.check(
        "tray slides forward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.12,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    locked_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="grip"))
    with ctx.pose({portafilter_joint: -0.9}):
        unlocked_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="grip"))
    ctx.check(
        "portafilter rotates out from the brew axis",
        locked_handle is not None
        and unlocked_handle is not None
        and unlocked_handle[0] > locked_handle[0] + 0.08
        and unlocked_handle[1] < locked_handle[1] - 0.04,
        details=f"locked={locked_handle}, unlocked={unlocked_handle}",
    )

    with ctx.pose({steam_joint: -0.8}):
        wand_left = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    with ctx.pose({steam_joint: 0.8}):
        wand_right = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    ctx.check(
        "steam wand sweeps around its vertical pivot",
        wand_left is not None
        and wand_right is not None
        and abs(wand_left[0] - wand_right[0]) > 0.05
        and abs(wand_left[1] - wand_right[1]) > 0.05,
        details=f"pose_a={wand_left}, pose_b={wand_right}",
    )

    closed_left = ctx.part_world_aabb(left_panel)
    closed_right = ctx.part_world_aabb(right_panel)
    with ctx.pose({left_panel_joint: 1.15, right_panel_joint: 1.15}):
        open_left = ctx.part_world_aabb(left_panel)
        open_right = ctx.part_world_aabb(right_panel)
    ctx.check(
        "side service panels open outward",
        closed_left is not None
        and closed_right is not None
        and open_left is not None
        and open_right is not None
        and open_left[0][0] < closed_left[0][0] - 0.10
        and open_right[1][0] > closed_right[1][0] + 0.10,
        details=f"closed_left={closed_left}, open_left={open_left}, closed_right={closed_right}, open_right={open_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
