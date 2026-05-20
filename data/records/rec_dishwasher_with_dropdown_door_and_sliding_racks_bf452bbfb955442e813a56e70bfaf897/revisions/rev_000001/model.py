from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _rod_x(part, name: str, x: float, y: float, z: float, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_y(part, name: str, x: float, y: float, z: float, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_z(part, name: str, x: float, y: float, z: float, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def _basket(
    part,
    *,
    width: float,
    depth: float,
    rim_z: float,
    side_height: float,
    tine_height: float,
    material,
    prefix: str,
) -> None:
    """Connected wire basket geometry in the rack part frame.

    The part frame is on the centerline of the two side runners.  The runner
    rods sit on fixed guide rails; all grid, side, and tine rods touch or
    slightly penetrate those rods so the rack reads as one welded basket.
    """

    runner_x = width / 2.0
    rod = 0.006
    # Runners that ride on the side guides.
    _rod_y(part, f"{prefix}_runner_0", -runner_x, 0.0, 0.0, depth, rod, material)
    _rod_y(part, f"{prefix}_runner_1", runner_x, 0.0, 0.0, depth, rod, material)

    # Bottom grid and rim.
    for i, x in enumerate((-runner_x, -width * 0.25, 0.0, width * 0.25, runner_x)):
        _rod_y(part, f"{prefix}_grid_y_{i}", x, 0.0, rim_z, depth * 0.92, 0.0045, material)
    for i, y in enumerate((-depth * 0.46, -depth * 0.23, 0.0, depth * 0.23, depth * 0.46)):
        _rod_x(part, f"{prefix}_grid_x_{i}", 0.0, y, rim_z, width, 0.0045, material)

    _rod_y(part, f"{prefix}_side_rail_0", -runner_x, 0.0, side_height, depth, rod, material)
    _rod_y(part, f"{prefix}_side_rail_1", runner_x, 0.0, side_height, depth, rod, material)
    _rod_x(part, f"{prefix}_front_rail", 0.0, -depth / 2.0, side_height, width, rod, material)
    _rod_x(part, f"{prefix}_rear_rail", 0.0, depth / 2.0, side_height, width, rod, material)

    # Corner posts connect runners, bottom grid, and upper rim.
    for i, x in enumerate((-runner_x, runner_x)):
        for j, y in enumerate((-depth / 2.0, depth / 2.0)):
            _rod_z(
                part,
                f"{prefix}_post_{i}_{j}",
                x,
                y,
                side_height / 2.0,
                side_height + 0.004,
                0.0045,
                material,
            )

    # Dish tines: a staggered set of upright rods welded to the bottom grid.
    for side, x in enumerate((-runner_x, runner_x)):
        for j, y in enumerate((-depth * 0.46, -depth * 0.23, 0.0, depth * 0.23, depth * 0.46)):
            _rod_z(
                part,
                f"{prefix}_grid_support_{side}_{j}",
                x,
                y,
                rim_z / 2.0,
                rim_z + 0.006,
                0.0038,
                material,
            )

    for i, x in enumerate((-width * 0.25, -width * 0.25, 0.0, width * 0.25, width * 0.25)):
        for j, y in enumerate((-depth * 0.28, -depth * 0.10, depth * 0.10, depth * 0.28)):
            _rod_z(
                part,
                f"{prefix}_tine_{i}_{j}",
                x,
                y,
                rim_z + tine_height / 2.0,
                tine_height,
                0.0038,
                material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_dishwasher")

    stainless = _mat(model, "brushed_stainless", (0.72, 0.72, 0.69, 1.0))
    dark_steel = _mat(model, "dark_steel", (0.10, 0.11, 0.12, 1.0))
    tub_steel = _mat(model, "warm_tub_stainless", (0.82, 0.84, 0.82, 1.0))
    black = _mat(model, "black_glass", (0.01, 0.012, 0.015, 1.0))
    grey = _mat(model, "soft_grey_plastic", (0.32, 0.34, 0.35, 1.0))
    wire = _mat(model, "coated_grey_wire", (0.54, 0.57, 0.58, 1.0))
    blue = _mat(model, "cool_blue_indicator", (0.05, 0.22, 0.85, 1.0))
    highlight = _mat(model, "brushed_highlight", (0.92, 0.92, 0.89, 0.45))

    # Root part: the built-in dishwasher tub is intentionally a hollow open-front
    # wash chamber made from walls, not a solid block.
    tub = model.part("tub")
    tub.visual(Box((0.65, 0.026, 0.80)), origin=Origin(xyz=(0.0, 0.300, 0.440)), material=tub_steel, name="back_wall")
    tub.visual(Box((0.026, 0.62, 0.80)), origin=Origin(xyz=(-0.313, 0.0, 0.440)), material=tub_steel, name="side_wall_0")
    tub.visual(Box((0.026, 0.62, 0.80)), origin=Origin(xyz=(0.313, 0.0, 0.440)), material=tub_steel, name="side_wall_1")
    tub.visual(Box((0.65, 0.62, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=tub_steel, name="floor_pan")
    tub.visual(Box((0.65, 0.62, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.842)), material=tub_steel, name="ceiling_pan")
    tub.visual(Box((0.650, 0.035, 0.045)), origin=Origin(xyz=(0.0, -0.318, 0.071)), material=dark_steel, name="front_sill")
    tub.visual(Box((0.650, 0.035, 0.050)), origin=Origin(xyz=(0.0, -0.318, 0.802)), material=dark_steel, name="front_header")
    tub.visual(Box((0.032, 0.035, 0.750)), origin=Origin(xyz=(-0.314, -0.318, 0.436)), material=dark_steel, name="front_jamb_0")
    tub.visual(Box((0.032, 0.035, 0.750)), origin=Origin(xyz=(0.314, -0.318, 0.436)), material=dark_steel, name="front_jamb_1")
    tub.visual(Box((0.580, 0.010, 0.018)), origin=Origin(xyz=(0.0, -0.338, 0.810)), material=black, name="black_gasket_top")
    tub.visual(Box((0.014, 0.010, 0.720)), origin=Origin(xyz=(-0.291, -0.338, 0.432)), material=black, name="black_gasket_0")
    tub.visual(Box((0.014, 0.010, 0.720)), origin=Origin(xyz=(0.291, -0.338, 0.432)), material=black, name="black_gasket_1")

    # Rack guide rails welded to both side walls: third tray, upper rack, lower rack.
    for rail_z, label in ((0.188, "lower"), (0.518, "upper"), (0.710, "third")):
        tub.visual(Box((0.048, 0.560, 0.012)), origin=Origin(xyz=(-0.276, 0.0, rail_z)), material=grey, name=f"{label}_guide_0")
        tub.visual(Box((0.048, 0.560, 0.012)), origin=Origin(xyz=(0.276, 0.0, rail_z)), material=grey, name=f"{label}_guide_1")

    # Sump and upper feed plumbing visibly mount the two spray arms inside the tub.
    _rod_z(tub, "lower_sump_post", 0.0, 0.0, 0.068, 0.086, 0.026, dark_steel)
    _rod_z(tub, "rear_feed_pipe", 0.0, 0.315, 0.235, 0.370, 0.014, grey)
    tub.visual(Box((0.026, 0.350, 0.018)), origin=Origin(xyz=(0.0, 0.155, 0.414)), material=grey, name="upper_feed_duct")
    _rod_z(tub, "upper_arm_boss", 0.0, 0.0, 0.418, 0.016, 0.022, dark_steel)

    # Exposed bottom hinge knuckles fixed to the tub.
    _rod_x(tub, "hinge_knuckle_0", -0.220, -0.412, 0.055, 0.125, 0.018, dark_steel)
    _rod_x(tub, "hinge_knuckle_1", 0.220, -0.412, 0.055, 0.125, 0.018, dark_steel)
    tub.visual(Box((0.150, 0.050, 0.020)), origin=Origin(xyz=(-0.220, -0.430, 0.055)), material=dark_steel, name="hinge_bracket_0")
    tub.visual(Box((0.150, 0.050, 0.020)), origin=Origin(xyz=(0.220, -0.430, 0.055)), material=dark_steel, name="hinge_bracket_1")
    tub.visual(Box((0.035, 0.105, 0.018)), origin=Origin(xyz=(-0.220, -0.372, 0.055)), material=dark_steel, name="hinge_arm_0")
    tub.visual(Box((0.035, 0.105, 0.018)), origin=Origin(xyz=(0.220, -0.372, 0.055)), material=dark_steel, name="hinge_arm_1")

    # Downward-opening stainless door.  Its local frame is on the bottom hinge.
    door = model.part("door")
    door.visual(Box((0.630, 0.045, 0.680)), origin=Origin(xyz=(0.0, -0.015, 0.425)), material=stainless, name="stainless_skin")
    door.visual(Box((0.590, 0.008, 0.055)), origin=Origin(xyz=(0.0, -0.039, 0.690)), material=black, name="hidden_control_band")
    door.visual(Box((0.500, 0.010, 0.030)), origin=Origin(xyz=(0.0, -0.041, 0.615)), material=dark_steel, name="recessed_handle_shadow")
    door.visual(Box((0.610, 0.012, 0.024)), origin=Origin(xyz=(0.0, 0.010, 0.745)), material=grey, name="inner_vent_strip")
    _rod_x(door, "door_hinge_barrel", 0.0, 0.0, 0.0, 0.220, 0.017, stainless)
    door.visual(Box((0.024, 0.012, 0.100)), origin=Origin(xyz=(-0.095, -0.004, 0.050)), material=stainless, name="hinge_leaf_0")
    door.visual(Box((0.024, 0.012, 0.100)), origin=Origin(xyz=(0.095, -0.004, 0.050)), material=stainless, name="hinge_leaf_1")
    for i, x in enumerate((-0.22, 0.0, 0.22)):
        door.visual(Box((0.002, 0.006, 0.700)), origin=Origin(xyz=(x, -0.035, 0.385)), material=highlight, name=f"subtle_brush_line_{i}")

    door_hinge = model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.355, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    # User controls in the hidden band: one continuous rotary dial and six
    # independent push buttons.
    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="dial_cap",
    )
    dial.visual(Box((0.006, 0.004, 0.045)), origin=Origin(xyz=(0.0, -0.028, 0.0)), material=blue, name="dial_index")
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.245, -0.043, 0.690)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    button_xs = [-0.150, -0.095, -0.040, 0.015, 0.070, 0.125]
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(Box((0.038, 0.014, 0.020)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=grey, name="button_stem")
        button.visual(Box((0.045, 0.012, 0.026)), origin=Origin(xyz=(0.0, -0.019, 0.0)), material=black, name="button_cap")
        button.visual(Box((0.030, 0.003, 0.004)), origin=Origin(xyz=(0.0, -0.026, 0.008)), material=blue, name="preset_mark")
        model.articulation(
            f"door_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.044, 0.690)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.008),
        )

    # Sliding racks on their own side guides.
    lower_rack = model.part("lower_rack")
    _basket(lower_rack, width=0.500, depth=0.550, rim_z=0.030, side_height=0.135, tine_height=0.150, material=wire, prefix="lower")
    lower_rack.visual(Box((0.440, 0.018, 0.045)), origin=Origin(xyz=(0.0, -0.282, 0.128)), material=grey, name="deep_front_handle")
    lower_slide = model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.460),
    )

    upper_rack = model.part("upper_rack")
    _basket(upper_rack, width=0.500, depth=0.530, rim_z=0.028, side_height=0.095, tine_height=0.120, material=wire, prefix="upper")
    upper_rack.visual(Box((0.360, 0.014, 0.030)), origin=Origin(xyz=(0.0, -0.272, 0.090)), material=grey, name="upper_front_handle")
    upper_rack.visual(Box((0.024, 0.450, 0.024)), origin=Origin(xyz=(0.248, 0.0, 0.096)), material=grey, name="glass_pivot_side")
    upper_slide = model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.0, 0.529)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.430),
    )

    third_tray = model.part("third_tray")
    _basket(third_tray, width=0.500, depth=0.500, rim_z=0.020, side_height=0.048, tine_height=0.035, material=wire, prefix="third")
    third_tray.visual(Box((0.480, 0.012, 0.020)), origin=Origin(xyz=(0.0, -0.255, 0.037)), material=grey, name="narrow_tray_lip")
    third_slide = model.articulation(
        "tub_to_third_tray",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=third_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.721)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.32, lower=0.0, upper=0.390),
    )

    # Fold-down stemware shelf on the upper rack.  At q=0 it is stowed vertical;
    # positive rotation folds it down inward on the side pivot line.
    glass_rack = model.part("glass_rack")
    _rod_y(glass_rack, "pivot_rail", 0.0, 0.0, 0.0, 0.430, 0.008, grey)
    _rod_y(glass_rack, "outer_shelf_rail", -0.115, 0.0, 0.060, 0.410, 0.005, wire)
    _rod_y(glass_rack, "inner_shelf_rail", -0.035, 0.0, 0.060, 0.410, 0.005, wire)
    _rod_y(glass_rack, "support_shelf_rail", -0.012, 0.0, 0.060, 0.410, 0.004, wire)
    for i, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        _rod_x(glass_rack, f"glass_cross_{i}", -0.058, y, 0.060, 0.120, 0.004, wire)
        _rod_z(glass_rack, f"glass_drop_link_{i}", 0.0, y, 0.030, 0.064, 0.0038, wire)
    model.articulation(
        "upper_rack_to_glass_rack",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=glass_rack,
        origin=Origin(xyz=(0.268, 0.0, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    # Lower and upper spray arms, both continuous rotating mechanisms.
    lower_arm = model.part("lower_spray_arm")
    lower_arm.visual(Box((0.470, 0.048, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=grey, name="lower_blade")
    lower_arm.visual(Box((0.160, 0.060, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)), material=grey, name="lower_cross_blade")
    lower_arm.visual(Cylinder(radius=0.035, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel, name="lower_hub")
    for i, x in enumerate((-0.170, -0.080, 0.080, 0.170)):
        lower_arm.visual(Box((0.025, 0.006, 0.006)), origin=Origin(xyz=(x, 0.010 if i % 2 else -0.010, 0.010)), material=black, name=f"lower_nozzle_{i}")
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=18.0),
    )

    upper_arm = model.part("upper_spray_arm")
    upper_arm.visual(Box((0.420, 0.042, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=grey, name="upper_blade")
    upper_arm.visual(Box((0.135, 0.052, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)), material=grey, name="upper_cross_blade")
    upper_arm.visual(Cylinder(radius=0.030, length=0.025), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel, name="upper_hub")
    for i, x in enumerate((-0.145, -0.060, 0.060, 0.145)):
        upper_arm.visual(Box((0.022, 0.005, 0.006)), origin=Origin(xyz=(x, -0.009 if i % 2 else 0.009, 0.009)), material=black, name=f"upper_nozzle_{i}")
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=18.0),
    )

    model.meta["primary_mechanisms"] = {
        "door": door_hinge.name,
        "rack_slides": [lower_slide.name, upper_slide.name, third_slide.name],
        "spray_arms": ["tub_to_lower_spray_arm", "tub_to_upper_spray_arm"],
        "controls": ["door_to_dial", *[f"door_to_button_{i}" for i in range(len(button_xs))]],
        "fold_down_glass_rack": "upper_rack_to_glass_rack",
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    third_tray = object_model.get_part("third_tray")
    glass_rack = object_model.get_part("glass_rack")

    door_hinge = object_model.get_articulation("tub_to_door")
    lower_slide = object_model.get_articulation("tub_to_lower_rack")
    upper_slide = object_model.get_articulation("tub_to_upper_rack")
    third_slide = object_model.get_articulation("tub_to_third_tray")
    glass_hinge = object_model.get_articulation("upper_rack_to_glass_rack")

    ctx.expect_overlap(door, tub, axes="x", min_overlap=0.50, elem_a="stainless_skin", elem_b="front_header", name="door spans built-in width")
    ctx.expect_gap(tub, door, axis="y", min_gap=0.010, max_gap=0.030, positive_elem="front_jamb_0", negative_elem="stainless_skin", name="closed door stands just proud of tub frame")
    ctx.expect_within(lower_rack, tub, axes="xy", margin=0.010, outer_elem="floor_pan", name="lower rack rests inside tub footprint")
    ctx.expect_within(upper_rack, tub, axes="xy", margin=0.010, outer_elem="floor_pan", name="upper rack rests inside tub footprint")
    ctx.expect_within(third_tray, tub, axes="xy", margin=0.010, outer_elem="floor_pan", name="third tray rests inside tub footprint")
    ctx.expect_contact(glass_rack, upper_rack, elem_b="glass_pivot_side", contact_tol=0.001, name="glass rack pivot bears on upper-rack side pivot")

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward on bottom hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.45
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.20,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    for rack, joint, label, travel in (
        (lower_rack, lower_slide, "lower rack", 0.430),
        (upper_rack, upper_slide, "upper rack", 0.390),
        (third_tray, third_slide, "third tray", 0.350),
    ):
        rest = ctx.part_world_position(rack)
        with ctx.pose({joint: travel}):
            extended = ctx.part_world_position(rack)
        ctx.check(
            f"{label} slides out on prismatic guides",
            rest is not None and extended is not None and extended[1] < rest[1] - 0.30,
            details=f"rest={rest}, extended={extended}",
        )

    stowed = ctx.part_world_position(glass_rack)
    with ctx.pose({glass_hinge: 1.20}):
        deployed = ctx.part_world_position(glass_rack)
        ctx.expect_overlap(glass_rack, upper_rack, axes="y", min_overlap=0.30, elem_b="glass_pivot_side", name="glass rack stays on side pivots")
    ctx.check(
        "glass rack folds down from side pivot",
        stowed is not None and deployed is not None and abs(deployed[0] - stowed[0]) < 0.02,
        details=f"stowed={stowed}, deployed={deployed}",
    )

    # Button stems intentionally move into the hidden control band when pressed.
    # At rest they are seated flush, so the baseline overlap pass remains clean.
    for i in range(6):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"door_to_button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.008}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"preset button {i} depresses inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.006,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
