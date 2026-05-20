from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_ready_dishwasher")

    stainless = model.material("brushed_stainless", color=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("shadow_black", color=(0.025, 0.027, 0.030, 1.0))
    panel_white = model.material("panel_ready_white", color=(0.88, 0.86, 0.80, 1.0))
    rail_metal = model.material("satin_handle_metal", color=(0.60, 0.62, 0.60, 1.0))
    rack_coat = model.material("nylon_rack_gray", color=(0.80, 0.82, 0.82, 1.0))
    blue = model.material("rinse_blue", color=(0.15, 0.38, 0.85, 1.0))
    black_rubber = model.material("black_gasket", color=(0.01, 0.01, 0.012, 1.0))

    def cyl_x(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_z(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    tub = model.part("tub")
    # Built-in 24 inch class envelope: hollow stainless tub, open at the front.
    tub.visual(Box((0.025, 0.62, 0.82)), origin=Origin(xyz=(-0.300, 0.010, 0.430)), material=stainless, name="side_wall_0")
    tub.visual(Box((0.025, 0.62, 0.82)), origin=Origin(xyz=(0.300, 0.010, 0.430)), material=stainless, name="side_wall_1")
    tub.visual(Box((0.60, 0.025, 0.82)), origin=Origin(xyz=(0.0, 0.315, 0.430)), material=stainless, name="back_wall")
    tub.visual(Box((0.60, 0.62, 0.030)), origin=Origin(xyz=(0.0, 0.010, 0.015)), material=stainless, name="floor_pan")
    tub.visual(Box((0.60, 0.62, 0.025)), origin=Origin(xyz=(0.0, 0.010, 0.852)), material=stainless, name="ceiling")
    tub.visual(Box((0.045, 0.020, 0.82)), origin=Origin(xyz=(-0.277, -0.308, 0.430)), material=stainless, name="front_jamb_0")
    tub.visual(Box((0.045, 0.020, 0.82)), origin=Origin(xyz=(0.277, -0.308, 0.430)), material=stainless, name="front_jamb_1")
    tub.visual(Box((0.60, 0.020, 0.045)), origin=Origin(xyz=(0.0, -0.308, 0.838)), material=stainless, name="front_header")
    tub.visual(Box((0.60, 0.018, 0.030)), origin=Origin(xyz=(0.0, -0.310, 0.050)), material=black_rubber, name="bottom_gasket")
    tub.visual(Box((0.010, 0.012, 0.745)), origin=Origin(xyz=(-0.286, -0.323, 0.445)), material=black_rubber, name="side_gasket_0")
    tub.visual(Box((0.010, 0.012, 0.745)), origin=Origin(xyz=(0.286, -0.323, 0.445)), material=black_rubber, name="side_gasket_1")
    tub.visual(Box((0.545, 0.012, 0.014)), origin=Origin(xyz=(0.0, -0.323, 0.813)), material=black_rubber, name="top_gasket")

    # Side slide rails for the three independently sliding rack levels.
    for z, level in ((0.167, "lower"), (0.507, "upper"), (0.742, "tray")):
        tub.visual(Box((0.018, 0.470, 0.018)), origin=Origin(xyz=(-0.260, -0.010, z)), material=rail_metal, name=f"{level}_rail_0")
        tub.visual(Box((0.018, 0.470, 0.018)), origin=Origin(xyz=(0.260, -0.010, z)), material=rail_metal, name=f"{level}_rail_1")
        tub.visual(Box((0.025, 0.030, 0.035)), origin=Origin(xyz=(-0.275, -0.240, z)), material=rail_metal, name=f"{level}_front_clip_0")
        tub.visual(Box((0.025, 0.030, 0.035)), origin=Origin(xyz=(0.275, -0.240, z)), material=rail_metal, name=f"{level}_front_clip_1")

    # Water-feed posts and hinge hardware are attached to the tub assembly.
    cyl_z(tub, "lower_spray_post", 0.018, 0.084, (0.0, -0.020, 0.058), rail_metal)
    tub.visual(Box((0.030, 0.245, 0.026)), origin=Origin(xyz=(0.0, 0.185, 0.455)), material=rail_metal, name="upper_feed_tube")
    cyl_z(tub, "upper_spray_post", 0.016, 0.055, (0.0, 0.060, 0.425), rail_metal)
    cyl_x(tub, "hinge_barrel_0", 0.014, 0.135, (-0.210, -0.336, 0.052), rail_metal)
    cyl_x(tub, "hinge_barrel_1", 0.014, 0.135, (0.210, -0.336, 0.052), rail_metal)
    tub.visual(Box((0.080, 0.020, 0.035)), origin=Origin(xyz=(-0.210, -0.323, 0.042)), material=rail_metal, name="hinge_bracket_0")
    tub.visual(Box((0.080, 0.020, 0.035)), origin=Origin(xyz=(0.210, -0.323, 0.042)), material=rail_metal, name="hinge_bracket_1")

    door = model.part("door")
    # Door frame is authored from the bottom hinge line upward; exterior is a flat panel-ready slab.
    door.visual(Box((0.600, 0.042, 0.790)), origin=Origin(xyz=(0.0, -0.020, 0.430)), material=panel_white, name="front_panel")
    door.visual(Box((0.535, 0.018, 0.610)), origin=Origin(xyz=(0.0, 0.010, 0.405)), material=stainless, name="door_liner")
    door.visual(Box((0.400, 0.020, 0.115)), origin=Origin(xyz=(-0.045, 0.029, 0.340)), material=stainless, name="detergent_recess")
    door.visual(Box((0.410, 0.014, 0.052)), origin=Origin(xyz=(0.0, -0.048, 0.805)), material=dark, name="hidden_top_strip")
    cyl_x(door, "handle_rail", 0.018, 0.520, (0.0, -0.066, 0.675), rail_metal)
    door.visual(Box((0.030, 0.030, 0.065)), origin=Origin(xyz=(-0.225, -0.050, 0.675)), material=rail_metal, name="handle_standoff_0")
    door.visual(Box((0.030, 0.030, 0.065)), origin=Origin(xyz=(0.225, -0.050, 0.675)), material=rail_metal, name="handle_standoff_1")
    cyl_x(door, "door_hinge_barrel", 0.013, 0.230, (0.0, 0.0, 0.000), rail_metal)
    door.visual(Box((0.210, 0.032, 0.040)), origin=Origin(xyz=(0.0, -0.014, 0.023)), material=rail_metal, name="hinge_leaf")
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.336, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=1.65),
    )

    def add_rack(part, width, depth, height, deep=False, bottle=False, tray=False):
        rod = 0.004
        # Rectangular basket perimeter.
        for y in (-depth / 2.0, depth / 2.0):
            cyl_x(part, f"perimeter_x_{y:.2f}", rod, width, (0.0, y, 0.030), rack_coat)
            cyl_x(part, f"top_rail_x_{y:.2f}", rod, width, (0.0, y, height), rack_coat)
        for x in (-width / 2.0, width / 2.0):
            cyl_y(part, f"perimeter_y_{x:.2f}", rod, depth, (x, 0.0, 0.030), rack_coat)
            cyl_y(part, f"top_rail_y_{x:.2f}", rod, depth, (x, 0.0, height), rack_coat)
        for x in (-width / 2.0, width / 2.0):
            for y in (-depth / 2.0, depth / 2.0):
                cyl_z(part, f"corner_post_{x:.2f}_{y:.2f}", rod, height - 0.030, (x, y, (height + 0.030) / 2.0), rack_coat)
        # Open bottom grid.
        for x in (-0.18, -0.09, 0.0, 0.09, 0.18):
            cyl_y(part, f"floor_y_{x:.2f}", 0.0032, depth, (x, 0.0, 0.035), rack_coat)
        for y in (-0.16, -0.08, 0.0, 0.08, 0.16):
            if abs(y) < depth / 2.0:
                cyl_x(part, f"floor_x_{y:.2f}", 0.0032, width, (0.0, y, 0.037), rack_coat)
        # Side slide shoes that visually ride near the tub rails.
        part.visual(Box((0.032, depth * 0.82, 0.018)), origin=Origin(xyz=(-width / 2.0 - 0.010, 0.0, 0.060)), material=rack_coat, name="runner_0")
        part.visual(Box((0.032, depth * 0.82, 0.018)), origin=Origin(xyz=(width / 2.0 + 0.010, 0.0, 0.060)), material=rack_coat, name="runner_1")
        part.visual(Box((0.010, depth * 0.76, 0.052)), origin=Origin(xyz=(-width / 2.0, 0.0, 0.055)), material=rack_coat, name="runner_web_0")
        part.visual(Box((0.010, depth * 0.76, 0.052)), origin=Origin(xyz=(width / 2.0, 0.0, 0.055)), material=rack_coat, name="runner_web_1")
        if deep:
            for x in (-0.20, -0.10, 0.10, 0.20):
                cyl_z(part, f"fixed_plate_tine_{x:.2f}", 0.0035, 0.135, (x, 0.165, 0.100), rack_coat)
        if bottle:
            for x in (-0.18, -0.06, 0.06, 0.18):
                cyl_z(part, f"bottle_loop_post_{x:.2f}", 0.0035, 0.115, (x, -0.105, 0.095), rack_coat)
                cyl_y(part, f"bottle_cradle_{x:.2f}", 0.0035, 0.145, (x, -0.085, 0.150), rack_coat)
            cyl_x(part, "bottle_retainer", 0.004, width * 0.78, (0.0, -0.155, 0.150), rack_coat)
        if tray:
            for x in (-0.20, -0.10, 0.0, 0.10, 0.20):
                cyl_y(part, f"utensil_channel_{x:.2f}", 0.0028, depth, (x, 0.0, 0.073), rack_coat)
            for y in (-0.13, -0.065, 0.0, 0.065, 0.13):
                cyl_x(part, f"utensil_divider_{y:.2f}", 0.0028, width, (0.0, y, 0.073), rack_coat)

    lower_rack = model.part("lower_rack")
    add_rack(lower_rack, width=0.490, depth=0.470, height=0.245, deep=True)
    # Side pivot cradles for the two fold-down tine banks.
    for y, bank in ((-0.095, "front"), (0.085, "rear")):
        lower_rack.visual(Box((0.020, 0.035, 0.035)), origin=Origin(xyz=(-0.245, y, 0.078)), material=rack_coat, name=f"{bank}_pivot_cradle_0")
        lower_rack.visual(Box((0.020, 0.035, 0.035)), origin=Origin(xyz=(0.245, y, 0.078)), material=rack_coat, name=f"{bank}_pivot_cradle_1")
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.010, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.410),
    )

    upper_rack = model.part("upper_rack")
    add_rack(upper_rack, width=0.485, depth=0.455, height=0.175, bottle=True)
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.010, 0.465)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=0.390),
    )

    utensil_tray = model.part("utensil_tray")
    add_rack(utensil_tray, width=0.500, depth=0.430, height=0.095, tray=True)
    model.articulation(
        "utensil_tray_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=utensil_tray,
        origin=Origin(xyz=(0.0, -0.010, 0.700)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=0.0, upper=0.360),
    )

    def add_spray_arm(part, length):
        cyl_z(part, "central_hub", 0.030, 0.016, (0.0, 0.0, 0.0), rail_metal)
        part.visual(Box((length, 0.035, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=rail_metal, name="spray_blade")
        part.visual(Box((0.020, 0.055, 0.014)), origin=Origin(xyz=(-length / 2.0 + 0.030, 0.0, 0.004), rpy=(0.0, 0.0, 0.30)), material=rail_metal, name="swept_tip_0")
        part.visual(Box((0.020, 0.055, 0.014)), origin=Origin(xyz=(length / 2.0 - 0.030, 0.0, 0.004), rpy=(0.0, 0.0, -0.30)), material=rail_metal, name="swept_tip_1")
        for x in (-0.17, -0.08, 0.08, 0.17):
            part.visual(Box((0.018, 0.006, 0.004)), origin=Origin(xyz=(x, 0.010 if x < 0 else -0.010, 0.009)), material=dark, name=f"spray_nozzle_{x:.2f}")

    lower_arm = model.part("lower_spray_arm")
    add_spray_arm(lower_arm, 0.470)
    model.articulation(
        "lower_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_arm,
        origin=Origin(xyz=(0.0, -0.020, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )

    upper_arm = model.part("upper_spray_arm")
    add_spray_arm(upper_arm, 0.405)
    model.articulation(
        "upper_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.060, 0.392)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )

    def make_tine_bank(name, y_offset):
        bank = model.part(name)
        cyl_x(bank, "pivot_rod", 0.0045, 0.438, (0.0, 0.0, 0.0), rack_coat)
        for x in (-0.180, -0.120, -0.060, 0.0, 0.060, 0.120, 0.180):
            cyl_z(bank, f"folding_tine_{x:.2f}", 0.0032, 0.135, (x, 0.0, 0.067), rack_coat)
        bank.visual(Box((0.018, 0.018, 0.018)), origin=Origin(xyz=(-0.226, 0.0, 0.0)), material=rack_coat, name="pivot_boss_0")
        bank.visual(Box((0.018, 0.018, 0.018)), origin=Origin(xyz=(0.226, 0.0, 0.0)), material=rack_coat, name="pivot_boss_1")
        model.articulation(
            f"{name}_pivot",
            ArticulationType.REVOLUTE,
            parent=lower_rack,
            child=bank,
            origin=Origin(xyz=(0.0, y_offset, 0.078)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.35),
        )
        return bank

    make_tine_bank("tine_bank_0", -0.095)
    make_tine_bank("tine_bank_1", 0.085)

    rinse_cap = model.part("rinse_cap")
    rinse_cap.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="round_cap",
    )
    rinse_cap.visual(Box((0.045, 0.004, 0.006)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=dark, name="coin_slot")
    model.articulation(
        "rinse_cap_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=rinse_cap,
        origin=Origin(xyz=(0.215, 0.025, 0.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    for x, label in ((-0.070, "start_button"), (0.070, "cancel_button")):
        button = model.part(label)
        button.visual(Box((0.060, 0.008, 0.020)), origin=Origin(), material=dark, name="button_cap")
        button.visual(Box((0.045, 0.004, 0.012)), origin=Origin(xyz=(0.0, -0.004, 0.0)), material=rail_metal, name="button_face")
        model.articulation(
            f"{label}_press",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.059, 0.805)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    utensil_tray = object_model.get_part("utensil_tray")
    lower_arm = object_model.get_part("lower_spray_arm")
    upper_arm = object_model.get_part("upper_spray_arm")
    tine_bank_0 = object_model.get_part("tine_bank_0")
    tine_bank_1 = object_model.get_part("tine_bank_1")
    start_button = object_model.get_part("start_button")
    cancel_button = object_model.get_part("cancel_button")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_slide = object_model.get_articulation("lower_rack_slide")
    upper_slide = object_model.get_articulation("upper_rack_slide")
    tray_slide = object_model.get_articulation("utensil_tray_slide")
    lower_spin = object_model.get_articulation("lower_spray_spin")
    upper_spin = object_model.get_articulation("upper_spray_spin")
    cap_spin = object_model.get_articulation("rinse_cap_spin")
    bank_0_pivot = object_model.get_articulation("tine_bank_0_pivot")
    bank_1_pivot = object_model.get_articulation("tine_bank_1_pivot")
    start_press = object_model.get_articulation("start_button_press")
    cancel_press = object_model.get_articulation("cancel_button_press")

    ctx.check("door uses a bottom revolute hinge", door_hinge.articulation_type == ArticulationType.REVOLUTE and door_hinge.axis == (1.0, 0.0, 0.0))
    ctx.check("all three racks are prismatic slides", all(j.articulation_type == ArticulationType.PRISMATIC for j in (lower_slide, upper_slide, tray_slide)))
    ctx.check("spray arms and rinse cap are continuous rotary parts", all(j.articulation_type == ArticulationType.CONTINUOUS for j in (lower_spin, upper_spin, cap_spin)))
    ctx.check("top strip buttons press independently", start_press.articulation_type == ArticulationType.PRISMATIC and cancel_press.articulation_type == ArticulationType.PRISMATIC)

    # At rest the three open rack baskets sit inside the hollow tub envelope.
    for rack, label in ((lower_rack, "lower rack"), (upper_rack, "upper rack"), (utensil_tray, "utensil tray")):
        ctx.expect_within(rack, tub, axes="xz", margin=0.0, name=f"{label} stays inside tub width and height")
    ctx.expect_origin_gap(upper_rack, lower_rack, axis="z", min_gap=0.25, name="upper bottle rack is above the deep lower rack")
    ctx.expect_origin_gap(utensil_tray, upper_rack, axis="z", min_gap=0.18, name="utensil tray is tucked above the upper rack")
    ctx.expect_within(lower_arm, tub, axes="xy", margin=0.0, name="lower spray arm spins inside tub footprint")
    ctx.expect_within(upper_arm, tub, axes="xy", margin=0.0, name="upper spray arm spins inside tub footprint")

    def _aabb_span(aabb, axis_index):
        return aabb[1][axis_index] - aabb[0][axis_index]

    door_closed = ctx.part_element_world_aabb(door, elem="front_panel")
    with ctx.pose({door_hinge: 1.45}):
        door_open = ctx.part_element_world_aabb(door, elem="front_panel")
    ctx.check(
        "dropdown door swings outward and down",
        door_closed is not None
        and door_open is not None
        and door_open[0][1] < door_closed[0][1] - 0.45
        and door_open[1][2] < door_closed[1][2] - 0.35,
        details=f"closed={door_closed}, open={door_open}",
    )

    for rack, joint, travel, label in (
        (lower_rack, lower_slide, 0.410, "lower rack"),
        (upper_rack, upper_slide, 0.390, "upper rack"),
        (utensil_tray, tray_slide, 0.360, "utensil tray"),
    ):
        rest = ctx.part_world_position(rack)
        with ctx.pose({joint: travel}):
            extended = ctx.part_world_position(rack)
        ctx.check(
            f"{label} slides outward on its own rails",
            rest is not None and extended is not None and extended[1] < rest[1] - travel * 0.95,
            details=f"rest={rest}, extended={extended}",
        )

    lower_rest = ctx.part_world_aabb(lower_arm)
    with ctx.pose({lower_spin: math.pi / 2.0}):
        lower_rotated = ctx.part_world_aabb(lower_arm)
    upper_rest = ctx.part_world_aabb(upper_arm)
    with ctx.pose({upper_spin: math.pi / 2.0}):
        upper_rotated = ctx.part_world_aabb(upper_arm)
    ctx.check(
        "lower spray arm rotates continuously about its central support",
        lower_rest is not None
        and lower_rotated is not None
        and _aabb_span(lower_rotated, 1) > _aabb_span(lower_rest, 1) + 0.30,
        details=f"rest={lower_rest}, rotated={lower_rotated}",
    )
    ctx.check(
        "upper spray arm rotates continuously about its central support",
        upper_rest is not None
        and upper_rotated is not None
        and _aabb_span(upper_rotated, 1) > _aabb_span(upper_rest, 1) + 0.24,
        details=f"rest={upper_rest}, rotated={upper_rotated}",
    )

    for bank, joint, label in ((tine_bank_0, bank_0_pivot, "front tine bank"), (tine_bank_1, bank_1_pivot, "rear tine bank")):
        upright = ctx.part_world_aabb(bank)
        with ctx.pose({joint: 1.25}):
            folded = ctx.part_world_aabb(bank)
        ctx.check(
            f"{label} folds down on side pivots",
            upright is not None and folded is not None and folded[1][2] < upright[1][2] - 0.07,
            details=f"upright={upright}, folded={folded}",
        )

    start_rest = ctx.part_world_position(start_button)
    cancel_rest = ctx.part_world_position(cancel_button)
    with ctx.pose({start_press: 0.006}):
        start_pressed = ctx.part_world_position(start_button)
        cancel_after_start = ctx.part_world_position(cancel_button)
    with ctx.pose({cancel_press: 0.006}):
        cancel_pressed = ctx.part_world_position(cancel_button)
    ctx.check(
        "start button has its own short push travel",
        start_rest is not None and start_pressed is not None and start_pressed[1] > start_rest[1] + 0.005,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )
    ctx.check(
        "cancel button moves independently of start button",
        cancel_rest is not None
        and cancel_after_start is not None
        and cancel_pressed is not None
        and abs(cancel_after_start[1] - cancel_rest[1]) < 1e-6
        and cancel_pressed[1] > cancel_rest[1] + 0.005,
        details=f"rest={cancel_rest}, after_start={cancel_after_start}, pressed={cancel_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
