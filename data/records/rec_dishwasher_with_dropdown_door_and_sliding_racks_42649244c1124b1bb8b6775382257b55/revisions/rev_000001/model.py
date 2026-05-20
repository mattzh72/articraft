from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DOOR_OPEN = 1.45
RACK_TRAVEL = 0.42
BUTTON_TRAVEL = 0.012
DETERGENT_OPEN = 1.10


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, rpy, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_undercounter_dishwasher")

    stainless = _mat(model, "brushed_stainless", (0.72, 0.74, 0.73, 1.0))
    dark_stainless = _mat(model, "dark_stainless", (0.42, 0.44, 0.45, 1.0))
    shadow = _mat(model, "black_shadow", (0.03, 0.035, 0.04, 1.0))
    rubber = _mat(model, "black_rubber", (0.01, 0.012, 0.013, 1.0))
    wire = _mat(model, "coated_rack_wire", (0.84, 0.86, 0.84, 1.0))
    blue = _mat(model, "blue_indicator", (0.10, 0.35, 0.95, 1.0))
    amber = _mat(model, "amber_indicator", (0.95, 0.55, 0.08, 1.0))
    green = _mat(model, "green_start", (0.04, 0.58, 0.18, 1.0))
    white = _mat(model, "white_label", (0.93, 0.94, 0.92, 1.0))

    # Root body: a real open stainless tub, not a solid block.  The front
    # opening, chamber, rack path, and bottom sump stay empty.
    body = model.part("body")
    _add_box(body, "left_wall", (0.68, 0.035, 0.74), (0.34, -0.3575, 0.47), stainless)
    _add_box(body, "right_wall", (0.68, 0.035, 0.74), (0.34, 0.3575, 0.47), stainless)
    _add_box(body, "rear_wall", (0.035, 0.75, 0.74), (0.6975, 0.0, 0.47), stainless)
    _add_box(body, "top_panel", (0.68, 0.75, 0.035), (0.34, 0.0, 0.8575), stainless)
    _add_box(body, "floor_pan", (0.68, 0.75, 0.035), (0.34, 0.0, 0.0825), stainless)
    _add_box(body, "front_sill", (0.055, 0.75, 0.050), (0.0275, 0.0, 0.105), dark_stainless)
    _add_box(body, "front_header", (0.055, 0.75, 0.050), (0.0275, 0.0, 0.835), dark_stainless)
    _add_box(body, "front_jamb_0", (0.055, 0.045, 0.74), (0.0275, -0.3525, 0.47), dark_stainless)
    _add_box(body, "front_jamb_1", (0.055, 0.045, 0.74), (0.0275, 0.3525, 0.47), dark_stainless)
    _add_box(body, "inner_shadow", (0.012, 0.62, 0.020), (0.36, 0.0, 0.112), shadow)

    # Paired internal slide guides for the deep rack, carried from the side walls.
    for index, y in enumerate((-0.325, 0.325)):
        _add_box(body, f"guide_{index}", (0.58, 0.028, 0.025), (0.34, y, 0.315), dark_stainless)
        _add_box(body, f"guide_web_{index}", (0.58, 0.018, 0.075), (0.34, y + (0.020 if y < 0 else -0.020), 0.282), dark_stainless)

    # Bottom sump features and wash-arm bearing pedestal.
    _add_cylinder(body, "sump_well", 0.105, 0.024, (0.34, 0.0, 0.112), (0.0, 0.0, 0.0), shadow)
    _add_cylinder(body, "sump_lip", 0.125, 0.012, (0.34, 0.0, 0.124), (0.0, 0.0, 0.0), dark_stainless)
    _add_cylinder(body, "arm_bearing", 0.050, 0.026, (0.34, 0.0, 0.117), (0.0, 0.0, 0.0), stainless)

    # Exposed lower hinge knuckles on the frame, with brackets tied into sill.
    for index, y in enumerate((-0.245, 0.245)):
        _add_box(body, f"hinge_bracket_{index}", (0.070, 0.105, 0.020), (-0.035, y, 0.100), dark_stainless)
        _add_cylinder(
            body,
            f"hinge_socket_{index}",
            0.018,
            0.145,
            (-0.080, y, 0.105),
            (math.pi / 2.0, 0.0, 0.0),
            dark_stainless,
        )

    # Heavy dropdown front door. Its local origin is on the bottom hinge line.
    door = model.part("door")
    _add_box(door, "outer_panel", (0.050, 0.730, 0.710), (-0.017, 0.0, 0.385), stainless)
    _add_box(door, "inner_liner", (0.007, 0.620, 0.620), (0.0015, 0.0, 0.395), dark_stainless)
    _add_box(door, "control_panel", (0.012, 0.650, 0.105), (-0.048, 0.0, 0.680), shadow)
    _add_box(door, "nameplate", (0.004, 0.110, 0.018), (-0.056, 0.260, 0.705), white)
    _add_box(door, "seal_top", (0.006, 0.610, 0.020), (0.006, 0.0, 0.700), rubber)
    _add_box(door, "seal_bottom", (0.006, 0.610, 0.020), (0.006, 0.0, 0.160), rubber)
    _add_box(door, "seal_side_0", (0.006, 0.020, 0.540), (0.006, -0.305, 0.430), rubber)
    _add_box(door, "seal_side_1", (0.006, 0.020, 0.540), (0.006, 0.305, 0.430), rubber)
    _add_box(door, "handle_mount_0", (0.064, 0.030, 0.060), (-0.074, -0.215, 0.535), stainless)
    _add_box(door, "handle_mount_1", (0.064, 0.030, 0.060), (-0.074, 0.215, 0.535), stainless)
    _add_cylinder(door, "front_handle", 0.017, 0.460, (-0.110, 0.0, 0.535), (math.pi / 2.0, 0.0, 0.0), stainless)
    _add_cylinder(door, "hinge_knuckle", 0.017, 0.220, (-0.068, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0), dark_stainless)
    _add_cylinder(door, "hinge_pin", 0.006, 0.640, (-0.068, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0), dark_stainless)
    _add_box(door, "hinge_leaf_0", (0.045, 0.030, 0.050), (-0.048, -0.075, 0.025), dark_stainless)
    _add_box(door, "hinge_leaf_1", (0.045, 0.030, 0.050), (-0.048, 0.075, 0.025), dark_stainless)

    # Detergent-cup surround and small hinge sockets on the inner door.
    _add_box(door, "detergent_recess", (0.006, 0.230, 0.165), (0.008, -0.135, 0.430), shadow)
    _add_box(door, "detergent_frame_top", (0.010, 0.230, 0.012), (0.014, -0.135, 0.510), dark_stainless)
    _add_box(door, "detergent_frame_bottom", (0.010, 0.230, 0.012), (0.014, -0.135, 0.337), dark_stainless)
    _add_box(door, "detergent_frame_side_0", (0.010, 0.012, 0.174), (0.014, -0.250, 0.423), dark_stainless)
    _add_box(door, "detergent_frame_side_1", (0.010, 0.012, 0.174), (0.014, -0.020, 0.423), dark_stainless)
    for index, y in enumerate((-0.210, -0.060)):
        _add_cylinder(
            door,
            f"detergent_hinge_socket_{index}",
            0.007,
            0.045,
            (0.016, y, 0.350),
            (math.pi / 2.0, 0.0, 0.0),
            dark_stainless,
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.012, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.9, lower=0.0, upper=DOOR_OPEN),
    )

    # Single deep square rack on runners.  Thin rods cross and touch to read as
    # one welded assembly, leaving a true open basket.
    rack = model.part("rack")
    for index, y in enumerate((-0.305, 0.305)):
        _add_box(rack, f"runner_{index}", (0.540, 0.018, 0.018), (0.0, y, -0.090), wire)
    for index, x in enumerate((-0.260, 0.260)):
        _add_box(rack, f"bottom_cross_{index}", (0.020, 0.600, 0.014), (x, 0.0, -0.075), wire)
    for index, y in enumerate((-0.300, 0.300)):
        _add_box(rack, f"bottom_side_{index}", (0.520, 0.020, 0.014), (0.0, y, -0.075), wire)
    for index, y in enumerate((-0.210, -0.105, 0.0, 0.105, 0.210)):
        _add_box(rack, f"grid_x_{index}", (0.500, 0.010, 0.010), (0.0, y, -0.067), wire)
    for index, x in enumerate((-0.180, -0.090, 0.0, 0.090, 0.180)):
        _add_box(rack, f"grid_y_{index}", (0.010, 0.570, 0.010), (x, 0.0, -0.067), wire)
    post_positions = [
        (-0.260, -0.300),
        (-0.260, 0.300),
        (0.260, -0.300),
        (0.260, 0.300),
        (-0.130, -0.300),
        (0.130, -0.300),
        (-0.130, 0.300),
        (0.130, 0.300),
    ]
    for index, (x, y) in enumerate(post_positions):
        _add_box(rack, f"upright_{index}", (0.014, 0.014, 0.185), (x, y, 0.020), wire)
    _add_box(rack, "top_front", (0.022, 0.610, 0.018), (-0.270, 0.0, 0.112), wire)
    _add_box(rack, "top_rear", (0.022, 0.610, 0.018), (0.270, 0.0, 0.112), wire)
    _add_box(rack, "top_side_0", (0.540, 0.022, 0.018), (0.0, -0.310, 0.112), wire)
    _add_box(rack, "top_side_1", (0.540, 0.022, 0.018), (0.0, 0.310, 0.112), wire)

    model.articulation(
        "body_to_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.340, 0.0, 0.4265)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=RACK_TRAVEL),
    )

    # Rotating lower wash arm assembly above the sump.
    wash_arm = model.part("wash_arm")
    _add_cylinder(wash_arm, "center_hub", 0.050, 0.035, (0.0, 0.0, 0.0175), (0.0, 0.0, 0.0), dark_stainless)
    _add_box(wash_arm, "spray_arm_long", (0.490, 0.048, 0.026), (0.0, 0.0, 0.045), stainless)
    _add_box(wash_arm, "spray_arm_short", (0.050, 0.220, 0.022), (0.0, 0.0, 0.047), stainless)
    for index, x in enumerate((-0.190, -0.110, 0.110, 0.190)):
        _add_box(wash_arm, f"spray_slot_{index}", (0.026, 0.010, 0.004), (x, 0.019, 0.060), shadow)

    model.articulation(
        "body_to_wash_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wash_arm,
        origin=Origin(xyz=(0.340, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )

    # Detergent flap on the inner door, hinged along its lower edge.
    detergent_flap = model.part("detergent_flap")
    _add_box(detergent_flap, "flap_panel", (0.007, 0.170, 0.125), (0.006, 0.0, 0.068), dark_stainless)
    _add_box(detergent_flap, "flap_label", (0.002, 0.080, 0.020), (0.010, 0.0, 0.100), white)
    _add_cylinder(
        detergent_flap,
        "flap_hinge_knuckle",
        0.006,
        0.210,
        (0.001, 0.0, 0.0),
        (math.pi / 2.0, 0.0, 0.0),
        dark_stainless,
    )

    model.articulation(
        "door_to_detergent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_flap,
        origin=Origin(xyz=(0.016, -0.135, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=DETERGENT_OPEN),
    )

    # Front control panel controls: continuous cycle knob and three independent
    # prismatic push buttons.
    cycle_knob = model.part("cycle_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.088,
            0.034,
            body_style="skirted",
            top_diameter=0.068,
            edge_radius=0.002,
            skirt=KnobSkirt(0.104, 0.008, flare=0.06, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=24, depth=0.0016),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "dishwasher_cycle_knob",
    )
    cycle_knob.visual(knob_mesh, origin=Origin(), material=dark_stainless, name="knob_cap")
    model.articulation(
        "door_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cycle_knob,
        origin=Origin(xyz=(-0.054, -0.205, 0.680), rpy=(0.0, -math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    def add_button(part_name: str, joint_name: str, y: float, z: float, material) -> None:
        button = model.part(part_name)
        _add_box(button, "button_cap", (0.018, 0.058, 0.034), (-0.009, 0.0, 0.0), material)
        _add_box(button, "button_bezel", (0.004, 0.070, 0.046), (-0.019, 0.0, 0.0), dark_stainless)
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(-0.054, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    add_button("start_button", "door_to_start_button", -0.030, 0.695, green)
    add_button("indicator_button_0", "door_to_indicator_button_0", 0.095, 0.695, blue)
    add_button("indicator_button_1", "door_to_indicator_button_1", 0.200, 0.695, amber)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    detergent_flap = object_model.get_part("detergent_flap")
    start_button = object_model.get_part("start_button")
    indicator_button_0 = object_model.get_part("indicator_button_0")

    door_joint = object_model.get_articulation("body_to_door")
    rack_joint = object_model.get_articulation("body_to_rack")
    flap_joint = object_model.get_articulation("door_to_detergent_flap")
    arm_joint = object_model.get_articulation("body_to_wash_arm")
    knob_joint = object_model.get_articulation("door_to_cycle_knob")
    start_joint = object_model.get_articulation("door_to_start_button")
    indicator_joint = object_model.get_articulation("door_to_indicator_button_0")

    for socket_name in ("hinge_socket_0", "hinge_socket_1"):
        ctx.allow_overlap(
            body,
            door,
            elem_a=socket_name,
            elem_b="hinge_pin",
            reason="The door hinge pin is intentionally captured inside the frame hinge socket proxy.",
        )
        ctx.expect_within(
            door,
            body,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=socket_name,
            name=f"door hinge pin is centered in {socket_name}",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="y",
            elem_a="hinge_pin",
            elem_b=socket_name,
            min_overlap=0.050,
            name=f"door hinge pin passes through {socket_name}",
        )

    for socket_name in ("detergent_hinge_socket_0", "detergent_hinge_socket_1"):
        ctx.allow_overlap(
            door,
            detergent_flap,
            elem_a=socket_name,
            elem_b="flap_hinge_knuckle",
            reason="The detergent flap hinge knuckle is intentionally retained in the molded door hinge socket.",
        )
        ctx.expect_within(
            detergent_flap,
            door,
            axes="xz",
            inner_elem="flap_hinge_knuckle",
            outer_elem=socket_name,
            name=f"detergent flap hinge is centered in {socket_name}",
        )
        ctx.expect_overlap(
            detergent_flap,
            door,
            axes="y",
            elem_a="flap_hinge_knuckle",
            elem_b=socket_name,
            min_overlap=0.035,
            name=f"detergent flap hinge passes through {socket_name}",
        )

    ctx.check(
        "prompt mechanisms are articulated",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and rack_joint.articulation_type == ArticulationType.PRISMATIC
        and flap_joint.articulation_type == ArticulationType.REVOLUTE
        and arm_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and start_joint.articulation_type == ArticulationType.PRISMATIC
        and indicator_joint.articulation_type == ArticulationType.PRISMATIC,
        details="Door, rack, detergent flap, wash arm, knob, and buttons need their specified joint types.",
    )

    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem="front_sill",
        negative_elem="outer_panel",
        min_gap=0.001,
        max_gap=0.006,
        name="closed heavy door sits just outside the front frame",
    )
    ctx.expect_within(
        rack,
        body,
        axes="yz",
        margin=0.010,
        name="rack stays inside the open wash chamber cross-section",
    )
    ctx.expect_overlap(
        rack,
        body,
        axes="x",
        elem_a="runner_0",
        elem_b="guide_0",
        min_overlap=0.45,
        name="collapsed rack runners are deeply engaged on guides",
    )
    ctx.expect_contact(
        body,
        object_model.get_part("wash_arm"),
        elem_a="arm_bearing",
        elem_b="center_hub",
        contact_tol=0.001,
        name="wash arm hub sits on the sump bearing",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: DOOR_OPEN}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "main door drops outward and down",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.25
        and opened_aabb[0][0] < closed_aabb[0][0] - 0.25,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_rack_pos = ctx.part_world_position(rack)
    with ctx.pose({door_joint: DOOR_OPEN, rack_joint: RACK_TRAVEL}):
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            elem_a="runner_0",
            elem_b="guide_0",
            min_overlap=0.10,
            name="extended rack remains captured by its guide",
        )
        extended_rack_pos = ctx.part_world_position(rack)
    ctx.check(
        "rack slides outward toward the open door",
        rest_rack_pos is not None
        and extended_rack_pos is not None
        and extended_rack_pos[0] < rest_rack_pos[0] - 0.35,
        details=f"rest={rest_rack_pos}, extended={extended_rack_pos}",
    )

    flap_rest = ctx.part_world_aabb(detergent_flap)
    with ctx.pose({flap_joint: DETERGENT_OPEN}):
        flap_open = ctx.part_world_aabb(detergent_flap)
    ctx.check(
        "detergent flap opens off the inner door",
        flap_rest is not None
        and flap_open is not None
        and flap_open[1][0] > flap_rest[1][0] + 0.05,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    start_rest = ctx.part_world_position(start_button)
    indicator_rest = ctx.part_world_position(indicator_button_0)
    with ctx.pose({start_joint: BUTTON_TRAVEL}):
        start_pressed = ctx.part_world_position(start_button)
        indicator_still = ctx.part_world_position(indicator_button_0)
    ctx.check(
        "start button presses independently inward",
        start_rest is not None
        and start_pressed is not None
        and indicator_rest is not None
        and indicator_still is not None
        and start_pressed[0] > start_rest[0] + 0.010
        and abs(indicator_still[0] - indicator_rest[0]) < 0.001,
        details=f"start={start_rest}->{start_pressed}, indicator={indicator_rest}->{indicator_still}",
    )

    return ctx.report()


object_model = build_object_model()
