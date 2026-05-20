from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


WHITE = Material("slightly_warm_white_enamel", rgba=(0.92, 0.93, 0.91, 1.0))
SHADOW = Material("deep_shadow_black", rgba=(0.01, 0.012, 0.014, 1.0))
DARK_RUBBER = Material("matte_black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
STEEL = Material("brushed_stainless_steel", rgba=(0.63, 0.65, 0.64, 1.0))
GLASS = Material("smoky_transparent_glass", rgba=(0.45, 0.67, 0.82, 0.38))
CONTROL_GREY = Material("satin_light_grey_plastic", rgba=(0.72, 0.74, 0.74, 1.0))
MID_GREY = Material("dark_grey_plastic", rgba=(0.20, 0.21, 0.22, 1.0))
BLUE = Material("pale_blue_drawer_insert", rgba=(0.65, 0.80, 0.90, 1.0))


def _box(part, name, size, xyz, material=WHITE):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_x(part, name, radius, length, xyz, material, rpy=(0.0, math.pi / 2.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl_y(part, name, radius, length, xyz, material, rpy=(math.pi / 2.0, 0.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl_z(part, name, radius, length, xyz, material):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def _tub_shell_mesh():
    outer = [(0.242, 0.000), (0.255, 0.035), (0.255, 0.360), (0.220, 0.430)]
    inner = [(0.202, 0.000), (0.214, 0.035), (0.214, 0.350), (0.178, 0.430)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=72, start_cap="flat", end_cap="flat"),
        "fixed_outer_tub_shell",
    )


def _drum_shell_mesh():
    outer = [(0.168, 0.000), (0.176, 0.018), (0.176, 0.260), (0.155, 0.292)]
    inner = [(0.145, 0.000), (0.153, 0.018), (0.153, 0.252), (0.130, 0.292)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=80, start_cap="flat", end_cap="flat"),
        "rotating_inner_drum_shell",
    )


def _add_drum_perforations(part):
    # Dark shallow plugs on the metal face and side read as punched perforations
    # while remaining tied into the rotating drum part.
    for ring_i, radius in enumerate((0.065, 0.105, 0.140)):
        count = 10 + ring_i * 6
        for i in range(count):
            a = 2.0 * math.pi * i / count + ring_i * 0.17
            y = radius * math.cos(a)
            z = radius * math.sin(a)
            _cyl_x(part, f"front_hole_{ring_i}_{i}", 0.0048, 0.003, (-0.004, y, z), SHADOW)
    for row, x in enumerate((0.045, 0.095, 0.145, 0.195)):
        for i in range(16):
            a = 2.0 * math.pi * (i + 0.5 * row) / 16
            y = 0.171 * math.cos(a)
            z = 0.171 * math.sin(a)
            part.visual(
                Cylinder(radius=0.004, length=0.0025),
                origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, a)),
                material=SHADOW,
                name=f"side_hole_{row}_{i}",
            )


def _add_screws(part, prefix, x, positions, axis="x"):
    for i, (y, z) in enumerate(positions):
        if axis == "x":
            _cyl_x(part, f"{prefix}_{i}", 0.006, 0.003, (x, y, z), STEEL)
        else:
            _cyl_z(part, f"{prefix}_{i}", 0.005, 0.003, (x, y, z), STEEL)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washing_machine")

    cabinet = model.part("cabinet")

    # One square enamel appliance carcass, built from contacting panels so the
    # front is genuinely open around the circular serviceable door area.
    _box(cabinet, "left_side_panel", (0.62, 0.020, 0.85), (0.0, -0.310, 0.425))
    _box(cabinet, "right_side_panel", (0.62, 0.020, 0.85), (0.0, 0.310, 0.425))
    _box(cabinet, "top_panel", (0.62, 0.62, 0.020), (0.0, 0.0, 0.850))
    _box(cabinet, "base_panel", (0.62, 0.62, 0.020), (0.0, 0.0, 0.010))
    _box(cabinet, "rear_panel", (0.020, 0.62, 0.85), (0.310, 0.0, 0.425))
    _box(cabinet, "front_left_stile", (0.022, 0.040, 0.85), (-0.310, -0.290, 0.425))
    _box(cabinet, "front_right_stile", (0.022, 0.040, 0.85), (-0.310, 0.290, 0.425))
    _box(cabinet, "front_top_rail", (0.022, 0.60, 0.050), (-0.310, 0.0, 0.835))
    _box(cabinet, "front_bottom_rail", (0.022, 0.60, 0.200), (-0.310, 0.0, 0.100))
    _cyl_z(cabinet, "rounded_front_edge_0", 0.012, 0.85, (-0.322, -0.302, 0.425), WHITE)
    _cyl_z(cabinet, "rounded_front_edge_1", 0.012, 0.85, (-0.322, 0.302, 0.425), WHITE)

    # Raised service fascia with a real drawer slot, button sockets, knob collar,
    # seams, screws, and supported slide rails.
    _box(cabinet, "fascia_left_post", (0.035, 0.020, 0.125), (-0.337, -0.285, 0.755), CONTROL_GREY)
    _box(cabinet, "fascia_slot_divider", (0.035, 0.018, 0.125), (-0.337, -0.050, 0.755), CONTROL_GREY)
    _box(cabinet, "fascia_slot_top", (0.035, 0.235, 0.022), (-0.337, -0.167, 0.805), CONTROL_GREY)
    _box(cabinet, "fascia_slot_bottom", (0.035, 0.235, 0.022), (-0.337, -0.167, 0.705), CONTROL_GREY)
    _box(cabinet, "fascia_control_plate", (0.035, 0.330, 0.125), (-0.337, 0.145, 0.755), CONTROL_GREY)
    _box(cabinet, "drawer_rail_0", (0.220, 0.010, 0.012), (-0.230, -0.250, 0.755), STEEL)
    _box(cabinet, "drawer_rail_1", (0.220, 0.010, 0.012), (-0.230, -0.084, 0.755), STEEL)
    _box(cabinet, "knob_recess_plate", (0.008, 0.118, 0.118), (-0.360, 0.135, 0.765), MID_GREY)
    _cyl_x(cabinet, "knob_collar", 0.058, 0.010, (-0.366, 0.135, 0.765), MID_GREY)
    for i, y in enumerate((0.015, 0.065, 0.215, 0.265)):
        _box(cabinet, f"button_socket_{i}", (0.010, 0.048, 0.030), (-0.363, y, 0.810), MID_GREY)
    _box(cabinet, "start_button_socket", (0.010, 0.070, 0.022), (-0.363, 0.240, 0.715), MID_GREY)

    # Door opening trim, latch receiver, hinge support brackets, lower toe-kick,
    # rear pads, and small fasteners.
    cabinet.visual(
        mesh_from_geometry(TorusGeometry(radius=0.255, tube=0.010, radial_segments=18, tubular_segments=96), "front_opening_bead"),
        origin=Origin(xyz=(-0.334, 0.0, 0.450), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE,
        name="front_opening_bead",
    )
    _box(cabinet, "latch_receiver", (0.030, 0.030, 0.060), (-0.365, 0.265, 0.450), STEEL)
    _box(cabinet, "latch_socket_shadow", (0.006, 0.018, 0.036), (-0.382, 0.266, 0.450), SHADOW)
    _box(cabinet, "hinge_bracket_upper", (0.030, 0.060, 0.038), (-0.377, -0.287, 0.565), STEEL)
    _box(cabinet, "hinge_bracket_lower", (0.030, 0.060, 0.038), (-0.377, -0.287, 0.335), STEEL)
    _box(cabinet, "toe_kick_recess", (0.018, 0.460, 0.052), (-0.334, 0.0, 0.043), SHADOW)
    _box(cabinet, "toe_kick_lip", (0.030, 0.500, 0.018), (-0.345, 0.0, 0.080), WHITE)
    _box(cabinet, "gasket_retainer_top", (0.030, 0.230, 0.024), (-0.345, 0.0, 0.695), DARK_RUBBER)
    _box(cabinet, "gasket_retainer_bottom", (0.030, 0.230, 0.024), (-0.345, 0.0, 0.205), DARK_RUBBER)
    _box(cabinet, "gasket_retainer_side_0", (0.030, 0.024, 0.180), (-0.345, -0.238, 0.450), DARK_RUBBER)
    _box(cabinet, "gasket_retainer_side_1", (0.030, 0.024, 0.180), (-0.345, 0.238, 0.450), DARK_RUBBER)
    _box(cabinet, "tub_mount_0", (0.110, 0.090, 0.110), (-0.055, -0.275, 0.450), MID_GREY)
    _box(cabinet, "tub_mount_1", (0.110, 0.090, 0.110), (-0.055, 0.275, 0.450), MID_GREY)
    _cyl_x(cabinet, "drain_cap_socket", 0.038, 0.048, (-0.300, 0.170, 0.155), MID_GREY)
    for y in (-0.235, 0.235):
        _cyl_z(cabinet, f"rear_leveling_pad_{0 if y < 0 else 1}", 0.035, 0.018, (0.250, y, -0.004), MID_GREY)
    _add_screws(cabinet, "fascia_screw", -0.370, [(-0.285, 0.812), (0.290, 0.812), (-0.285, 0.700), (0.290, 0.700)])
    _add_screws(cabinet, "hinge_screw", -0.372, [(-0.292, 0.588), (-0.292, 0.542), (-0.292, 0.358), (-0.292, 0.312)])

    tub = model.part("outer_tub")
    tub.visual(_tub_shell_mesh(), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=MID_GREY, name="tub_shell")
    _cyl_x(tub, "rear_axle_boss", 0.045, 0.045, (0.438, 0.0, 0.0), STEEL)
    _cyl_x(tub, "dark_tub_depth", 0.198, 0.004, (0.010, 0.0, 0.0), SHADOW)
    model.articulation("cabinet_to_outer_tub", ArticulationType.FIXED, cabinet, tub, origin=Origin(xyz=(-0.270, 0.0, 0.450)))

    gasket = model.part("gasket")
    gasket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.218, tube=0.024, radial_segments=20, tubular_segments=96), "thick_rubber_gasket"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK_RUBBER,
        name="rubber_gasket_ring",
    )
    gasket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.178, tube=0.010, radial_segments=14, tubular_segments=96), "inner_gasket_lip"),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK_RUBBER,
        name="inner_flexible_lip",
    )
    model.articulation("cabinet_to_gasket", ArticulationType.FIXED, cabinet, gasket, origin=Origin(xyz=(-0.360, 0.0, 0.450)))

    drum = model.part("drum")
    drum.visual(_drum_shell_mesh(), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=STEEL, name="perforated_drum_shell")
    _cyl_x(drum, "drum_spider_hub", 0.030, 0.040, (0.305, 0.0, 0.0), STEEL)
    _cyl_x(drum, "drum_axle_shaft", 0.014, 0.150, (0.365, 0.0, 0.0), STEEL)
    _add_drum_perforations(drum)
    model.articulation(
        "outer_tub_to_drum",
        ArticulationType.CONTINUOUS,
        tub,
        drum,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    door_frame = model.part("door_frame")
    door_frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.205, tube=0.027, radial_segments=24, tubular_segments=112), "metal_door_frame_ring"),
        origin=Origin(xyz=(0.0, 0.255, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="metal_frame_ring",
    )
    door_frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.175, tube=0.006, radial_segments=12, tubular_segments=96), "glass_retaining_bead"),
        origin=Origin(xyz=(-0.018, 0.255, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="glass_retaining_bead",
    )
    _cyl_z(door_frame, "hinge_barrel_upper", 0.016, 0.082, (0.0, 0.0, 0.115), STEEL)
    _cyl_z(door_frame, "hinge_barrel_lower", 0.016, 0.082, (0.0, 0.0, -0.115), STEEL)
    _cyl_z(door_frame, "hinge_pin_upper", 0.006, 0.094, (0.0, 0.0, 0.115), MID_GREY)
    _cyl_z(door_frame, "hinge_pin_lower", 0.006, 0.094, (0.0, 0.0, -0.115), MID_GREY)
    _box(door_frame, "latch_tongue", (0.030, 0.030, 0.032), (-0.006, 0.530, 0.000), STEEL)
    _add_screws(door_frame, "door_screw", -0.033, [(0.108, 0.148), (0.402, 0.148), (0.108, -0.148), (0.402, -0.148)])
    model.articulation(
        "cabinet_to_door_frame",
        ArticulationType.REVOLUTE,
        cabinet,
        door_frame,
        origin=Origin(xyz=(-0.385, -0.255, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    glass = model.part("glass")
    _cyl_x(glass, "convex_glass_window", 0.158, 0.014, (0.0, 0.0, 0.0), GLASS)
    _cyl_x(glass, "smoked_glass_edge", 0.172, 0.006, (0.0, 0.0, 0.0), GLASS)
    model.articulation("door_frame_to_glass", ArticulationType.FIXED, door_frame, glass, origin=Origin(xyz=(-0.020, 0.255, 0.0)))

    drawer = model.part("detergent_drawer")
    _box(drawer, "drawer_front", (0.024, 0.212, 0.074), (-0.012, 0.0, 0.0), CONTROL_GREY)
    _box(drawer, "drawer_tray_floor", (0.230, 0.206, 0.008), (0.115, 0.0, -0.033), BLUE)
    _box(drawer, "drawer_side_0", (0.230, 0.008, 0.058), (0.115, -0.103, -0.004), BLUE)
    _box(drawer, "drawer_side_1", (0.230, 0.008, 0.058), (0.115, 0.103, -0.004), BLUE)
    _box(drawer, "drawer_rear_wall", (0.010, 0.206, 0.058), (0.230, 0.0, -0.004), BLUE)
    _box(drawer, "compartment_divider_0", (0.210, 0.006, 0.048), (0.113, -0.034, 0.000), BLUE)
    _box(drawer, "compartment_divider_1", (0.210, 0.006, 0.048), (0.113, 0.035, 0.000), BLUE)
    _box(drawer, "rail_runner_0", (0.225, 0.012, 0.012), (0.108, -0.083, -0.002), MID_GREY)
    _box(drawer, "rail_runner_1", (0.225, 0.012, 0.012), (0.108, 0.083, -0.002), MID_GREY)
    model.articulation(
        "cabinet_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        cabinet,
        drawer,
        origin=Origin(xyz=(-0.360, -0.167, 0.755)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.175),
    )

    knob = model.part("cycle_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.080,
                0.035,
                body_style="skirted",
                top_diameter=0.060,
                grip=KnobGrip(style="fluted", count=24, depth=0.002),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            ),
            "cycle_selector_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=MID_GREY,
        name="fluted_cycle_knob",
    )
    model.articulation(
        "cabinet_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        cabinet,
        knob,
        origin=Origin(xyz=(-0.386, 0.135, 0.765)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    for i, y in enumerate((0.015, 0.065, 0.215, 0.265)):
        button = model.part(f"button_{i}")
        _box(button, "button_cap", (0.018, 0.038, 0.023), (-0.009, 0.0, 0.0), CONTROL_GREY)
        _box(button, "button_stem", (0.014, 0.020, 0.015), (0.008, 0.0, 0.0), MID_GREY)
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            cabinet,
            button,
            origin=Origin(xyz=(-0.374, y, 0.810)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=0.012),
        )

    start = model.part("start_button")
    _box(start, "slim_start_cap", (0.016, 0.060, 0.016), (-0.008, 0.0, 0.0), Material("green_start_button", rgba=(0.20, 0.55, 0.30, 1.0)))
    _box(start, "start_stem", (0.014, 0.030, 0.012), (0.007, 0.0, 0.0), MID_GREY)
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        cabinet,
        start,
        origin=Origin(xyz=(-0.374, 0.240, 0.715)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.18, lower=0.0, upper=0.010),
    )

    hatch = model.part("service_hatch")
    _box(hatch, "hatch_panel", (0.018, 0.160, 0.082), (0.0, 0.0, 0.041), WHITE)
    _cyl_y(hatch, "bottom_hinge_barrel", 0.008, 0.164, (0.000, 0.0, 0.000), STEEL)
    _box(hatch, "hatch_pull_notch", (0.006, 0.070, 0.012), (-0.012, 0.0, 0.075), SHADOW)
    model.articulation(
        "cabinet_to_service_hatch",
        ArticulationType.REVOLUTE,
        cabinet,
        hatch,
        origin=Origin(xyz=(-0.330, 0.170, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    drain = model.part("drain_cap")
    _cyl_x(drain, "round_filter_cap", 0.032, 0.014, (0.0, 0.0, 0.0), MID_GREY)
    _box(drain, "cap_grip_slot", (0.004, 0.050, 0.008), (-0.010, 0.0, 0.0), SHADOW)
    model.articulation(
        "cabinet_to_drain_cap",
        ArticulationType.CONTINUOUS,
        cabinet,
        drain,
        origin=Origin(xyz=(-0.284, 0.170, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("door_frame")
    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("detergent_drawer")
    drum = object_model.get_part("drum")
    tub = object_model.get_part("outer_tub")

    door_hinge = object_model.get_articulation("cabinet_to_door_frame")
    drawer_slide = object_model.get_articulation("cabinet_to_detergent_drawer")
    hatch_hinge = object_model.get_articulation("cabinet_to_service_hatch")

    def aabb_center(part):
        box = ctx.part_world_aabb(part)
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(
            button,
            cabinet,
            elem_a="button_stem",
            elem_b=f"button_socket_{i}",
            reason="The push-button stem is intentionally captured inside its recessed socket.",
        )
        ctx.expect_within(button, cabinet, axes="yz", elem_a="button_stem", elem_b=f"button_socket_{i}", margin=0.001, name=f"button {i} stem sits in its socket")
        ctx.expect_overlap(button, cabinet, axes="x", elem_a="button_stem", elem_b=f"button_socket_{i}", min_overlap=0.004, name=f"button {i} has retained socket insertion")

    ctx.allow_overlap(
        object_model.get_part("start_button"),
        cabinet,
        elem_a="start_stem",
        elem_b="start_button_socket",
        reason="The start-button stem is intentionally nested in its fascia socket.",
    )
    ctx.expect_within(object_model.get_part("start_button"), cabinet, axes="yz", elem_a="start_stem", elem_b="start_button_socket", margin=0.001, name="start stem sits in its socket")
    ctx.expect_overlap(object_model.get_part("start_button"), cabinet, axes="x", elem_a="start_stem", elem_b="start_button_socket", min_overlap=0.004, name="start button has retained socket insertion")

    for suffix in ("upper", "lower"):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"hinge_bracket_{suffix}",
            elem_b=f"hinge_barrel_{suffix}",
            reason="The visible hinge barrel is captured by the cabinet-mounted hinge bracket around the pin.",
        )
        ctx.expect_overlap(cabinet, door, axes="z", elem_a=f"hinge_bracket_{suffix}", elem_b=f"hinge_barrel_{suffix}", min_overlap=0.025, name=f"{suffix} hinge bracket captures barrel")

    for elem in ("gasket_retainer_top", "gasket_retainer_bottom", "gasket_retainer_side_0", "gasket_retainer_side_1"):
        ctx.allow_overlap(
            cabinet,
            object_model.get_part("gasket"),
            elem_a=elem,
            elem_b="rubber_gasket_ring",
            reason="The rubber gasket is locally compressed into the cabinet retainer lip.",
        )
    ctx.allow_overlap(
        door,
        object_model.get_part("gasket"),
        elem_a="metal_frame_ring",
        elem_b="rubber_gasket_ring",
        reason="The closed door frame intentionally compresses the soft rubber gasket.",
    )
    ctx.allow_overlap(
        door,
        object_model.get_part("gasket"),
        elem_a="metal_frame_ring",
        elem_b="inner_flexible_lip",
        reason="The door frame also lightly compresses the inner gasket lip in the closed pose.",
    )
    ctx.expect_overlap(door, object_model.get_part("gasket"), axes="yz", elem_a="metal_frame_ring", elem_b="rubber_gasket_ring", min_overlap=0.30, name="closed door bears on the rubber gasket")
    ctx.expect_overlap(door, object_model.get_part("gasket"), axes="yz", elem_a="metal_frame_ring", elem_b="inner_flexible_lip", min_overlap=0.24, name="closed door engages inner gasket lip")

    for runner, rail in (("rail_runner_0", "drawer_rail_0"), ("rail_runner_1", "drawer_rail_1")):
        ctx.allow_overlap(
            cabinet,
            drawer,
            elem_a=rail,
            elem_b=runner,
            reason="Drawer runner is intentionally nested on the fixed metal slide rail.",
        )
        ctx.expect_overlap(drawer, cabinet, axes="x", elem_a=runner, elem_b=rail, min_overlap=0.15, name=f"{runner} rides on fixed rail")
    ctx.allow_overlap(
        cabinet,
        drawer,
        elem_a="drawer_rail_0",
        elem_b="drawer_rear_wall",
        reason="The rear wall of the drawer tray locally wraps around the retained rail end.",
    )
    ctx.expect_overlap(drawer, cabinet, axes="yz", elem_a="drawer_rear_wall", elem_b="drawer_rail_0", min_overlap=0.010, name="drawer rear wall wraps rail end")
    ctx.allow_overlap(
        cabinet,
        drawer,
        elem_a="drawer_rail_1",
        elem_b="drawer_rear_wall",
        reason="The drawer rear wall also wraps the opposite retained rail end.",
    )
    ctx.expect_overlap(drawer, cabinet, axes="yz", elem_a="drawer_rear_wall", elem_b="drawer_rail_1", min_overlap=0.010, name="drawer rear wall wraps second rail end")
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="latch_socket_shadow",
        elem_b="latch_tongue",
        reason="The door latch tongue is intentionally inserted into the cabinet latch receiver.",
    )
    ctx.expect_overlap(door, cabinet, axes="yz", elem_a="latch_tongue", elem_b="latch_socket_shadow", min_overlap=0.010, name="door latch tongue enters receiver")

    for elem in ("tub_mount_0", "tub_mount_1"):
        ctx.allow_overlap(
            cabinet,
            tub,
            elem_a=elem,
            elem_b="tub_shell",
            reason="The hidden tub cradle pad locally seats against the outer tub shell.",
        )
    ctx.allow_overlap(
        tub,
        drum,
        elem_a="rear_axle_boss",
        elem_b="drum_axle_shaft",
        reason="The rotating drum shaft is intentionally captured in the fixed rear bearing boss.",
    )
    ctx.expect_overlap(tub, drum, axes="x", elem_a="rear_axle_boss", elem_b="drum_axle_shaft", min_overlap=0.015, name="drum shaft is retained by rear bearing boss")

    ctx.allow_overlap(
        door,
        object_model.get_part("glass"),
        elem_a="glass_retaining_bead",
        elem_b="smoked_glass_edge",
        reason="The glass edge is seated inside the metal retaining bead.",
    )
    ctx.expect_overlap(door, object_model.get_part("glass"), axes="yz", elem_a="glass_retaining_bead", elem_b="smoked_glass_edge", min_overlap=0.32, name="glass is retained by door bead")

    ctx.allow_overlap(
        cabinet,
        object_model.get_part("cycle_knob"),
        elem_a="knob_collar",
        elem_b="fluted_cycle_knob",
        reason="The selector knob is seated onto the fixed fascia collar.",
    )
    ctx.allow_overlap(
        cabinet,
        object_model.get_part("drain_cap"),
        elem_a="drain_cap_socket",
        elem_b="round_filter_cap",
        reason="The drain-filter cap threads into the recessed service socket.",
    )

    ctx.expect_contact(cabinet, object_model.get_part("gasket"), elem_a="front_opening_bead", elem_b="rubber_gasket_ring", contact_tol=0.035, name="rubber gasket is seated in the front opening")
    ctx.expect_within(drum, tub, axes="yz", margin=0.020, elem_a="perforated_drum_shell", elem_b="tub_shell", name="rotating drum nests radially inside fixed tub")
    ctx.expect_overlap(drum, tub, axes="x", min_overlap=0.20, elem_a="perforated_drum_shell", elem_b="tub_shell", name="drum has real depth inside the tub")
    ctx.expect_overlap(drawer, cabinet, axes="x", min_overlap=0.040, elem_a="rail_runner_0", elem_b="drawer_rail_0", name="drawer runner remains captured on rail")

    closed_door_pos = aabb_center(door)
    closed_drawer_pos = ctx.part_world_position(drawer)
    closed_hatch_pos = aabb_center(object_model.get_part("service_hatch"))
    with ctx.pose({door_hinge: 1.1, drawer_slide: 0.14, hatch_hinge: 0.9}):
        opened_door_pos = aabb_center(door)
        extended_drawer_pos = ctx.part_world_position(drawer)
        opened_hatch_pos = aabb_center(object_model.get_part("service_hatch"))
        ctx.expect_overlap(drawer, cabinet, axes="x", min_overlap=0.040, elem_a="rail_runner_0", elem_b="drawer_rail_0", name="extended drawer still retains rail engagement")

    ctx.check(
        "front door swings outward from left hinge",
        closed_door_pos is not None and opened_door_pos is not None and opened_door_pos[0] < closed_door_pos[0] - 0.12,
        details=f"closed={closed_door_pos}, opened={opened_door_pos}",
    )
    ctx.check(
        "detergent drawer slides outward",
        closed_drawer_pos is not None and extended_drawer_pos is not None and extended_drawer_pos[0] < closed_drawer_pos[0] - 0.10,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )
    ctx.check(
        "service hatch folds downward",
        closed_hatch_pos is not None and opened_hatch_pos is not None and opened_hatch_pos[2] < closed_hatch_pos[2] - 0.01,
        details=f"closed={closed_hatch_pos}, opened={opened_hatch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
