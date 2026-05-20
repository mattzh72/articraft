from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_control_range")

    stainless = model.material("brushed_stainless", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_stainless = model.material("dark_brushed_steel", rgba=(0.16, 0.17, 0.17, 1.0))
    enamel_black = model.material("black_enamel", rgba=(0.005, 0.005, 0.006, 1.0))
    glass_black = model.material("smoked_oven_glass", rgba=(0.02, 0.025, 0.03, 0.72))
    cast_iron = model.material("cast_iron_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    white_mark = model.material("white_silk_screen", rgba=(0.92, 0.91, 0.86, 1.0))
    amber = model.material("amber_display", rgba=(1.0, 0.46, 0.08, 1.0))

    width = 0.76
    depth = 0.66
    front_y = -0.35
    back_y = 0.31

    body = model.part("body")

    # Structural cabinet shell: side walls, rear panel, bottom pan, and top deck
    # form a real, hollow oven opening rather than a solid block.
    body.visual(Box((0.036, depth, 0.82)), origin=Origin(xyz=(-0.362, -0.020, 0.460)), material=stainless, name="side_panel_0")
    body.visual(Box((0.036, depth, 0.82)), origin=Origin(xyz=(0.362, -0.020, 0.460)), material=stainless, name="side_panel_1")
    body.visual(Box((width, 0.036, 0.82)), origin=Origin(xyz=(0.000, back_y - 0.018, 0.460)), material=stainless, name="rear_panel")
    body.visual(Box((width, depth, 0.065)), origin=Origin(xyz=(0.000, -0.020, 0.033)), material=dark_stainless, name="bottom_pan")
    body.visual(Box((width, depth, 0.058)), origin=Origin(xyz=(0.000, -0.020, 0.851)), material=stainless, name="top_deck")

    # Front oven-frame rails and toe kick, all tied back into the cabinet.
    body.visual(Box((0.060, 0.055, 0.555)), origin=Origin(xyz=(-0.350, front_y - 0.016, 0.395)), material=stainless, name="front_stile_0")
    body.visual(Box((0.060, 0.055, 0.555)), origin=Origin(xyz=(0.350, front_y - 0.016, 0.395)), material=stainless, name="front_stile_1")
    body.visual(Box((width, 0.055, 0.052)), origin=Origin(xyz=(0.000, front_y - 0.016, 0.654)), material=stainless, name="upper_oven_rail")
    body.visual(Box((width, 0.055, 0.064)), origin=Origin(xyz=(0.000, front_y - 0.016, 0.129)), material=dark_stainless, name="lower_oven_rail")
    body.visual(Box((width, 0.060, 0.095)), origin=Origin(xyz=(0.000, front_y - 0.020, 0.073)), material=dark_stainless, name="toe_kick")

    # Slightly sloped-looking front control band represented as a substantial
    # vertical stainless fascia with raised trim.
    body.visual(Box((width, 0.042, 0.178)), origin=Origin(xyz=(0.000, front_y - 0.020, 0.745)), material=stainless, name="control_fascia")
    body.visual(Box((0.735, 0.010, 0.020)), origin=Origin(xyz=(0.000, front_y - 0.045, 0.838)), material=dark_stainless, name="upper_trim")
    body.visual(Box((0.735, 0.010, 0.018)), origin=Origin(xyz=(0.000, front_y - 0.045, 0.660)), material=dark_stainless, name="lower_trim")

    # Dark oven liner and rack rails seen when the door opens.
    body.visual(Box((0.700, 0.040, 0.430)), origin=Origin(xyz=(0.000, 0.257, 0.375)), material=enamel_black, name="oven_back_liner")
    body.visual(Box((0.026, 0.520, 0.430)), origin=Origin(xyz=(-0.335, -0.040, 0.375)), material=enamel_black, name="oven_liner_0")
    body.visual(Box((0.026, 0.520, 0.430)), origin=Origin(xyz=(0.335, -0.040, 0.375)), material=enamel_black, name="oven_liner_1")
    body.visual(Box((0.700, 0.520, 0.018)), origin=Origin(xyz=(0.000, -0.040, 0.165)), material=enamel_black, name="oven_floor")
    body.visual(Box((0.700, 0.520, 0.018)), origin=Origin(xyz=(0.000, -0.040, 0.585)), material=enamel_black, name="oven_ceiling")
    for z in (0.295, 0.445):
        body.visual(Box((0.012, 0.470, 0.012)), origin=Origin(xyz=(-0.326, -0.045, z)), material=dark_stainless, name=f"rack_rail_0_{z:.2f}")
        body.visual(Box((0.012, 0.470, 0.012)), origin=Origin(xyz=(0.326, -0.045, z)), material=dark_stainless, name=f"rack_rail_1_{z:.2f}")
        body.visual(Box((0.660, 0.006, 0.006)), origin=Origin(xyz=(0.000, -0.255, z + 0.010)), material=dark_stainless, name=f"rack_front_frame_{z:.2f}")
        body.visual(Box((0.660, 0.006, 0.006)), origin=Origin(xyz=(0.000, 0.165, z + 0.010)), material=dark_stainless, name=f"rack_rear_frame_{z:.2f}")
        for x in (-0.190, -0.095, 0.000, 0.095, 0.190):
            body.visual(Box((0.006, 0.420, 0.006)), origin=Origin(xyz=(x, -0.045, z + 0.010)), material=dark_stainless, name=f"rack_wire_{x:.2f}_{z:.2f}")

    # Black glass/enamel cooktop with four burners, trim, and raised grates.
    body.visual(Box((0.720, 0.600, 0.020)), origin=Origin(xyz=(0.000, -0.020, 0.888)), material=enamel_black, name="cooktop_glass")
    body.visual(Box((0.735, 0.018, 0.026)), origin=Origin(xyz=(0.000, front_y + 0.028, 0.896)), material=dark_stainless, name="front_cooktop_lip")
    body.visual(Box((0.735, 0.018, 0.026)), origin=Origin(xyz=(0.000, back_y - 0.016, 0.896)), material=dark_stainless, name="rear_cooktop_lip")
    body.visual(Box((0.018, 0.600, 0.026)), origin=Origin(xyz=(-0.368, -0.020, 0.896)), material=dark_stainless, name="side_cooktop_lip_0")
    body.visual(Box((0.018, 0.600, 0.026)), origin=Origin(xyz=(0.368, -0.020, 0.896)), material=dark_stainless, name="side_cooktop_lip_1")

    ring_mesh = mesh_from_geometry(TorusGeometry(0.080, 0.006, radial_segments=14, tubular_segments=48), "burner_trim_ring")
    small_ring_mesh = mesh_from_geometry(TorusGeometry(0.060, 0.005, radial_segments=12, tubular_segments=44), "small_burner_trim_ring")
    burner_positions = (
        (-0.215, -0.175, 0.086, ring_mesh),
        (0.215, -0.175, 0.074, small_ring_mesh),
        (-0.215, 0.125, 0.066, small_ring_mesh),
        (0.215, 0.125, 0.090, ring_mesh),
    )
    for idx, (x, y, cap_radius, trim_mesh) in enumerate(burner_positions):
        body.visual(trim_mesh, origin=Origin(xyz=(x, y, 0.904)), material=dark_stainless, name=f"burner_trim_{idx}")
        body.visual(Cylinder(radius=cap_radius, length=0.012), origin=Origin(xyz=(x, y, 0.904)), material=rubber, name=f"burner_cap_{idx}")
        body.visual(Cylinder(radius=cap_radius * 0.48, length=0.015), origin=Origin(xyz=(x, y, 0.912)), material=cast_iron, name=f"burner_center_{idx}")
        # Four-piece grate per burner, each bar slightly embedded into the
        # burner cap and cooktop so it reads as a supported cast-iron member.
        body.visual(Box((0.205, 0.018, 0.020)), origin=Origin(xyz=(x, y, 0.920)), material=cast_iron, name=f"grate_cross_x_{idx}")
        body.visual(Box((0.018, 0.205, 0.020)), origin=Origin(xyz=(x, y, 0.920)), material=cast_iron, name=f"grate_cross_y_{idx}")
        body.visual(Box((0.110, 0.014, 0.018)), origin=Origin(xyz=(x, y + 0.080, 0.920)), material=cast_iron, name=f"grate_rear_{idx}")
        body.visual(Box((0.110, 0.014, 0.018)), origin=Origin(xyz=(x, y - 0.080, 0.920)), material=cast_iron, name=f"grate_front_{idx}")

    # Low rear backsplash/backguard connects to the cooktop and carries no
    # functional controls, keeping the prompt's controls front-facing.
    body.visual(Box((width, 0.045, 0.210)), origin=Origin(xyz=(0.000, back_y + 0.006, 1.000)), material=stainless, name="backguard_panel")
    body.visual(Box((0.720, 0.012, 0.020)), origin=Origin(xyz=(0.000, back_y - 0.019, 0.887)), material=dark_stainless, name="backguard_foot")

    # Control display and silkscreen markings on the front fascia.
    display_bezel = mesh_from_geometry(
        BezelGeometry(
            (0.150, 0.043),
            (0.190, 0.072),
            0.010,
            opening_corner_radius=0.006,
            outer_corner_radius=0.010,
            face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.0015),
            center=False,
        ),
        "display_bezel",
    )
    body.visual(display_bezel, origin=Origin(xyz=(0.000, front_y - 0.043, 0.798), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_stainless, name="display_bezel")
    body.visual(Box((0.145, 0.007, 0.038)), origin=Origin(xyz=(0.000, front_y - 0.044, 0.798)), material=glass_black, name="display_lens")
    for x in (-0.042, -0.018, 0.006, 0.030):
        body.visual(Box((0.012, 0.0040, 0.018)), origin=Origin(xyz=(x, front_y - 0.046, 0.798)), material=amber, name=f"display_segment_{x:.2f}")

    # White tick marks around each rotary knob.
    knob_xs = (-0.300, -0.180, -0.060, 0.180, 0.300)
    knob_z = 0.705
    for idx, x in enumerate(knob_xs):
        for angle, sx, sz in ((0.0, 0.006, 0.018), (-36.0, 0.006, 0.012), (36.0, 0.006, 0.012)):
            radians = math.radians(angle)
            mark_x = x + 0.035 * math.sin(radians)
            mark_z = knob_z + 0.035 * math.cos(radians)
            body.visual(Box((sx, 0.004, sz)), origin=Origin(xyz=(mark_x, front_y - 0.0405, mark_z), rpy=(0.0, 0.0, radians)), material=white_mark, name=f"knob_tick_{idx}_{angle:.0f}")

    # Fixed hinge receivers protrude from the lower frame and physically meet
    # the door's moving knuckles at the joint line.
    for x in (-0.245, 0.245):
        body.visual(Box((0.082, 0.014, 0.044)), origin=Origin(xyz=(x, front_y - 0.047, 0.143)), material=dark_stainless, name=f"body_hinge_receiver_{x:.2f}")

    # Oven door as a bottom-hinged moving part. Its local frame is the hinge
    # line; the panel rises in local +Z and sits proud in local -Y when closed.
    door = model.part("oven_door")
    door.visual(Box((0.655, 0.046, 0.500)), origin=Origin(xyz=(0.000, -0.035, 0.270)), material=stainless, name="door_slab")
    window_bezel = mesh_from_geometry(
        BezelGeometry(
            (0.390, 0.235),
            (0.480, 0.310),
            0.012,
            opening_corner_radius=0.020,
            outer_corner_radius=0.030,
            face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.002),
            center=False,
        ),
        "oven_window_bezel",
    )
    door.visual(window_bezel, origin=Origin(xyz=(0.000, -0.060, 0.330), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_stainless, name="window_bezel")
    door.visual(Box((0.384, 0.006, 0.228)), origin=Origin(xyz=(0.000, -0.066, 0.330)), material=glass_black, name="window_glass")
    door.visual(Box((0.590, 0.012, 0.024)), origin=Origin(xyz=(0.000, -0.062, 0.505)), material=dark_stainless, name="upper_door_trim")
    door.visual(Box((0.590, 0.012, 0.022)), origin=Origin(xyz=(0.000, -0.062, 0.047)), material=dark_stainless, name="lower_door_trim")
    door.visual(Cylinder(radius=0.018, length=0.545), origin=Origin(xyz=(0.000, -0.087, 0.474), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_stainless, name="door_handle_bar")
    for x in (-0.245, 0.245):
        door.visual(Box((0.046, 0.052, 0.050)), origin=Origin(xyz=(x, -0.063, 0.474)), material=dark_stainless, name=f"handle_standoff_{x:.2f}")
        door.visual(Cylinder(radius=0.014, length=0.080), origin=Origin(xyz=(x, -0.024, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_stainless, name=f"door_hinge_knuckle_{x:.2f}")

    hinge = model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.000, front_y - 0.044, 0.123)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    # Five distinct front control knobs. The cap geometry extends along local
    # +Z, which the joint frame turns to world -Y so each knob protrudes from
    # the fascia and rotates about the face normal.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.027,
            body_style="skirted",
            top_diameter=0.037,
            skirt=KnobSkirt(0.062, 0.006, flare=0.07, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "front_range_knob",
    )
    for idx, x in enumerate(knob_xs):
        knob = model.part(f"knob_{idx}")
        knob.visual(knob_mesh, origin=Origin(), material=dark_stainless, name="knob_cap")
        model.articulation(
            f"body_to_knob_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, front_y - 0.041, knob_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=5.0, lower=-2.35, upper=2.35),
        )

    # Three low-profile push buttons under the clock display depress inward.
    for idx, x in enumerate((0.135, 0.190, 0.245)):
        button = model.part(f"button_{idx}")
        button.visual(Box((0.044, 0.018, 0.010)), origin=Origin(xyz=(0.000, 0.000, 0.005)), material=rubber, name="button_cap")
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, front_y - 0.041, 0.797), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    # Keep references from being optimized away by linters while making the
    # intended primary mechanism explicit for readers.
    hinge.meta["role"] = "bottom hinged oven door"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    door_hinge = object_model.get_articulation("body_to_oven_door")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_stile_0",
        negative_elem="door_slab",
        min_gap=None,
        max_gap=0.016,
        max_penetration=0.0,
        name="closed door sits just proud of frame",
    )
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.45, name="closed door covers the oven opening")
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "oven door folds outward and downward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.18
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    for idx in range(5):
        knob = object_model.get_part(f"knob_{idx}")
        joint = object_model.get_articulation(f"body_to_knob_{idx}")
        ctx.expect_contact(knob, body, elem_a="knob_cap", elem_b="control_fascia", contact_tol=0.003, name=f"knob {idx} seats on fascia")
        ctx.check(
            f"knob {idx} has range-like rotary travel",
            joint.motion_limits is not None and joint.motion_limits.lower <= -2.0 and joint.motion_limits.upper >= 2.0,
            details=str(joint.motion_limits),
        )

    button = object_model.get_part("button_0")
    button_joint = object_model.get_articulation("body_to_button_0")
    rest_pos = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.004}):
        pressed_pos = ctx.part_world_position(button)
    ctx.check(
        "push button depresses inward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.002,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
