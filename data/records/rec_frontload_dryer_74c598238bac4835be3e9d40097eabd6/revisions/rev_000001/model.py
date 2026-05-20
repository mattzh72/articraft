from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vented_tumble_dryer")

    enamel = model.material("warm_white_enamel", rgba=(0.96, 0.96, 0.91, 1.0))
    trim_white = model.material("slightly_glossy_white_trim", rgba=(1.0, 1.0, 0.97, 1.0))
    dark_gap = model.material("deep_shadow_cavity", rgba=(0.015, 0.016, 0.018, 1.0))
    brushed_metal = model.material("brushed_galvanized_drum", rgba=(0.68, 0.69, 0.66, 1.0))
    hinge_metal = model.material("dull_hinge_steel", rgba=(0.54, 0.55, 0.54, 1.0))
    smoked_glass = model.material("smoked_blue_glass", rgba=(0.20, 0.31, 0.40, 0.45))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    soft_grey = model.material("soft_grey_buttons", rgba=(0.48, 0.50, 0.51, 1.0))

    cabinet = model.part("cabinet")

    # The dryer cabinet is built as connected panels instead of a solid block so
    # the visible drum sits in a real front opening rather than intersecting a
    # proxy body.
    cabinet.visual(
        Box((0.035, 0.660, 0.900)),
        origin=Origin(xyz=(-0.3825, 0.0, 0.450)),
        material=enamel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((0.035, 0.660, 0.900)),
        origin=Origin(xyz=(0.3825, 0.0, 0.450)),
        material=enamel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((0.800, 0.660, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.8825)),
        material=enamel,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.800, 0.660, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=enamel,
        name="bottom_plinth",
    )
    cabinet.visual(
        Box((0.800, 0.035, 0.900)),
        origin=Origin(xyz=(0.0, 0.3125, 0.450)),
        material=enamel,
        name="rear_panel",
    )

    # Front face pieces leave a large service/porthole aperture.
    cabinet.visual(
        Box((0.135, 0.035, 0.900)),
        origin=Origin(xyz=(-0.3325, -0.3475, 0.450)),
        material=enamel,
        name="front_side_0",
    )
    cabinet.visual(
        Box((0.135, 0.035, 0.900)),
        origin=Origin(xyz=(0.3325, -0.3475, 0.450)),
        material=enamel,
        name="front_side_1",
    )
    cabinet.visual(
        Box((0.800, 0.035, 0.170)),
        origin=Origin(xyz=(0.0, -0.3475, 0.815)),
        material=enamel,
        name="front_upper_rail",
    )
    cabinet.visual(
        Box((0.800, 0.035, 0.205)),
        origin=Origin(xyz=(0.0, -0.3475, 0.1025)),
        material=enamel,
        name="front_lower_rail",
    )

    front_bezel = BezelGeometry(
        (0.470, 0.470),
        (0.605, 0.605),
        0.026,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    cabinet.visual(
        mesh_from_geometry(front_bezel, "cabinet_front_round_bezel"),
        origin=Origin(xyz=(0.0, -0.345, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_white,
        name="front_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.238, length=0.020),
        origin=Origin(xyz=(0.0, -0.326, 0.455), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gap,
        name="porthole_shadow",
    )

    front_vent = VentGrilleGeometry(
        (0.250, 0.075),
        frame=0.012,
        face_thickness=0.004,
        duct_depth=0.022,
        slat_pitch=0.016,
        slat_width=0.007,
        slat_angle_deg=32.0,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
    )
    cabinet.visual(
        mesh_from_geometry(front_vent, "front_exhaust_vent"),
        origin=Origin(xyz=(-0.185, -0.365, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_white,
        name="front_vent",
    )
    rear_vent = VentGrilleGeometry(
        (0.210, 0.105),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.020,
        slat_pitch=0.017,
        slat_width=0.007,
        slat_angle_deg=25.0,
        corner_radius=0.004,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1),
    )
    cabinet.visual(
        mesh_from_geometry(rear_vent, "rear_service_vent"),
        origin=Origin(xyz=(0.220, 0.335, 0.360), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_white,
        name="rear_vent",
    )

    # Fixed hinge pads on the cabinet, just behind the two moving hinge barrels.
    for suffix, z in (("upper", 0.640), ("lower", 0.300)):
        cabinet.visual(
            Box((0.088, 0.036, 0.125)),
            origin=Origin(xyz=(0.368, -0.340, z)),
            material=hinge_metal,
            name=f"{suffix}_fixed_hinge_leaf",
        )
        cabinet.visual(
            Cylinder(radius=0.010, length=0.090),
            origin=Origin(xyz=(0.330, -0.352, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"{suffix}_fixed_hinge_pin_stub",
        )

    for suffix, x in (("left", -0.285), ("right", 0.285)):
        cabinet.visual(
            Box((0.100, 0.100, 0.035)),
            origin=Origin(xyz=(x, 0.0, -0.012)),
            material=dark_plastic,
            name=f"{suffix}_leveling_foot",
        )

    # Rotating drum on a front-to-back axle.
    drum = model.part("drum")
    drum_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.255, -0.230), (0.255, 0.230)],
        inner_profile=[(0.235, 0.230), (0.235, -0.230)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    drum.visual(
        mesh_from_geometry(drum_shell, "hollow_tumble_drum_shell"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.245, length=0.014),
        origin=Origin(xyz=(0.0, 0.226, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="drum_back_plate",
    )
    drum.visual(
        Cylinder(radius=0.034, length=0.115),
        origin=Origin(xyz=(0.0, 0.2525, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="rear_axle_stub",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        # Three low internal lifter ribs make the cylinder read as a dryer drum.
        drum.visual(
            Box((0.055, 0.360, 0.024)),
            origin=Origin(
                xyz=(0.218 * math.cos(angle), 0.0, 0.218 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=brushed_metal,
            name=f"drum_lifter_{idx}",
        )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.015, 0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )

    # Porthole door.  The child frame is on the right-hand hinge line; the door
    # geometry extends along local -X, so positive Z rotation opens it outward.
    door = model.part("porthole_door")
    door_ring = BezelGeometry(
        (0.430, 0.430),
        (0.575, 0.575),
        0.042,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    door.visual(
        mesh_from_geometry(door_ring, "door_round_frame"),
        origin=Origin(xyz=(-0.310, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_white,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=0.224, length=0.014),
        origin=Origin(xyz=(-0.310, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="glass_bowl",
    )
    door.visual(
        Box((0.060, 0.034, 0.170)),
        origin=Origin(xyz=(-0.580, -0.004, 0.0)),
        material=trim_white,
        name="front_handle",
    )
    door.visual(
        Box((0.030, 0.036, 0.105)),
        origin=Origin(xyz=(-0.568, -0.023, 0.0)),
        material=dark_gap,
        name="handle_recess",
    )
    for suffix, z in (("upper", 0.185), ("lower", -0.155)):
        door.visual(
            Cylinder(radius=0.026, length=0.118),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"{suffix}_hinge_barrel",
        )
        door.visual(
            Box((0.145, 0.028, 0.096)),
            origin=Origin(xyz=(-0.070, 0.0, z)),
            material=hinge_metal,
            name=f"{suffix}_moving_hinge_leaf",
        )
    model.articulation(
        "cabinet_to_porthole_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.310, -0.391, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.92),
    )

    # Top controls sit on a service cover that hinges upward at the rear.
    cover = model.part("control_cover")
    cover.visual(
        Box((0.720, 0.240, 0.032)),
        origin=Origin(xyz=(0.0, -0.120, 0.016)),
        material=trim_white,
        name="cover_panel",
    )
    cover.visual(
        Box((0.690, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, -0.006, 0.016)),
        material=hinge_metal,
        name="rear_hinge_leaf",
    )
    cover.visual(
        Cylinder(radius=0.018, length=0.690),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="rear_hinge_barrel",
    )
    cover.visual(
        Box((0.170, 0.050, 0.004)),
        origin=Origin(xyz=(0.185, -0.125, 0.033)),
        material=dark_plastic,
        name="status_window",
    )
    cover.visual(
        Box((0.120, 0.004, 0.003)),
        origin=Origin(xyz=(-0.215, -0.065, 0.033)),
        material=dark_plastic,
        name="dial_label_line",
    )
    model.articulation(
        "cabinet_to_control_cover",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cover,
        origin=Origin(xyz=(0.0, 0.292, 0.900)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.28),
    )

    dial = model.part("timer_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.070,
                0.034,
                body_style="skirted",
                top_diameter=0.052,
                skirt=KnobSkirt(0.082, 0.006, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=24, depth=0.0013),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial_knob",
        ),
        origin=Origin(),
        material=soft_grey,
        name="dial_cap",
    )
    model.articulation(
        "control_cover_to_timer_dial",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=dial,
        origin=Origin(xyz=(-0.245, -0.125, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    for idx, x in enumerate((0.050, 0.115, 0.180)):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.050, 0.034, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=soft_grey,
            name="button_cap",
        )
        model.articulation(
            f"control_cover_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cover,
            child=button,
            origin=Origin(xyz=(x, -0.068, 0.032)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.009),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("porthole_door")
    cover = object_model.get_part("control_cover")
    dial = object_model.get_part("timer_dial")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_porthole_door")
    cover_joint = object_model.get_articulation("cabinet_to_control_cover")

    ctx.expect_overlap(
        drum,
        cabinet,
        axes="xz",
        min_overlap=0.40,
        elem_a="drum_shell",
        elem_b="front_bezel",
        name="drum aligns behind the round front opening",
    )
    ctx.expect_overlap(
        door,
        drum,
        axes="xz",
        min_overlap=0.38,
        elem_a="glass_bowl",
        elem_b="drum_shell",
        name="porthole glass looks into the rotating drum",
    )
    ctx.expect_contact(
        dial,
        cover,
        elem_a="dial_cap",
        elem_b="cover_panel",
        contact_tol=0.003,
        name="timer dial is mounted on the hinged service cover",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.10}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "right-hinged porthole door opens outward",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.18,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.05}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "rear-hinged control cover lifts for service access",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.14,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({drum_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            drum,
            cabinet,
            axes="xz",
            min_overlap=0.40,
            elem_a="drum_shell",
            elem_b="front_bezel",
            name="drum remains on its front-to-back axle while rotating",
        )

    return ctx.report()


object_model = build_object_model()
