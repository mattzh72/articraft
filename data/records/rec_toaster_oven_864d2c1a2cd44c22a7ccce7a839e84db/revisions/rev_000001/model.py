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
    MotionProperties,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_toaster_oven")

    # Utility-size countertop oven, scaled in real meters.
    model.material("powder_coat", rgba=(0.13, 0.15, 0.15, 1.0))
    model.material("black_molded", rgba=(0.025, 0.027, 0.026, 1.0))
    model.material("dark_liner", rgba=(0.045, 0.045, 0.042, 1.0))
    model.material("brushed_steel", rgba=(0.60, 0.58, 0.52, 1.0))
    model.material("warm_element", rgba=(1.0, 0.36, 0.08, 1.0))
    model.material("glass_smoke", rgba=(0.20, 0.28, 0.32, 0.42))
    model.material("white_marking", rgba=(0.92, 0.90, 0.82, 1.0))
    model.material("rubber", rgba=(0.015, 0.016, 0.015, 1.0))

    body = model.part("body")

    # Main reinforced shell: separate service panels visibly overlap as a welded /
    # screwed metal enclosure, while the center front stays open for the oven.
    body.visual(
        Box((0.60, 0.395, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="powder_coat",
        name="bottom_pan",
    )
    body.visual(
        Box((0.60, 0.395, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material="powder_coat",
        name="top_cover",
    )
    body.visual(
        Box((0.026, 0.395, 0.300)),
        origin=Origin(xyz=(-0.287, 0.0, 0.150)),
        material="powder_coat",
        name="left_side_wall",
    )
    body.visual(
        Box((0.026, 0.395, 0.300)),
        origin=Origin(xyz=(0.287, 0.0, 0.150)),
        material="powder_coat",
        name="right_side_wall",
    )
    body.visual(
        Box((0.60, 0.025, 0.300)),
        origin=Origin(xyz=(0.0, 0.185, 0.150)),
        material="powder_coat",
        name="rear_wall",
    )

    # Heavy front perimeter and right-side control fascia.
    body.visual(
        Box((0.032, 0.026, 0.275)),
        origin=Origin(xyz=(-0.286, -0.193, 0.155)),
        material="black_molded",
        name="left_corner_guard",
    )
    body.visual(
        Box((0.032, 0.026, 0.275)),
        origin=Origin(xyz=(0.286, -0.193, 0.155)),
        material="black_molded",
        name="right_corner_guard",
    )
    body.visual(
        Box((0.130, 0.022, 0.255)),
        origin=Origin(xyz=(0.225, -0.193, 0.162)),
        material="powder_coat",
        name="control_fascia",
    )
    body.visual(
        Box((0.460, 0.022, 0.038)),
        origin=Origin(xyz=(-0.065, -0.193, 0.275)),
        material="black_molded",
        name="upper_front_rail",
    )
    body.visual(
        Box((0.460, 0.022, 0.032)),
        origin=Origin(xyz=(-0.065, -0.193, 0.054)),
        material="black_molded",
        name="lower_front_rail",
    )

    opening_bezel = BezelGeometry(
        (0.365, 0.172),
        (0.435, 0.235),
        0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.014,
        outer_corner_radius=0.020,
        face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.0015),
    )
    body.visual(
        mesh_from_geometry(opening_bezel, "oven_opening_bezel"),
        origin=Origin(xyz=(-0.065, -0.202, 0.163), rpy=(math.pi / 2, 0.0, 0.0)),
        material="black_molded",
        name="opening_bezel",
    )

    # Dark cavity liner and rack/heater supports visible through the glass door.
    body.visual(
        Box((0.380, 0.010, 0.180)),
        origin=Origin(xyz=(-0.065, 0.172, 0.163)),
        material="dark_liner",
        name="cavity_back",
    )
    body.visual(
        Box((0.012, 0.340, 0.185)),
        origin=Origin(xyz=(-0.268, -0.005, 0.160)),
        material="dark_liner",
        name="left_liner",
    )
    body.visual(
        Box((0.016, 0.360, 0.185)),
        origin=Origin(xyz=(0.156, -0.010, 0.160)),
        material="dark_liner",
        name="right_liner",
    )
    for idx, z in enumerate((0.116, 0.206)):
        body.visual(
            Box((0.018, 0.270, 0.008)),
            origin=Origin(xyz=(-0.257, -0.020, z)),
            material="brushed_steel",
            name=f"rack_rail_left_{idx}",
        )
        body.visual(
            Box((0.018, 0.270, 0.008)),
            origin=Origin(xyz=(0.140, -0.020, z)),
            material="brushed_steel",
            name=f"rack_rail_right_{idx}",
        )

    for idx, (y, z) in enumerate(((-0.065, 0.082), (0.060, 0.246))):
        body.visual(
            Cylinder(radius=0.007, length=0.380),
            origin=Origin(xyz=(-0.065, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material="warm_element",
            name=f"heater_tube_{idx}",
        )
        body.visual(
            Cylinder(radius=0.011, length=0.020),
            origin=Origin(xyz=(-0.257, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material="brushed_steel",
            name=f"heater_socket_left_{idx}",
        )
        body.visual(
            Cylinder(radius=0.011, length=0.020),
            origin=Origin(xyz=(0.140, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material="brushed_steel",
            name=f"heater_socket_right_{idx}",
        )

    # Serviceable ventilation panels with actual repeated slots/slats.
    top_slots = SlotPatternPanelGeometry(
        (0.330, 0.125),
        0.004,
        slot_size=(0.045, 0.006),
        pitch=(0.060, 0.018),
        frame=0.012,
        corner_radius=0.004,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(top_slots, "top_slot_panel"),
        origin=Origin(xyz=(-0.085, 0.010, 0.312)),
        material="brushed_steel",
        name="top_vent_panel",
    )
    side_grille = VentGrilleGeometry(
        (0.105, 0.135),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.016,
        duct_wall=0.0025,
        slat_pitch=0.018,
        slat_width=0.008,
        slat_angle_deg=28.0,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1, divider_width=0.0035),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        mounts=VentGrilleMounts(style="holes", inset=0.010, hole_diameter=0.003),
        sleeve=VentGrilleSleeve(style="none"),
    )
    body.visual(
        mesh_from_geometry(side_grille, "side_vent_grille"),
        origin=Origin(xyz=(-0.302, 0.020, 0.180), rpy=(0.0, -math.pi / 2, 0.0)),
        material="black_molded",
        name="side_vent_grille",
    )

    # Bottom feet and external seam lines.
    for idx, (x, y) in enumerate(((-0.235, -0.135), (0.235, -0.135), (-0.235, 0.145), (0.235, 0.145))):
        body.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(x, y, -0.009)),
            material="rubber",
            name=f"rubber_foot_{idx}",
        )

    body.visual(
        Box((0.118, 0.004, 0.0025)),
        origin=Origin(xyz=(0.225, -0.206, 0.286)),
        material="black_molded",
        name="fascia_top_seam",
    )
    body.visual(
        Box((0.118, 0.004, 0.0025)),
        origin=Origin(xyz=(0.225, -0.206, 0.038)),
        material="black_molded",
        name="fascia_bottom_seam",
    )
    body.visual(
        Box((0.0025, 0.004, 0.238)),
        origin=Origin(xyz=(0.160, -0.206, 0.162)),
        material="black_molded",
        name="fascia_side_seam",
    )

    # Bushings, labels, and screw heads around the three rotary controls.
    knob_specs = (
        ("temp", 0.230, "TEMP"),
        ("function", 0.162, "MODE"),
        ("timer", 0.094, "TIMER"),
    )
    for idx, (prefix, z, _label) in enumerate(knob_specs):
        x = 0.225
        body.visual(
            Cylinder(radius=0.032, length=0.008),
            origin=Origin(xyz=(x, -0.208, z), rpy=(math.pi / 2, 0.0, 0.0)),
            material="black_molded",
            name=f"{prefix}_bushing",
        )
        body.visual(
            Box((0.050, 0.0025, 0.006)),
            origin=Origin(xyz=(x, -0.2045, z + 0.045)),
            material="white_marking",
            name=f"{prefix}_label_mark",
        )
        for mark_idx, angle in enumerate((-55.0, 0.0, 55.0)):
            rad = math.radians(angle)
            body.visual(
                Box((0.003, 0.0025, 0.012)),
                origin=Origin(
                    xyz=(x + 0.043 * math.sin(rad), -0.2045, z + 0.043 * math.cos(rad)),
                    rpy=(0.0, 0.0, -rad),
                ),
                material="white_marking",
                name=f"{prefix}_tick_{mark_idx}",
            )

    screw_points = [
        (-0.278, -0.208, 0.282),
        (0.278, -0.208, 0.282),
        (-0.278, -0.208, 0.038),
        (0.278, -0.208, 0.038),
        (0.171, -0.208, 0.282),
        (0.279, -0.208, 0.162),
        (0.171, -0.208, 0.038),
    ]
    for idx, xyz in enumerate(screw_points):
        body.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2, 0.0, 0.0)),
            material="brushed_steel",
            name=f"front_screw_{idx}",
        )
        body.visual(
            Box((0.010, 0.0015, 0.0018)),
            origin=Origin(xyz=(xyz[0], xyz[1] - 0.0026, xyz[2]), rpy=(0.0, 0.0, math.pi / 8)),
            material="dark_liner",
            name=f"screw_slot_{idx}",
        )

    # Exposed lower hinge knuckles mounted on the chassis, separated from the
    # rotating door knuckle so the pivot mechanism reads honestly.
    for idx, x in enumerate((-0.210, 0.110)):
        body.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(x, -0.222, 0.045), rpy=(0.0, math.pi / 2, 0.0)),
            material="brushed_steel",
            name=f"body_hinge_barrel_{idx}",
        )
        body.visual(
            Box((0.095, 0.016, 0.010)),
            origin=Origin(xyz=(x, -0.211, 0.048)),
            material="brushed_steel",
            name=f"body_hinge_leaf_{idx}",
        )

    # Rotating front viewing door.  The child frame is the hinge line; all door
    # geometry extends upward at q=0, and positive rotation drops the door out.
    door = model.part("door")
    door.visual(
        Box((0.420, 0.026, 0.038)),
        origin=Origin(xyz=(-0.055, -0.005, 0.027)),
        material="black_molded",
        name="lower_door_rail",
    )
    door.visual(
        Box((0.420, 0.026, 0.040)),
        origin=Origin(xyz=(-0.055, -0.005, 0.232)),
        material="black_molded",
        name="upper_door_rail",
    )
    door.visual(
        Box((0.034, 0.026, 0.222)),
        origin=Origin(xyz=(-0.248, -0.005, 0.130)),
        material="black_molded",
        name="door_side_0",
    )
    door.visual(
        Box((0.034, 0.026, 0.222)),
        origin=Origin(xyz=(0.138, -0.005, 0.130)),
        material="black_molded",
        name="door_side_1",
    )
    door.visual(
        Box((0.350, 0.006, 0.166)),
        origin=Origin(xyz=(-0.055, -0.020, 0.136)),
        material="glass_smoke",
        name="glass_pane",
    )
    door.visual(
        Box((0.342, 0.010, 0.010)),
        origin=Origin(xyz=(-0.055, -0.018, 0.218)),
        material="brushed_steel",
        name="handle_backplate",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.335),
        origin=Origin(xyz=(-0.055, -0.052, 0.218), rpy=(0.0, math.pi / 2, 0.0)),
        material="black_molded",
        name="handle_bar",
    )
    for idx, x in enumerate((-0.205, 0.095)):
        door.visual(
            Box((0.020, 0.040, 0.032)),
            origin=Origin(xyz=(x, -0.038, 0.218)),
            material="black_molded",
            name=f"handle_standoff_{idx}",
        )
    door.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(-0.055, 0.000, 0.000), rpy=(0.0, math.pi / 2, 0.0)),
        material="brushed_steel",
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.155, 0.014, 0.016)),
        origin=Origin(xyz=(-0.055, -0.001, 0.018)),
        material="brushed_steel",
        name="door_hinge_leaf",
    )
    for idx, (x, z) in enumerate(((-0.235, 0.232), (0.125, 0.232), (-0.235, 0.055), (0.125, 0.055))):
        door.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.019, z), rpy=(math.pi / 2, 0.0, 0.0)),
            material="brushed_steel",
            name=f"door_screw_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.222, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.5, lower=0.0, upper=1.65),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )

    # Three independently rotating appliance knobs with hidden stems captured in
    # the fascia bushings.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.028,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.054, 0.006, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            edge_radius=0.0008,
            center=False,
        ),
        "rugged_control_knob",
    )
    for prefix, z, _label in knob_specs:
        knob = model.part(f"{prefix}_knob")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
            material="black_molded",
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
            material="brushed_steel",
            name="stem",
        )
        model.articulation(
            f"fascia_to_{prefix}_knob",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.225, -0.215, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.55, velocity=3.0, lower=0.0, upper=4.7),
            motion_properties=MotionProperties(damping=0.025, friction=0.020),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    # The knobs have short metal stems intentionally captured inside molded
    # fascia bushings.  Scope each allowance to that hidden stem/bushing pair
    # and prove that the stem remains centered and retained.
    for prefix in ("temp", "function", "timer"):
        knob = object_model.get_part(f"{prefix}_knob")
        ctx.allow_overlap(
            body,
            knob,
            elem_a=f"{prefix}_bushing",
            elem_b="stem",
            reason="The rotary control stem is intentionally captured inside the fascia bushing.",
        )
        ctx.expect_within(
            knob,
            body,
            axes="xz",
            inner_elem="stem",
            outer_elem=f"{prefix}_bushing",
            margin=0.001,
            name=f"{prefix} stem centered in bushing",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="y",
            elem_a="stem",
            elem_b=f"{prefix}_bushing",
            min_overlap=0.006,
            name=f"{prefix} stem retained in bushing",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="opening_bezel",
            negative_elem="upper_door_rail",
            min_gap=0.0,
            max_gap=0.012,
            name="closed door seats just in front of the opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="glass_pane",
            elem_b="opening_bezel",
            min_overlap=0.130,
            name="viewing glass covers the framed oven opening",
        )
        closed_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")

    with ctx.pose({door_hinge: 1.20}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")

    if closed_handle_aabb is not None and open_handle_aabb is not None:
        closed_center_y = (closed_handle_aabb[0][1] + closed_handle_aabb[1][1]) * 0.5
        open_center_y = (open_handle_aabb[0][1] + open_handle_aabb[1][1]) * 0.5
        closed_center_z = (closed_handle_aabb[0][2] + closed_handle_aabb[1][2]) * 0.5
        open_center_z = (open_handle_aabb[0][2] + open_handle_aabb[1][2]) * 0.5
        ctx.check(
            "door opens downward and outward",
            open_center_y < closed_center_y - 0.08 and open_center_z < closed_center_z - 0.08,
            details=(
                f"closed_handle_yz=({closed_center_y:.3f}, {closed_center_z:.3f}), "
                f"open_handle_yz=({open_center_y:.3f}, {open_center_z:.3f})"
            ),
        )
    else:
        ctx.fail("door opens downward and outward", "handle AABBs could not be measured")

    knob_joints = [object_model.get_articulation(f"fascia_to_{prefix}_knob") for prefix in ("temp", "function", "timer")]
    ctx.check(
        "three limited rotary controls",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 4.5
            for joint in knob_joints
        ),
        details="Expected temperature, function, and timer knobs with roughly 270 degree travel.",
    )

    return ctx.report()


object_model = build_object_model()
