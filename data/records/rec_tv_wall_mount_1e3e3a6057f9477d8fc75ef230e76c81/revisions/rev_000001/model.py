from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL = "powder_coated_steel"
EDGE_STEEL = "zinc_plated_steel"
RUBBER = "black_rubber"
LABEL = "engraved_black"


def _origin(x=0.0, y=0.0, z=0.0, r=0.0, p=0.0, w=0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(r, p, w))


def _tube_z(outer_radius: float, inner_radius: float, length: float):
    """Annular cylinder with its axis on local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _wall_plate_cq():
    """Broad fabricated wall plate with real through-slots and cable passage."""
    plate = cq.Workplane("XY").box(0.014, 0.66, 0.50)

    # Four vertical wall fastener slots plus two auxiliary datum slots.
    for y in (-0.235, 0.235):
        for z in (-0.155, 0.155):
            cutter = (
                cq.Workplane("YZ")
                .center(y, z)
                .slot2D(0.118, 0.028, 90)
                .extrude(0.08, both=True)
            )
            plate = plate.cut(cutter)

    for y in (-0.105, 0.105):
        cutter = (
            cq.Workplane("YZ")
            .center(y, 0.0)
            .slot2D(0.095, 0.020, 0)
            .extrude(0.08, both=True)
        )
        plate = plate.cut(cutter)

    cable_passage = (
        cq.Workplane("YZ")
        .center(0.0, -0.015)
        .slot2D(0.185, 0.072, 0)
        .extrude(0.08, both=True)
    )
    return plate.cut(cable_passage).edges("|X").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_tv_wall_mount_study")

    model.material(STEEL, rgba=(0.055, 0.060, 0.064, 1.0))
    model.material(EDGE_STEEL, rgba=(0.58, 0.58, 0.53, 1.0))
    model.material(RUBBER, rgba=(0.008, 0.008, 0.007, 1.0))
    model.material(LABEL, rgba=(0.015, 0.017, 0.018, 1.0))

    # Root: a wide slotted steel wall plate. The frame origin is the vertical
    # pan axis, slightly proud of the plate face.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_cq(), "slotted_wall_plate", tolerance=0.0007),
        origin=_origin(-0.045, 0.0, 0.0),
        material=STEEL,
        name="slotted_plate",
    )

    # Formed-edge stiffeners and a raised service-cover area on the plate face.
    for y in (-0.335, 0.335):
        wall_plate.visual(
            Box((0.018, 0.026, 0.50)),
            origin=_origin(-0.036, y, 0.0),
            material=STEEL,
            name=f"side_flange_{'neg' if y < 0 else 'pos'}",
        )
    for z in (-0.255, 0.255):
        wall_plate.visual(
            Box((0.018, 0.66, 0.024)),
            origin=_origin(-0.036, 0.0, z),
            material=STEEL,
            name=f"edge_return_{'low' if z < 0 else 'high'}",
        )
    wall_plate.visual(
        Box((0.006, 0.205, 0.105)),
        origin=_origin(-0.035, 0.0, -0.015),
        material=STEEL,
        name="access_cover",
    )
    for y in (-0.112, 0.112):
        wall_plate.visual(
            Box((0.007, 0.014, 0.125)),
            origin=_origin(-0.031, y, -0.015),
            material=EDGE_STEEL,
            name=f"cover_slide_{'neg' if y < 0 else 'pos'}",
        )

    # Cable-passage grommet built as four seated bars around the through opening.
    wall_plate.visual(
        Box((0.004, 0.210, 0.015)),
        origin=_origin(-0.032, 0.0, 0.030),
        material=RUBBER,
        name="cable_lip_top",
    )
    wall_plate.visual(
        Box((0.004, 0.210, 0.015)),
        origin=_origin(-0.032, 0.0, -0.060),
        material=RUBBER,
        name="cable_lip_bottom",
    )
    for y in (-0.110, 0.110):
        wall_plate.visual(
            Box((0.004, 0.015, 0.090)),
            origin=_origin(-0.032, y, -0.015),
            material=RUBBER,
            name=f"cable_lip_{'neg' if y < 0 else 'pos'}",
        )

    # Captive pan bearing: two hollow collars on the wall plate.
    for z, collar_name, mesh_name in (
        (-0.070, "pan_collar_low", "wall_pan_collar_low"),
        (0.070, "pan_collar_high", "wall_pan_collar_high"),
    ):
        wall_plate.visual(
            mesh_from_cadquery(
                _tube_z(0.060, 0.033, 0.060),
                mesh_name,
                tolerance=0.0006,
            ),
            origin=_origin(0.0, 0.0, z),
            material=EDGE_STEEL,
            name=collar_name,
        )

    # Slot washers/fasteners are seated over the slotted holes and visually
    # reinforce that this is a mechanical wall mount, not a device housing.
    for y in (-0.235, 0.235):
        for z in (-0.155, 0.155):
            wall_plate.visual(
                Cylinder(radius=0.023, length=0.006),
                origin=_origin(-0.035, y, z, p=math.pi / 2.0),
                material=EDGE_STEEL,
                name=f"mount_washer_{'n' if y < 0 else 'p'}_{'l' if z < 0 else 'h'}",
            )
            wall_plate.visual(
                Cylinder(radius=0.008, length=0.008),
                origin=_origin(-0.031, y, z, p=math.pi / 2.0),
                material=EDGE_STEEL,
                name=f"socket_head_{'n' if y < 0 else 'p'}_{'l' if z < 0 else 'h'}",
            )

    # First moving stage: a pan shaft and a rigid double-link shoulder arm.
    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.026, length=0.275),
        origin=_origin(0.0, 0.0, 0.0),
        material=EDGE_STEEL,
        name="pan_shaft",
    )
    for z, plate_name in (
        (-0.107, "crank_plate_low"),
        (0.107, "crank_plate_high"),
    ):
        shoulder_link.visual(
            Box((0.170, 0.145, 0.014)),
            origin=_origin(0.080, 0.0, z),
            material=STEEL,
            name=plate_name,
        )
    for y in (-0.079, 0.079):
        shoulder_link.visual(
            Box((0.170, 0.026, 0.250)),
            origin=_origin(0.105, y, 0.0),
            material=STEEL,
            name=f"pan_web_{'neg' if y < 0 else 'pos'}",
        )

    # Upper/lower pairs make the folded load path obvious and keep the arm
    # visually rigid without relying on a decorative shell.
    for y in (-0.102, 0.102):
        for z in (-0.052, 0.052):
            shoulder_link.visual(
                Box((0.330, 0.030, 0.026)),
                origin=_origin(0.245, y, z),
                material=STEEL,
                name=f"shoulder_rail_{'n' if y < 0 else 'p'}_{'l' if z < 0 else 'h'}",
            )

    shoulder_link.visual(
        Cylinder(radius=0.025, length=0.270),
        origin=_origin(0.250, 0.0, 0.104, p=math.pi / 2.0),
        material=EDGE_STEEL,
        name="spring_assist_tube",
    )
    for x in (0.120, 0.380):
        shoulder_link.visual(
            Box((0.040, 0.100, 0.035)),
            origin=_origin(x, 0.0, 0.085),
            material=STEEL,
            name=f"spring_saddle_{'rear' if x < 0.2 else 'front'}",
        )
    shoulder_link.visual(
        Box((0.230, 0.044, 0.006)),
        origin=_origin(0.250, 0.0, 0.132),
        material=STEEL,
        name="spring_cover",
    )

    # Elbow fold bearing, modeled as a hollow sleeve connected to the first arm.
    shoulder_link.visual(
        mesh_from_cadquery(
            _tube_z(0.052, 0.032, 0.140),
            "elbow_bearing_sleeve",
            tolerance=0.0006,
        ),
        origin=_origin(0.420, 0.0, 0.0),
        material=EDGE_STEEL,
        name="elbow_sleeve",
    )
    for y in (-0.071, 0.071):
        shoulder_link.visual(
            Box((0.095, 0.038, 0.165)),
            origin=_origin(0.398, y, 0.0),
            material=STEEL,
            name=f"elbow_tang_{'neg' if y < 0 else 'pos'}",
        )

    # Second moving stage: a folding forearm with its own vertical pin and a
    # trunnion yoke at the display-frame end.
    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        Cylinder(radius=0.024, length=0.190),
        origin=_origin(0.0, 0.0, 0.0),
        material=EDGE_STEEL,
        name="elbow_pin",
    )
    for z in (-0.091, 0.091):
        forearm_link.visual(
            Box((0.110, 0.155, 0.014)),
            origin=_origin(0.044, 0.0, z),
            material=STEEL,
            name=f"elbow_keeper_{'low' if z < 0 else 'high'}",
        )
    for y in (-0.066, 0.066):
        forearm_link.visual(
            Box((0.115, 0.024, 0.178)),
            origin=_origin(0.095, y, 0.0),
            material=STEEL,
            name=f"forearm_web_{'neg' if y < 0 else 'pos'}",
        )
    for y in (-0.074, 0.074):
        for z in (-0.038, 0.038):
            forearm_link.visual(
                Box((0.285, 0.026, 0.024)),
                origin=_origin(0.205, y, z),
                material=STEEL,
                name=f"forearm_rail_{'n' if y < 0 else 'p'}_{'l' if z < 0 else 'h'}",
            )
    forearm_link.visual(
        Box((0.044, 0.180, 0.070)),
        origin=_origin(0.235, 0.0, 0.0),
        material=STEEL,
        name="cable_guide_bridge",
    )
    forearm_link.visual(
        Cylinder(radius=0.021, length=0.120),
        origin=_origin(0.235, 0.0, 0.000, r=-math.pi / 2.0),
        material=RUBBER,
        name="cable_guide_ring",
    )
    for z in (-0.058, 0.058):
        forearm_link.visual(
            Box((0.060, 0.230, 0.030)),
            origin=_origin(0.330, 0.0, z),
            material=STEEL,
            name=f"yoke_bridge_{'low' if z < 0 else 'high'}",
        )
    # Tilt yoke cheeks and bearing bosses. The child trunnion sits between them
    # with visible clearance.
    for y, cheek_name, boss_name in (
        (-0.105, "tilt_cheek_neg", "tilt_boss_neg"),
        (0.105, "tilt_cheek_pos", "tilt_boss_pos"),
    ):
        forearm_link.visual(
            Box((0.083, 0.018, 0.185)),
            origin=_origin(0.380, y, 0.0),
            material=STEEL,
            name=cheek_name,
        )
        forearm_link.visual(
            Cylinder(radius=0.040, length=0.012),
            origin=_origin(0.380, math.copysign(0.0902, y), 0.0, r=-math.pi / 2.0),
            material=EDGE_STEEL,
            name=boss_name,
        )

    # Final moving stage: an open tilt frame, intentionally empty and detailed
    # with rail slots so no display is implied.
    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.018, length=0.1684),
        origin=_origin(0.0, 0.0, 0.0, r=-math.pi / 2.0),
        material=EDGE_STEEL,
        name="tilt_trunnion",
    )
    tilt_frame.visual(
        Box((0.095, 0.118, 0.042)),
        origin=_origin(0.044, 0.0, 0.0),
        material=STEEL,
        name="trunnion_block",
    )
    for y in (-0.220, 0.220):
        tilt_frame.visual(
            Box((0.025, 0.038, 0.430)),
            origin=_origin(0.092, y, 0.0),
            material=STEEL,
            name=f"vertical_rail_{'neg' if y < 0 else 'pos'}",
        )
        for z in (-0.122, 0.0, 0.122):
            tilt_frame.visual(
                Box((0.004, 0.028, 0.066)),
                origin=_origin(0.105, y, z),
                material=LABEL,
                name=f"rail_slot_{'n' if y < 0 else 'p'}_{'l' if z < -0.01 else 'm' if z < 0.01 else 'h'}",
            )
    for z in (-0.190, 0.190):
        tilt_frame.visual(
            Box((0.025, 0.482, 0.034)),
            origin=_origin(0.092, 0.0, z),
            material=STEEL,
            name=f"crossbar_{'low' if z < 0 else 'high'}",
        )
    tilt_frame.visual(
        Box((0.020, 0.075, 0.360)),
        origin=_origin(0.096, 0.0, 0.0),
        material=STEEL,
        name="center_stay",
    )
    for y in (-0.026, 0.026):
        tilt_frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=_origin(0.105, y, 0.0, p=math.pi / 2.0),
            material=EDGE_STEEL,
            name=f"stay_fastener_{'neg' if y < 0 else 'pos'}",
        )

    model.articulation(
        "shoulder_pan",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=shoulder_link,
        origin=_origin(0.0, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.85, lower=-0.75, upper=0.75),
        motion_properties=MotionProperties(damping=0.25, friction=0.08),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm_link,
        origin=_origin(0.420, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.75, lower=-1.45, upper=1.45),
        motion_properties=MotionProperties(damping=0.20, friction=0.06),
    )
    model.articulation(
        "frame_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=tilt_frame,
        origin=_origin(0.380, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.55, lower=-0.35, upper=0.25),
        motion_properties=MotionProperties(damping=0.32, friction=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    shoulder_link = object_model.get_part("shoulder_link")
    forearm_link = object_model.get_part("forearm_link")
    tilt_frame = object_model.get_part("tilt_frame")

    shoulder_pan = object_model.get_articulation("shoulder_pan")
    elbow_fold = object_model.get_articulation("elbow_fold")
    frame_tilt = object_model.get_articulation("frame_tilt")

    ctx.check(
        "three explicit revolute mechanisms",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder_pan, elbow_fold, frame_tilt)
        ),
        details="pan, fold, and tilt must all be articulated revolute joints",
    )
    ctx.check(
        "mechanical study has no display part",
        all("display" not in part.name and "screen" not in part.name for part in object_model.parts),
        details="assembly should remain a bare wall-mount mechanism",
    )

    # Bearing and pivot checks: these prove the visible shafts, sleeves, and
    # trunnions are not decorative isolated forms.
    ctx.expect_contact(
        shoulder_link,
        wall_plate,
        elem_a="crank_plate_high",
        elem_b="pan_collar_high",
        contact_tol=0.001,
        name="upper pan thrust plate seats on collar",
    )
    ctx.expect_within(
        forearm_link,
        shoulder_link,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="elbow_sleeve",
        margin=0.002,
        name="elbow pin stays centered in sleeve",
    )
    ctx.expect_overlap(
        forearm_link,
        shoulder_link,
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_sleeve",
        min_overlap=0.12,
        name="elbow pin is retained through sleeve height",
    )
    ctx.expect_contact(
        tilt_frame,
        forearm_link,
        elem_a="tilt_trunnion",
        elem_b="tilt_boss_pos",
        contact_tol=0.001,
        name="tilt trunnion reaches side bearing",
    )

    rest_forearm = ctx.part_world_position(forearm_link)
    with ctx.pose({shoulder_pan: 0.45}):
        panned_forearm = ctx.part_world_position(forearm_link)
    ctx.check(
        "shoulder pan swings arm laterally",
        rest_forearm is not None
        and panned_forearm is not None
        and panned_forearm[1] > rest_forearm[1] + 0.10,
        details=f"rest={rest_forearm}, panned={panned_forearm}",
    )

    rest_frame = ctx.part_world_position(tilt_frame)
    with ctx.pose({elbow_fold: 0.85}):
        folded_frame = ctx.part_world_position(tilt_frame)
    ctx.check(
        "elbow fold changes frame side offset",
        rest_frame is not None
        and folded_frame is not None
        and folded_frame[1] > rest_frame[1] + 0.20,
        details=f"rest={rest_frame}, folded={folded_frame}",
    )

    rest_aabb = ctx.part_world_aabb(tilt_frame)
    with ctx.pose({frame_tilt: 0.25}):
        tilted_aabb = ctx.part_world_aabb(tilt_frame)
    ctx.check(
        "positive tilt pitches frame forward",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][0] > rest_aabb[1][0] + 0.035,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
