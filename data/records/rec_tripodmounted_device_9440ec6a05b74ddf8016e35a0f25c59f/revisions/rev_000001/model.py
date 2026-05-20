from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_tripod_device")

    matte_black = Material("matte_black", color=(0.02, 0.022, 0.025, 1.0))
    dark_graphite = Material("dark_graphite", color=(0.10, 0.105, 0.11, 1.0))
    anodized = Material("dark_anodized_aluminum", color=(0.18, 0.19, 0.20, 1.0))
    rubber = Material("soft_rubber", color=(0.006, 0.006, 0.006, 1.0))
    screen = Material("muted_glass_screen", color=(0.05, 0.09, 0.12, 1.0))
    accent = Material("brushed_silver", color=(0.72, 0.70, 0.64, 1.0))

    # Root hub: a short desktop collar with three radial hinge sockets.  The
    # hinge socket bridges stop at the leg knuckle tangency, so the moving legs
    # can fold without broad intersections.
    base = model.part("base_hub")
    base.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="low_puck",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_graphite,
        name="hinge_collar",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=anodized,
        name="short_mast",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_graphite,
        name="pan_seat",
    )

    leg_pitch = 0.16
    leg_pivot_radius = 0.050
    leg_pivot_z = 0.035
    leg_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    for i, yaw in enumerate(leg_angles):
        base.visual(
            Box((0.024, 0.018, 0.016)),
            origin=Origin(
                xyz=(0.028 * math.cos(yaw), 0.028 * math.sin(yaw), leg_pivot_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_graphite,
            name=f"leg_socket_{i}",
        )

    # Three low, rubber-tipped legs.  Their local +X axes point away from the
    # hub at q=0; the lower limit cancels the pitch for fold-flat stowage.
    for i, yaw in enumerate(leg_angles):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.010, length=0.034),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_graphite,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.190, 0.015, 0.012)),
            origin=Origin(xyz=(0.100, 0.0, 0.0)),
            material=anodized,
            name="tapered_leg",
        )
        leg.visual(
            Box((0.048, 0.030, 0.010)),
            origin=Origin(xyz=(0.215, 0.0, -0.002)),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"base_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(
                    leg_pivot_radius * math.cos(yaw),
                    leg_pivot_radius * math.sin(yaw),
                    leg_pivot_z,
                ),
                rpy=(0.0, leg_pitch, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-leg_pitch, upper=0.42),
        )

    # Pan turntable and real yoke.  The trunnion-yoke mesh gives the tilt head a
    # visible fork, base, and bored cheeks rather than a decorative block.
    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_graphite,
        name="pan_disc",
    )
    pan_head.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=accent,
        name="degree_ring",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.085, 0.038, 0.066),
            span_width=0.052,
            trunnion_diameter=0.014,
            trunnion_center_z=0.046,
            base_thickness=0.012,
            corner_radius=0.003,
            center=False,
        ),
        "compact_tilt_yoke",
    )
    pan_head.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_graphite,
        name="tilt_yoke",
    )
    model.articulation(
        "base_to_pan",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    # Pan friction knob, tangent to the turntable side.
    pan_knob = model.part("pan_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(0.030, 0.014, body_style="lobed", crown_radius=0.001),
        "pan_lobed_knob",
    )
    pan_knob.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="screw_stem",
    )
    pan_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="thumb_wheel",
    )
    model.articulation(
        "pan_to_pan_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=pan_knob,
        origin=Origin(xyz=(0.0, -0.037, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # Tilted head plate and phone bracket, carried on the trunnion axis.
    head = model.part("head_plate")
    head.visual(
        Cylinder(radius=0.007, length=0.078),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="trunnion_pin",
    )
    head.visual(
        Box((0.030, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        material=dark_graphite,
        name="neck_block",
    )
    head.visual(
        Box((0.080, 0.010, 0.125)),
        origin=Origin(xyz=(0.0, 0.050, 0.035)),
        material=dark_graphite,
        name="back_plate",
    )
    head.visual(
        Box((0.008, 0.018, 0.128)),
        origin=Origin(xyz=(-0.043, 0.058, 0.034)),
        material=rubber,
        name="side_rail_0",
    )
    head.visual(
        Box((0.008, 0.018, 0.128)),
        origin=Origin(xyz=(0.043, 0.058, 0.034)),
        material=rubber,
        name="side_rail_1",
    )
    head.visual(
        Box((0.102, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, -0.030)),
        material=rubber,
        name="bottom_shelf",
    )
    head.visual(
        Box((0.070, 0.003, 0.040)),
        origin=Origin(xyz=(0.0, 0.056, 0.020)),
        material=rubber,
        name="back_pad",
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.70, upper=0.95),
    )

    # Coaxial tilt lock knob, outside the yoke cheek and clear of the fork.
    tilt_knob = model.part("tilt_knob")
    tilt_knob_mesh = mesh_from_geometry(
        KnobGeometry(0.034, 0.014, body_style="lobed", crown_radius=0.001),
        "tilt_lobed_knob",
    )
    tilt_knob.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="screw_stem",
    )
    tilt_knob.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="friction_washer",
    )
    tilt_knob.visual(
        tilt_knob_mesh,
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="thumb_wheel",
    )
    model.articulation(
        "pan_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.048, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # The mounted device sits in the rubber shelf and between the guide rails.
    device = model.part("device")
    device.visual(
        Box((0.068, 0.006, 0.105)),
        origin=Origin(),
        material=matte_black,
        name="phone_slab",
    )
    device.visual(
        Box((0.058, 0.0015, 0.088)),
        origin=Origin(xyz=(0.0, 0.0030, 0.003)),
        material=screen,
        name="screen_face",
    )
    model.articulation(
        "head_to_device",
        ArticulationType.FIXED,
        parent=head,
        child=device,
        origin=Origin(xyz=(0.0, 0.061, 0.0285)),
    )

    # Spring clamp jaw slides upward to open.  Its side collars ride outside the
    # bracket rails, leaving visible clearance for safe motion.
    clamp = model.part("clamp_jaw")
    clamp.visual(
        Box((0.112, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.082, 0.089)),
        material=dark_graphite,
        name="top_crossbar",
    )
    clamp.visual(
        Box((0.072, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.065, 0.085)),
        material=rubber,
        name="top_pad",
    )
    clamp.visual(
        Box((0.010, 0.024, 0.055)),
        origin=Origin(xyz=(-0.052, 0.060, 0.060)),
        material=dark_graphite,
        name="slider_ear_0",
    )
    clamp.visual(
        Box((0.010, 0.024, 0.055)),
        origin=Origin(xyz=(0.052, 0.060, 0.060)),
        material=dark_graphite,
        name="slider_ear_1",
    )
    model.articulation(
        "head_to_clamp",
        ArticulationType.PRISMATIC,
        parent=head,
        child=clamp,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.034),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob_mesh = mesh_from_geometry(
        KnobGeometry(0.026, 0.012, body_style="lobed", crown_radius=0.0008),
        "clamp_lobed_knob",
    )
    clamp_knob.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="screw_stem",
    )
    clamp_knob.visual(
        clamp_knob_mesh,
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="thumb_wheel",
    )
    model.articulation(
        "clamp_to_knob",
        ArticulationType.CONTINUOUS,
        parent=clamp,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.091, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head_plate")
    clamp = object_model.get_part("clamp_jaw")
    device = object_model.get_part("device")
    base = object_model.get_part("base_hub")
    pan = object_model.get_part("pan_head")
    clamp_slide = object_model.get_articulation("head_to_clamp")

    ctx.allow_overlap(
        head,
        pan,
        elem_a="trunnion_pin",
        elem_b="tilt_yoke",
        reason=(
            "The trunnion shaft is intentionally captured through the tripod-head yoke; "
            "the yoke mesh represents the bored cheek support as a solid proxy."
        ),
    )

    ctx.check(
        "three foldable tripod legs",
        all(object_model.get_part(f"leg_{i}") is not None for i in range(3))
        and all(object_model.get_articulation(f"base_to_leg_{i}") is not None for i in range(3)),
    )
    ctx.check(
        "tripod head carries pan tilt and clamp adjustments",
        object_model.get_articulation("base_to_pan") is not None
        and object_model.get_articulation("pan_to_tilt") is not None
        and clamp_slide is not None,
    )

    ctx.expect_contact(
        head,
        device,
        elem_a="bottom_shelf",
        elem_b="phone_slab",
        contact_tol=0.0006,
        name="device is seated on the rubber shelf",
    )
    ctx.expect_gap(
        clamp,
        device,
        axis="z",
        positive_elem="top_pad",
        negative_elem="phone_slab",
        min_gap=0.0004,
        max_gap=0.004,
        name="closed clamp leaves safe rubber clearance over device",
    )
    ctx.expect_within(
        device,
        head,
        axes="x",
        inner_elem="phone_slab",
        outer_elem="back_plate",
        margin=0.004,
        name="device width fits inside bracket back plate",
    )
    ctx.expect_gap(
        pan,
        base,
        axis="z",
        positive_elem="pan_disc",
        negative_elem="pan_seat",
        max_gap=0.0008,
        max_penetration=0.0001,
        name="pan disc sits on mast seat without penetration",
    )
    ctx.expect_overlap(
        head,
        pan,
        axes="x",
        elem_a="trunnion_pin",
        elem_b="tilt_yoke",
        min_overlap=0.070,
        name="trunnion remains captured across the yoke cheeks",
    )
    ctx.expect_contact(
        head,
        clamp,
        elem_a="side_rail_0",
        elem_b="slider_ear_0",
        contact_tol=0.0008,
        name="clamp slider ear rides on bracket rail",
    )

    closed_aabb = ctx.part_element_world_aabb(clamp, elem="top_pad")
    with ctx.pose({clamp_slide: 0.030}):
        open_aabb = ctx.part_element_world_aabb(clamp, elem="top_pad")
        ctx.expect_gap(
            clamp,
            device,
            axis="z",
            positive_elem="top_pad",
            negative_elem="phone_slab",
            min_gap=0.028,
            name="opened clamp clears the device top",
        )
    ctx.check(
        "clamp slide opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][2] > closed_aabb[0][2] + 0.025,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # At the stow limit the leg pitch is cancelled, making the three struts lie
    # nearly flat around the low hub for drawer/desk storage.
    leg_joints = {object_model.get_articulation(f"base_to_leg_{i}"): -0.16 for i in range(3)}
    with ctx.pose(leg_joints):
        leg_aabbs = [ctx.part_world_aabb(object_model.get_part(f"leg_{i}")) for i in range(3)]
    ctx.check(
        "leg stow pose is fold-flat",
        all(aabb is not None and (aabb[1][2] - aabb[0][2]) < 0.035 for aabb in leg_aabbs),
        details=f"leg_aabbs={leg_aabbs}",
    )

    return ctx.report()


object_model = build_object_model()
