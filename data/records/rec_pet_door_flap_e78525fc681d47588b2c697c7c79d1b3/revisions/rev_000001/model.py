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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_flap_assembly")

    dark_plastic = Material("dark_textured_plastic", rgba=(0.035, 0.035, 0.032, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    smoked_flap = Material("smoked_amber_polycarbonate", rgba=(0.55, 0.34, 0.12, 0.46))
    security_plastic = Material("off_white_security_panel", rgba=(0.86, 0.84, 0.78, 1.0))
    screw_metal = Material("dull_stainless_screw_heads", rgba=(0.55, 0.55, 0.52, 1.0))

    # Object frame: X is width, Y is front/back depth (front is +Y), Z is height.
    # The root is an outer trim frame mounted in the door/wall opening, with
    # rear vertical U-channels that guide the removable/sliding security panel.
    frame = model.part("frame")

    # Four broad trim members leave a real central opening for the pet flap.
    frame.visual(
        Box((0.46, 0.09, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=dark_plastic,
        name="top_rail",
    )
    frame.visual(
        Box((0.46, 0.09, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=dark_plastic,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.07, 0.09, 0.60)),
        origin=Origin(xyz=(-0.195, 0.0, 0.0)),
        material=dark_plastic,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.07, 0.09, 0.60)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        material=dark_plastic,
        name="side_rail_1",
    )

    # A shallow inner stop lip frames the swinging flap without filling the opening.
    frame.visual(
        Box((0.34, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.046, 0.222)),
        material=black_rubber,
        name="top_stop_lip",
    )
    frame.visual(
        Box((0.34, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.046, -0.214)),
        material=black_rubber,
        name="bottom_stop_lip",
    )
    frame.visual(
        Box((0.020, 0.022, 0.41)),
        origin=Origin(xyz=(-0.163, 0.046, 0.0)),
        material=black_rubber,
        name="side_stop_lip_0",
    )
    frame.visual(
        Box((0.020, 0.022, 0.41)),
        origin=Origin(xyz=(0.163, 0.046, 0.0)),
        material=black_rubber,
        name="side_stop_lip_1",
    )

    # Rear sliding-cover side channels.  Each U-channel has an outer wall plus
    # front/back lips, leaving a clear slot around the security panel tongues.
    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        x = sign * 0.182
        frame.visual(
            Box((0.025, 0.040, 0.80)),
            origin=Origin(xyz=(x, -0.065, 0.150)),
            material=dark_plastic,
            name="side_channel_0" if suffix == "0" else "side_channel_1",
        )
        frame.visual(
            Box((0.020, 0.008, 0.80)),
            origin=Origin(xyz=(sign * 0.162, -0.045, 0.150)),
            material=dark_plastic,
            name=f"front_channel_lip_{suffix}",
        )
        frame.visual(
            Box((0.020, 0.008, 0.80)),
            origin=Origin(xyz=(sign * 0.162, -0.085, 0.150)),
            material=dark_plastic,
            name=f"rear_channel_lip_{suffix}",
        )

    # Rear bridge/header for the raised security cover to disappear into.
    frame.visual(
        Box((0.39, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.065, 0.550)),
        material=dark_plastic,
        name="cover_header",
    )

    # Small hinge ears on the frame show where the flap pin is carried.
    frame.visual(
        Box((0.030, 0.036, 0.060)),
        origin=Origin(xyz=(-0.162, 0.018, 0.220)),
        material=dark_plastic,
        name="hinge_ear_0",
    )
    frame.visual(
        Box((0.030, 0.036, 0.060)),
        origin=Origin(xyz=(0.162, 0.018, 0.220)),
        material=dark_plastic,
        name="hinge_ear_1",
    )

    # Front screw heads sit slightly proud of the trim and read as real mounting.
    for i, (x, z) in enumerate(
        [(-0.185, 0.245), (0.185, 0.245), (-0.185, -0.245), (0.185, -0.245)]
    ):
        frame.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, 0.047, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"screw_head_{i}",
        )

    # The rigid flap part is authored with its local frame on the hinge axis.
    # At q=0 it hangs vertically; positive rotation swings the lower edge forward.
    flap = model.part("flap")
    flap.visual(
        Box((0.286, 0.010, 0.372)),
        origin=Origin(xyz=(0.0, 0.0, -0.198)),
        material=smoked_flap,
        name="clear_panel",
    )
    flap.visual(
        Box((0.286, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=black_rubber,
        name="top_gasket",
    )
    flap.visual(
        Box((0.302, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.386)),
        material=black_rubber,
        name="bottom_weight",
    )
    flap.visual(
        Box((0.010, 0.018, 0.372)),
        origin=Origin(xyz=(-0.138, 0.0, -0.198)),
        material=black_rubber,
        name="side_gasket_0",
    )
    flap.visual(
        Box((0.010, 0.018, 0.372)),
        origin=Origin(xyz=(0.138, 0.0, -0.198)),
        material=black_rubber,
        name="side_gasket_1",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.294),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="hinge_barrel",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.018, 0.220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25),
    )

    # The security cover is a separate sliding rigid panel behind the flap.
    # Its side tongues ride in the rear channels; q=0 is fully lowered/secured.
    cover = model.part("security_cover")
    cover.visual(
        Box((0.300, 0.010, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=security_plastic,
        name="cover_panel",
    )
    cover.visual(
        Box((0.025, 0.010, 0.430)),
        origin=Origin(xyz=(-0.157, 0.0, 0.215)),
        material=security_plastic,
        name="side_tongue_0",
    )
    cover.visual(
        Box((0.025, 0.010, 0.430)),
        origin=Origin(xyz=(0.157, 0.0, 0.215)),
        material=security_plastic,
        name="side_tongue_1",
    )
    cover.visual(
        Box((0.120, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.014, 0.260)),
        material=dark_plastic,
        name="finger_pull",
    )

    model.articulation(
        "frame_to_security_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(0.0, -0.065, -0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.330),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    cover = object_model.get_part("security_cover")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    cover_slide = object_model.get_articulation("frame_to_security_cover")

    ctx.check(
        "flap uses upper horizontal revolute axis",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 4) for v in flap_hinge.axis) == (1.0, 0.0, 0.0)
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper >= 1.0,
        details=f"axis={flap_hinge.axis}, limits={flap_hinge.motion_limits}",
    )
    ctx.check(
        "cover uses vertical prismatic channel slide",
        cover_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 4) for v in cover_slide.axis) == (0.0, 0.0, 1.0)
        and cover_slide.motion_limits.upper >= 0.30,
        details=f"axis={cover_slide.axis}, limits={cover_slide.motion_limits}",
    )

    with ctx.pose({flap_hinge: 0.0, cover_slide: 0.0}):
        ctx.expect_contact(
            frame,
            flap,
            elem_a="hinge_ear_0",
            elem_b="hinge_barrel",
            contact_tol=1e-5,
            name="flap hinge barrel is seated in frame ear",
        )
        ctx.expect_gap(
            flap,
            cover,
            axis="y",
            min_gap=0.035,
            name="security cover sits behind closed flap",
        )
        ctx.expect_overlap(
            cover,
            frame,
            axes="z",
            elem_a="side_tongue_0",
            elem_b="side_channel_0",
            min_overlap=0.40,
            name="lowered cover tongue is retained by channel",
        )
        ctx.expect_contact(
            cover,
            frame,
            elem_a="side_tongue_0",
            elem_b="side_channel_0",
            contact_tol=1e-5,
            name="security cover tongue bears on side channel",
        )

    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.05}):
        opened_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "positive flap rotation swings lower edge forward",
        closed_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.20,
        details=f"closed={closed_flap_aabb}, opened={opened_flap_aabb}",
    )

    lowered_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_slide: 0.330}):
        raised_cover_aabb = ctx.part_world_aabb(cover)
        ctx.expect_overlap(
            cover,
            frame,
            axes="z",
            elem_a="side_tongue_0",
            elem_b="side_channel_0",
            min_overlap=0.10,
            name="raised cover remains captured in channel",
        )
    ctx.check(
        "security cover slides upward in frame channels",
        lowered_cover_aabb is not None
        and raised_cover_aabb is not None
        and raised_cover_aabb[0][2] > lowered_cover_aabb[0][2] + 0.30,
        details=f"lowered={lowered_cover_aabb}, raised={raised_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
