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
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cat_flap_assembly")

    dark_plastic = model.material("dark_warm_gray_plastic", rgba=(0.05, 0.055, 0.055, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.005, 0.006, 0.007, 1.0))
    smoke_clear = model.material("smoky_translucent_flap", rgba=(0.30, 0.46, 0.58, 0.47))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    white_print = model.material("white_selector_print", rgba=(0.92, 0.90, 0.84, 1.0))
    red_print = model.material("red_lock_print", rgba=(0.85, 0.10, 0.06, 1.0))

    frame = model.part("frame")
    frame_depth = 0.055

    # A molded rectangular frame with a clear opening (about 225 mm by 240 mm)
    # and an integrated C-shaped upper hood that captures the flap hinge barrel.
    frame.visual(
        Box((0.052, frame_depth, 0.380)),
        origin=Origin(xyz=(-0.134, 0.0, 0.190)),
        material=dark_plastic,
        name="side_jamb_0",
    )
    frame.visual(
        Box((0.052, frame_depth, 0.380)),
        origin=Origin(xyz=(0.134, 0.0, 0.190)),
        material=dark_plastic,
        name="side_jamb_1",
    )
    frame.visual(
        Box((0.320, frame_depth, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=dark_plastic,
        name="top_lintel",
    )
    frame.visual(
        Box((0.320, frame_depth, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_plastic,
        name="bottom_sill",
    )

    frame.visual(
        Box((0.242, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, -0.008, 0.326)),
        material=dark_plastic,
        name="hinge_hood_top",
    )
    frame.visual(
        Box((0.242, 0.009, 0.052)),
        origin=Origin(xyz=(0.0, -0.043, 0.304)),
        material=dark_plastic,
        name="hood_front_lip",
    )
    frame.visual(
        Box((0.242, 0.009, 0.052)),
        origin=Origin(xyz=(0.0, 0.024, 0.304)),
        material=dark_plastic,
        name="hood_rear_lip",
    )
    frame.visual(
        Box((0.220, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.030, 0.282)),
        material=rubber,
        name="soft_top_stop",
    )

    dial_center = (0.125, -0.0335, 0.050)
    frame.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(dial_center[0], -0.0305, dial_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="dial_boss",
    )
    # Four printed detent marks around the selector: free, in-only, out-only,
    # and locked. They are thin raised/printed pads seated into the frame face.
    frame.visual(
        Box((0.012, 0.0016, 0.004)),
        origin=Origin(xyz=(dial_center[0], -0.0282, dial_center[2] + 0.034)),
        material=white_print,
        name="selector_mark_0",
    )
    frame.visual(
        Box((0.004, 0.0016, 0.012)),
        origin=Origin(xyz=(dial_center[0] + 0.034, -0.0282, dial_center[2])),
        material=white_print,
        name="selector_mark_1",
    )
    frame.visual(
        Box((0.012, 0.0016, 0.004)),
        origin=Origin(xyz=(dial_center[0], -0.0282, dial_center[2] - 0.034)),
        material=red_print,
        name="selector_mark_2",
    )
    frame.visual(
        Box((0.004, 0.0016, 0.012)),
        origin=Origin(xyz=(dial_center[0] - 0.034, -0.0282, dial_center[2])),
        material=white_print,
        name="selector_mark_3",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.204, 0.007, 0.222)),
        origin=Origin(xyz=(0.0, 0.0, -0.124)),
        material=smoke_clear,
        name="panel",
    )
    flap.visual(
        Box((0.198, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=smoke_clear,
        name="top_web",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke_clear,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.178, 0.009, 0.016)),
        origin=Origin(xyz=(0.0, -0.0005, -0.231)),
        material=rubber,
        name="bottom_seal",
    )
    flap.visual(
        Box((0.160, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.0045, -0.052)),
        material=smoke_clear,
        name="upper_rib",
    )
    flap.visual(
        Box((0.010, 0.003, 0.162)),
        origin=Origin(xyz=(-0.087, -0.0045, -0.138)),
        material=smoke_clear,
        name="stiffening_rib_0",
    )
    flap.visual(
        Box((0.010, 0.003, 0.162)),
        origin=Origin(xyz=(0.087, -0.0045, -0.138)),
        material=smoke_clear,
        name="stiffening_rib_1",
    )

    selector_dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.014,
            body_style="faceted",
            top_diameter=0.045,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0010, width=0.0018),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        ),
        "selector_dial_cap",
    )
    selector_dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="dial_cap",
    )
    selector_dial.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="dial_stem",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, 0.307)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-1.05, upper=1.05),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )
    model.articulation(
        "frame_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=selector_dial,
        origin=Origin(xyz=dial_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.02, friction=0.04),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    dial = object_model.get_part("selector_dial")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    dial_joint = object_model.get_articulation("frame_to_selector_dial")

    ctx.allow_overlap(
        frame,
        dial,
        elem_a="dial_boss",
        elem_b="dial_stem",
        reason="The selector dial stem is intentionally captured inside the molded boss socket.",
    )

    ctx.check(
        "flap hinge is the upper horizontal axis",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}",
    )
    ctx.check(
        "selector dial rotates on frame-normal axis",
        dial_joint.articulation_type == ArticulationType.REVOLUTE and tuple(dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )

    with ctx.pose({flap_hinge: 0.0, dial_joint: 0.0}):
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            positive_elem="side_jamb_1",
            negative_elem="panel",
            min_gap=0.003,
            max_gap=0.012,
            name="panel clears right jamb",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="x",
            positive_elem="panel",
            negative_elem="side_jamb_0",
            min_gap=0.003,
            max_gap=0.012,
            name="panel clears left jamb",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem="panel",
            negative_elem="bottom_sill",
            min_gap=0.001,
            max_gap=0.006,
            name="flap bottom just clears sill",
        )
        ctx.expect_within(
            flap,
            frame,
            axes="x",
            inner_elem="hinge_barrel",
            outer_elem="hinge_hood_top",
            margin=0.002,
            name="hinge barrel is clipped under hood width",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            positive_elem="hinge_hood_top",
            negative_elem="hinge_barrel",
            min_gap=0.001,
            max_gap=0.006,
            name="hinge barrel runs under hood cap",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="y",
            positive_elem="hinge_barrel",
            negative_elem="hood_front_lip",
            min_gap=0.020,
            max_gap=0.040,
            name="front hood lip captures barrel",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="y",
            positive_elem="hood_rear_lip",
            negative_elem="hinge_barrel",
            min_gap=0.006,
            max_gap=0.020,
            name="rear hood lip captures barrel",
        )
        ctx.expect_contact(
            dial,
            frame,
            elem_a="dial_cap",
            elem_b="dial_boss",
            contact_tol=0.0015,
            name="selector dial seats on lower-corner boss",
        )
        ctx.expect_within(
            dial,
            frame,
            axes="xz",
            inner_elem="dial_stem",
            outer_elem="dial_boss",
            margin=0.002,
            name="selector stem is centered in boss socket",
        )
        ctx.expect_overlap(
            dial,
            frame,
            axes="y",
            elem_a="dial_stem",
            elem_b="dial_boss",
            min_overlap=0.004,
            name="selector stem remains inserted in boss",
        )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.85}):
        inward_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: -0.85}):
        outward_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "flap swings both ways through the opening",
        closed_aabb is not None
        and inward_aabb is not None
        and outward_aabb is not None
        and inward_aabb[1][1] > closed_aabb[1][1] + 0.12
        and outward_aabb[0][1] < closed_aabb[0][1] - 0.12,
        details=f"closed={closed_aabb}, inward={inward_aabb}, outward={outward_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
