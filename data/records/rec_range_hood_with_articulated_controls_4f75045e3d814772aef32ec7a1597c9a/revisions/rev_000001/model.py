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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_mesh = model.material("dark_filter_mesh", rgba=(0.08, 0.09, 0.09, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.015, 0.018, 1.0))
    warm_lens = model.material("warm_light_lens", rgba=(1.0, 0.78, 0.35, 0.72))

    canopy = model.part("canopy")

    # Suspended central chimney and ceiling trim.
    canopy.visual(
        Box((0.26, 0.22, 0.685)),
        origin=Origin(xyz=(0.0, 0.0, 0.5375)),
        material=stainless,
        name="chimney",
    )
    canopy.visual(
        Box((0.38, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.8975)),
        material=stainless,
        name="ceiling_trim",
    )

    # Rectangular island canopy shell.  The underside is an open perimeter ring
    # around the filter intake instead of a solid bottom plate.
    canopy.visual(
        Box((1.04, 0.035, 0.16)),
        origin=Origin(xyz=(0.0, 0.3425, 0.08)),
        material=stainless,
        name="front_wall",
    )
    canopy.visual(
        Box((1.04, 0.035, 0.16)),
        origin=Origin(xyz=(0.0, -0.3425, 0.08)),
        material=stainless,
        name="rear_wall",
    )
    canopy.visual(
        Box((0.035, 0.685, 0.16)),
        origin=Origin(xyz=(0.5025, 0.0, 0.08)),
        material=stainless,
        name="side_wall_0",
    )
    canopy.visual(
        Box((0.035, 0.685, 0.16)),
        origin=Origin(xyz=(-0.5025, 0.0, 0.08)),
        material=stainless,
        name="side_wall_1",
    )
    canopy.visual(
        Box((1.00, 0.64, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=stainless,
        name="upper_deck",
    )

    # Lower intake rim leaves a large central opening for the filters.
    canopy.visual(
        Box((1.04, 0.13, 0.025)),
        origin=Origin(xyz=(0.0, 0.2925, 0.010)),
        material=stainless,
        name="front_intake_lip",
    )
    canopy.visual(
        Box((1.04, 0.13, 0.025)),
        origin=Origin(xyz=(0.0, -0.2925, 0.010)),
        material=stainless,
        name="rear_intake_lip",
    )
    canopy.visual(
        Box((0.13, 0.59, 0.025)),
        origin=Origin(xyz=(0.455, 0.0, 0.010)),
        material=stainless,
        name="side_intake_lip_0",
    )
    canopy.visual(
        Box((0.13, 0.59, 0.025)),
        origin=Origin(xyz=(-0.455, 0.0, 0.010)),
        material=stainless,
        name="side_intake_lip_1",
    )

    # Flush warm light lenses under the rim, outside the filter opening.
    canopy.visual(
        Box((0.22, 0.030, 0.006)),
        origin=Origin(xyz=(0.19, 0.292, -0.004)),
        material=warm_lens,
        name="light_lens_0",
    )
    canopy.visual(
        Box((0.22, 0.030, 0.006)),
        origin=Origin(xyz=(0.19, -0.292, -0.004)),
        material=warm_lens,
        name="light_lens_1",
    )

    # A visible hinge line on one long side of the intake opening.  The fixed
    # rail is split into knuckle-supporting segments so the moving knuckles can
    # occupy the alternate gaps without interpenetration.
    canopy.visual(
        Box((0.030, 0.50, 0.035)),
        origin=Origin(xyz=(-0.455, 0.0, -0.005)),
        material=stainless,
        name="hinge_mount_rail",
    )
    for i, y in enumerate((-0.195, 0.0, 0.195)):
        canopy.visual(
            Box((0.065, 0.085, 0.006)),
            origin=Origin(xyz=(-0.424, y, -0.014)),
            material=stainless,
            name=f"fixed_hinge_leaf_{i}",
        )
        canopy.visual(
            Cylinder(radius=0.009, length=0.105),
            origin=Origin(
                xyz=(-0.390, y, -0.014),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=stainless,
            name=f"fixed_hinge_knuckle_{i}",
        )

    # Fixed collars on each side face carry the rotating light knobs.
    for i, x in enumerate((0.509, -0.509)):
        canopy.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(
                xyz=(x, 0.0, 0.055),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stainless,
            name=f"knob_collar_{i}",
        )

    filter_frame = model.part("filter_frame")
    filter_frame.visual(
        Box((0.050, 0.475, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, -0.013)),
        material=stainless,
        name="hinge_side_bar",
    )
    filter_frame.visual(
        Box((0.048, 0.475, 0.018)),
        origin=Origin(xyz=(0.712, 0.0, -0.013)),
        material=stainless,
        name="latch_side_bar",
    )
    filter_frame.visual(
        Box((0.715, 0.040, 0.018)),
        origin=Origin(xyz=(0.375, 0.225, -0.013)),
        material=stainless,
        name="front_frame_bar",
    )
    filter_frame.visual(
        Box((0.715, 0.040, 0.018)),
        origin=Origin(xyz=(0.375, -0.225, -0.013)),
        material=stainless,
        name="rear_frame_bar",
    )
    filter_frame.visual(
        Box((0.625, 0.365, 0.004)),
        origin=Origin(xyz=(0.375, 0.0, -0.016)),
        material=dark_mesh,
        name="filter_screen",
    )
    for i, x in enumerate((0.18, 0.30, 0.42, 0.54)):
        filter_frame.visual(
            Box((0.010, 0.370, 0.008)),
            origin=Origin(xyz=(x, 0.0, -0.011)),
            material=stainless,
            name=f"filter_rib_{i}",
        )
    for i, y in enumerate((-0.12, 0.0, 0.12)):
        filter_frame.visual(
            Box((0.620, 0.008, 0.007)),
            origin=Origin(xyz=(0.375, y, -0.010)),
            material=stainless,
            name=f"filter_crossbar_{i}",
        )
    for i, y in enumerate((-0.0975, 0.0975)):
        filter_frame.visual(
            Box((0.030, 0.085, 0.006)),
            origin=Origin(xyz=(0.020, y, -0.004)),
            material=stainless,
            name=f"moving_hinge_leaf_{i}",
        )
        filter_frame.visual(
            Cylinder(radius=0.009, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"moving_hinge_knuckle_{i}",
        )
    filter_frame.visual(
        Box((0.028, 0.080, 0.010)),
        origin=Origin(xyz=(0.745, 0.0, -0.004)),
        material=black,
        name="release_tab",
    )

    model.articulation(
        "canopy_to_filter",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=filter_frame,
        origin=Origin(xyz=(-0.390, 0.0, -0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.15),
    )

    knob_geometry = KnobGeometry(
        0.055,
        0.030,
        body_style="skirted",
        top_diameter=0.044,
        skirt=KnobSkirt(0.061, 0.006, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geometry, "side_light_knob")

    for i, (x, axis, rot) in enumerate(
        (
            (0.516, (1.0, 0.0, 0.0), math.pi / 2.0),
            (-0.516, (-1.0, 0.0, 0.0), -math.pi / 2.0),
        )
    ):
        knob = model.part(f"side_knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, rot, 0.0)),
            material=black,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(
                xyz=(0.009 if x > 0.0 else -0.009, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name="knob_shaft",
        )
        model.articulation(
            f"canopy_to_knob_{i}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=knob,
            origin=Origin(xyz=(x, 0.0, 0.055)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=6.0,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    filter_frame = object_model.get_part("filter_frame")
    hinge = object_model.get_articulation("canopy_to_filter")
    knob_0 = object_model.get_part("side_knob_0")
    knob_1 = object_model.get_part("side_knob_1")

    ctx.expect_within(
        filter_frame,
        canopy,
        axes="xy",
        margin=0.01,
        name="closed filter frame sits inside the canopy footprint",
    )
    ctx.expect_gap(
        canopy,
        filter_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.06,
        positive_elem="side_intake_lip_0",
        negative_elem="filter_screen",
        name="filter screen is just below the lower intake opening",
    )

    closed_aabb = ctx.part_world_aabb(filter_frame)
    with ctx.pose({hinge: 0.95}):
        opened_aabb = ctx.part_world_aabb(filter_frame)
        ctx.expect_gap(
            canopy,
            filter_frame,
            axis="z",
            min_gap=0.20,
            max_gap=0.90,
            positive_elem="upper_deck",
            negative_elem="latch_side_bar",
            name="opened filter frame swings downward from the canopy",
        )
    ctx.check(
        "filter frame free edge drops when hinged",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][2] < closed_aabb[0][2] - 0.18,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_contact(
        knob_0,
        canopy,
        elem_a="knob_cap",
        elem_b="knob_collar_0",
        contact_tol=0.002,
        name="side knob 0 seats on its side collar",
    )
    ctx.expect_contact(
        knob_1,
        canopy,
        elem_a="knob_cap",
        elem_b="knob_collar_1",
        contact_tol=0.002,
        name="side knob 1 seats on its side collar",
    )

    for i, expected_axis in enumerate(((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0))):
        joint = object_model.get_articulation(f"canopy_to_knob_{i}")
        ctx.check(
            f"side knob {i} rotates about its short side shaft",
            tuple(round(v, 3) for v in joint.axis) == expected_axis
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None,
            details=f"axis={joint.axis}, limits={joint.motion_limits}",
        )

    return ctx.report()


object_model = build_object_model()
