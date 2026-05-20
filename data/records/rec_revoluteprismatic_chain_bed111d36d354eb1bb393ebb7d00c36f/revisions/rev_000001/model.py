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
    model = ArticulatedObject(name="compact_inspection_arm")

    dark_cast = model.material("dark_cast_iron", rgba=(0.10, 0.115, 0.12, 1.0))
    graphite = model.material("graphite_beam", rgba=(0.06, 0.07, 0.075, 1.0))
    machined = model.material("machined_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    oiled = model.material("oiled_steel", rgba=(0.28, 0.30, 0.30, 1.0))
    bronze = model.material("bronze_wear_faces", rgba=(0.70, 0.48, 0.20, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.016, 0.017, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.42, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_cast,
        name="ground_plate",
    )
    base.visual(
        Box((0.36, 0.26, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_cast,
        name="raised_plinth",
    )
    base.visual(
        Box((0.22, 0.19, 0.115)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_cast,
        name="pivot_block",
    )
    base.visual(
        Cylinder(radius=0.125, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=oiled,
        name="bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        material=machined,
        name="upper_bearing_face",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.256)),
        material=black,
        name="grease_seal",
    )
    for i, (x, y) in enumerate(
        ((-0.215, -0.145), (-0.215, 0.145), (0.215, -0.145), (0.215, 0.145))
    ):
        base.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(x, y, 0.044)),
            material=black,
            name=f"anchor_bolt_{i}",
        )
    for i, y in enumerate((-0.118, 0.118)):
        base.visual(
            Box((0.29, 0.020, 0.016)),
            origin=Origin(xyz=(0.0, y, 0.108)),
            material=machined,
            name=f"machined_side_face_{i}",
        )

    beam = model.part("beam")
    # The beam part frame is the vertical slew axis.  The rectangular beam is a
    # built-up hollow box section, leaving a real centerline tunnel for the ram.
    beam.visual(
        Cylinder(radius=0.142, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=machined,
        name="lower_bearing_face",
    )
    beam.visual(
        Cylinder(radius=0.105, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=oiled,
        name="rotary_hub",
    )
    beam.visual(
        Box((0.190, 0.210, 0.085)),
        origin=Origin(xyz=(0.080, 0.0, 0.078)),
        material=graphite,
        name="root_saddle",
    )

    # Hollow rectangular beam walls.
    beam.visual(
        Box((0.720, 0.190, 0.036)),
        origin=Origin(xyz=(0.450, 0.0, 0.202)),
        material=graphite,
        name="top_wall",
    )
    beam.visual(
        Box((0.720, 0.190, 0.036)),
        origin=Origin(xyz=(0.450, 0.0, 0.038)),
        material=graphite,
        name="bottom_wall",
    )
    for i, y in enumerate((-0.078, 0.078)):
        beam.visual(
            Box((0.720, 0.034, 0.180)),
            origin=Origin(xyz=(0.450, y, 0.120)),
            material=graphite,
            name=f"side_web_{i}",
        )

    # External ribs make the deep beam read as a compact, heavy casting.
    for i, x in enumerate((0.185, 0.345, 0.505)):
        beam.visual(
            Box((0.026, 0.218, 0.194)),
            origin=Origin(xyz=(x, 0.0, 0.120)),
            material=graphite,
            name=f"beam_rib_{i}",
        )
    for i, x in enumerate((0.255, 0.575)):
        beam.visual(
            Box((0.105, 0.012, 0.110)),
            origin=Origin(xyz=(x, -0.101, 0.120)),
            material=machined,
            name=f"inspection_flat_{i}",
        )

    # Nose sleeve: four separated guide bars around the opening; the aperture is
    # deliberately larger than the ram and is checked in run_tests().
    beam.visual(
        Box((0.140, 0.222, 0.040)),
        origin=Origin(xyz=(0.810, 0.0, 0.202)),
        material=graphite,
        name="nose_top_rail",
    )
    beam.visual(
        Box((0.140, 0.222, 0.040)),
        origin=Origin(xyz=(0.810, 0.0, 0.038)),
        material=graphite,
        name="nose_bottom_rail",
    )
    for i, y in enumerate((-0.088, 0.088)):
        beam.visual(
            Box((0.140, 0.044, 0.196)),
            origin=Origin(xyz=(0.810, y, 0.120)),
            material=graphite,
            name=f"nose_side_rail_{i}",
        )
    for i, y in enumerate((-0.056, 0.056)):
        beam.visual(
            Box((0.250, 0.010, 0.088)),
            origin=Origin(xyz=(0.705, y, 0.120)),
            material=bronze,
            name=f"wear_strip_{i}",
        )
    beam.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.790, -0.114, 0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="side_boss_0",
    )
    beam.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.790, 0.114, 0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="side_boss_1",
    )

    ram = model.part("ram")
    # The ram frame sits on the nose opening centerline.  Negative X is hidden
    # inside the sleeve for retained insertion; positive X is the exposed stroke.
    ram.visual(
        Box((0.365, 0.102, 0.068)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
        material=machined,
        name="ram_bar",
    )
    ram.visual(
        Box((0.030, 0.102, 0.078)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=oiled,
        name="wiper_band",
    )
    ram.visual(
        Box((0.032, 0.102, 0.082)),
        origin=Origin(xyz=(0.168, 0.0, 0.0)),
        material=machined,
        name="end_block",
    )
    ram.visual(
        Cylinder(radius=0.028, length=0.052),
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oiled,
        name="probe_nose",
    )
    ram.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.251, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="contact_tip",
    )

    model.articulation(
        "base_to_beam",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.9, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "beam_to_ram",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=ram,
        origin=Origin(xyz=(0.790, 0.0, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.165),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    ram = object_model.get_part("ram")
    slew = object_model.get_articulation("base_to_beam")
    slide = object_model.get_articulation("beam_to_ram")

    ctx.expect_contact(
        beam,
        base,
        elem_a="lower_bearing_face",
        elem_b="upper_bearing_face",
        contact_tol=0.001,
        name="slew bearing faces are seated",
    )
    ctx.expect_overlap(
        beam,
        base,
        axes="xy",
        elem_a="lower_bearing_face",
        elem_b="upper_bearing_face",
        min_overlap=0.20,
        name="slew bearing faces have broad overlap",
    )

    def check_ram_clearance(label: str) -> None:
        ctx.expect_gap(
            beam,
            ram,
            axis="z",
            positive_elem="nose_top_rail",
            negative_elem="ram_bar",
            min_gap=0.012,
            name=f"{label} ram clears top guide",
        )
        ctx.expect_gap(
            ram,
            beam,
            axis="z",
            positive_elem="ram_bar",
            negative_elem="nose_bottom_rail",
            min_gap=0.012,
            name=f"{label} ram clears bottom guide",
        )
        ctx.expect_gap(
            beam,
            ram,
            axis="y",
            positive_elem="nose_side_rail_1",
            negative_elem="ram_bar",
            min_gap=0.014,
            name=f"{label} ram clears positive side rail",
        )
        ctx.expect_gap(
            ram,
            beam,
            axis="y",
            positive_elem="ram_bar",
            negative_elem="nose_side_rail_0",
            min_gap=0.014,
            name=f"{label} ram clears negative side rail",
        )
        ctx.expect_overlap(
            ram,
            beam,
            axes="x",
            elem_a="ram_bar",
            elem_b="nose_top_rail",
            min_overlap=0.065,
            name=f"{label} ram remains captured in sleeve",
        )

    with ctx.pose({slide: 0.0, slew: 0.0}):
        check_ram_clearance("retracted")
        retracted_pos = ctx.part_world_position(ram)

    with ctx.pose({slide: 0.165, slew: 0.0}):
        check_ram_clearance("extended")
        extended_pos = ctx.part_world_position(ram)

    ctx.check(
        "ram extends along beam centerline",
        retracted_pos is not None
        and extended_pos is not None
        and extended_pos[0] > retracted_pos[0] + 0.150
        and abs(extended_pos[1] - retracted_pos[1]) < 1.0e-6
        and abs(extended_pos[2] - retracted_pos[2]) < 1.0e-6,
        details=f"retracted={retracted_pos}, extended={extended_pos}",
    )

    with ctx.pose({slew: 0.95, slide: 0.165}):
        swept_pos = ctx.part_world_position(ram)
    ctx.check(
        "base revolute slews the carried ram",
        retracted_pos is not None
        and swept_pos is not None
        and swept_pos[1] > retracted_pos[1] + 0.10,
        details=f"retracted={retracted_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
