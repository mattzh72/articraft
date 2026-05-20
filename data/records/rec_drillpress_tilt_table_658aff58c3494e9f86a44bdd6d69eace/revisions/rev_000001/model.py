from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _radial_arm_shape() -> cq.Workplane:
    """Hollow column sleeve fused to a long boxed radial arm, in the arm frame."""
    sleeve_outer_radius = 0.170
    # The bearing bore is modeled as a lightly captured bushing around the
    # column.  A tiny hidden interference is allowed in tests as a shaft fit.
    sleeve_inner_radius = 0.105
    sleeve_height = 0.260
    arm_length = 1.120
    arm_width = 0.145
    arm_height = 0.125

    sleeve = (
        cq.Workplane("XY")
        .circle(sleeve_outer_radius)
        .circle(sleeve_inner_radius)
        .extrude(sleeve_height)
        .translate((0.0, 0.0, -sleeve_height / 2.0))
    )
    beam = cq.Workplane("XY").box(arm_length, arm_width, arm_height).translate(
        (0.135 + arm_length / 2.0, 0.0, 0.0)
    )
    nose_cap = cq.Workplane("XY").box(0.060, 0.185, 0.155).translate(
        (0.135 + arm_length + 0.020, 0.0, 0.0)
    )
    return sleeve.union(beam).union(nose_cap)


def _head_carriage_shape() -> cq.Workplane:
    """Sliding drill head with a rectangular arm tunnel and vertical quill bore."""
    sleeve = cq.Workplane("XY").box(0.340, 0.300, 0.280)
    tunnel = cq.Workplane("XY").box(0.390, 0.190, 0.172)
    sleeve = sleeve.cut(tunnel)

    gearbox = cq.Workplane("XY").box(0.245, 0.245, 0.420).translate(
        (0.055, 0.0, -0.320)
    )
    throat = cq.Workplane("XY").box(0.165, 0.190, 0.130).translate(
        (0.060, 0.0, -0.125)
    )
    shape = sleeve.union(gearbox).union(throat)

    quill_bore = (
        cq.Workplane("XY")
        .circle(0.052)
        .extrude(0.720)
        .translate((0.055, 0.0, -0.650))
    )
    return shape.cut(quill_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    cast_iron = model.material("cast_iron_blue_gray", rgba=(0.23, 0.29, 0.32, 1.0))
    dark_casting = model.material("dark_cast_iron", rgba=(0.12, 0.14, 0.15, 1.0))
    machined = model.material("machined_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("blackened_slots", rgba=(0.025, 0.025, 0.025, 1.0))
    warning = model.material("brass_control_caps", rgba=(0.78, 0.62, 0.30, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.98, 0.64, 0.105)),
        origin=Origin(xyz=(0.48, 0.0, 0.0525)),
        material=dark_casting,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.095, length=1.62),
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        material=cast_iron,
        name="round_column",
    )
    base.visual(
        Cylinder(radius=0.125, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.760)),
        material=dark_casting,
        name="column_cap",
    )
    base.visual(
        Box((0.745, 0.125, 0.080)),
        origin=Origin(xyz=(0.435, 0.0, 0.405)),
        material=cast_iron,
        name="table_support",
    )
    base.visual(
        Cylinder(radius=0.355, length=0.055),
        origin=Origin(xyz=(0.755, 0.0, 0.4575)),
        material=cast_iron,
        name="round_table",
    )
    for slot_y in (-0.105, 0.105):
        base.visual(
            Box((0.560, 0.024, 0.006)),
            origin=Origin(xyz=(0.755, slot_y, 0.484)),
            material=black,
            name=f"table_t_slot_{0 if slot_y < 0 else 1}",
        )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_radial_arm_shape(), "radial_arm_casting"),
        material=cast_iron,
        name="radial_arm_casting",
    )
    arm.visual(
        Box((0.940, 0.018, 0.018)),
        origin=Origin(xyz=(0.675, -0.068, 0.071)),
        material=machined,
        name="front_slide_way",
    )
    arm.visual(
        Box((0.940, 0.018, 0.018)),
        origin=Origin(xyz=(0.675, 0.068, 0.071)),
        material=machined,
        name="rear_slide_way",
    )
    for index, (xyz, size) in enumerate(
        (
            ((0.130, 0.0, 0.0), (0.070, 0.028, 0.180)),
            ((-0.130, 0.0, 0.0), (0.070, 0.028, 0.180)),
            ((0.0, 0.130, 0.0), (0.028, 0.070, 0.180)),
            ((0.0, -0.130, 0.0), (0.028, 0.070, 0.180)),
        )
    ):
        arm.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=machined,
            name=f"bearing_shoe_{index}",
        )

    model.articulation(
        "column_to_arm",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.7),
    )

    head = model.part("head")
    head.visual(
        Box((0.350, 0.310, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=cast_iron,
        name="sleeve_top",
    )
    for index, rail_y in enumerate((-0.115, 0.115)):
        head.visual(
            Box((0.350, 0.080, 0.055)),
            origin=Origin(xyz=(0.0, rail_y, -0.1375)),
            material=cast_iron,
            name=f"sleeve_bottom_{index}",
        )
    head.visual(
        Box((0.350, 0.055, 0.270)),
        origin=Origin(xyz=(0.0, -0.1325, 0.0)),
        material=cast_iron,
        name="sleeve_cheek_0",
    )
    head.visual(
        Box((0.350, 0.055, 0.270)),
        origin=Origin(xyz=(0.0, 0.1325, 0.0)),
        material=cast_iron,
        name="sleeve_cheek_1",
    )
    for index, throat_y in enumerate((-0.080, 0.080)):
        head.visual(
            Box((0.200, 0.055, 0.060)),
            origin=Origin(xyz=(0.030, throat_y, -0.175)),
            material=cast_iron,
            name=f"gearbox_throat_{index}",
        )
    head.visual(
        Box((0.220, 0.060, 0.410)),
        origin=Origin(xyz=(0.055, -0.112, -0.345)),
        material=cast_iron,
        name="gearbox_cheek_0",
    )
    head.visual(
        Box((0.220, 0.060, 0.410)),
        origin=Origin(xyz=(0.055, 0.112, -0.345)),
        material=cast_iron,
        name="gearbox_cheek_1",
    )
    head.visual(
        Box((0.060, 0.220, 0.410)),
        origin=Origin(xyz=(-0.070, 0.0, -0.345)),
        material=cast_iron,
        name="gearbox_back",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.044, tube=0.010), "quill_guide_ring"),
        origin=Origin(xyz=(0.055, 0.0, -0.300)),
        material=machined,
        name="quill_guide_ring",
    )
    for index, lug_y in enumerate((-0.068, 0.068)):
        head.visual(
            Box((0.070, 0.030, 0.026)),
            origin=Origin(xyz=(0.055, lug_y, -0.300)),
            material=cast_iron,
            name=f"guide_lug_{index}",
        )
    head.visual(
        Box((0.150, 0.180, 0.070)),
        origin=Origin(xyz=(-0.035, 0.0, 0.170)),
        material=dark_casting,
        name="motor_pedestal",
    )
    head.visual(
        Cylinder(radius=0.112, length=0.350),
        origin=Origin(xyz=(-0.040, 0.0, 0.305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_casting,
        name="horizontal_motor",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.055, 0.150, -0.315), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="feed_shaft",
    )
    head.visual(
        Box((0.070, 0.020, 0.048)),
        origin=Origin(xyz=(0.168, -0.137, -0.235)),
        material=warning,
        name="slide_lock_knob",
    )

    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.620, effort=850.0, velocity=0.22),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.035, length=0.415),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=machined,
        name="quill_sleeve",
    )
    quill.visual(
        Cylinder(radius=0.046, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.326)),
        material=dark_casting,
        name="drill_chuck",
    )
    quill.visual(
        Cylinder(radius=0.010, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, -0.4645)),
        material=machined,
        name="drill_bit",
    )

    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.055, 0.0, -0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=420.0, velocity=0.16),
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_casting,
        name="feed_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        dx = math.cos(angle)
        dz = math.sin(angle)
        feed_handle.visual(
            Cylinder(radius=0.010, length=0.165),
            origin=Origin(
                xyz=(0.086 * dx, 0.030, 0.086 * dz),
                rpy=(0.0, math.pi / 2.0 - angle, 0.0),
            ),
            material=machined,
            name=f"feed_spoke_{index}",
        )
        feed_handle.visual(
            Sphere(radius=0.023),
            origin=Origin(xyz=(0.172 * dx, 0.030, 0.172 * dz)),
            material=warning,
            name=f"feed_grip_{index}",
        )

    model.articulation(
        "head_to_feed_handle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(0.055, 0.180, -0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=1.15, effort=18.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")
    arm_sweep = object_model.get_articulation("column_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    quill_drop = object_model.get_articulation("head_to_quill")

    ctx.allow_overlap(
        head,
        quill,
        elem_a="quill_guide_ring",
        elem_b="quill_sleeve",
        reason=(
            "The quill sleeve is intentionally captured in a simplified sliding "
            "guide bushing inside the drill head."
        ),
    )
    ctx.expect_within(
        quill,
        head,
        axes="xy",
        inner_elem="quill_sleeve",
        outer_elem="quill_guide_ring",
        margin=0.0,
        name="quill sleeve is centered in the guide ring",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        min_overlap=0.015,
        elem_a="quill_sleeve",
        elem_b="quill_guide_ring",
        name="quill sleeve is retained in the guide at rest",
    )
    with ctx.pose({quill_drop: 0.180}):
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            min_overlap=0.015,
            elem_a="quill_sleeve",
            elem_b="quill_guide_ring",
            name="quill sleeve remains retained when dropped",
        )

    ctx.expect_overlap(
        head,
        arm,
        axes="x",
        min_overlap=0.25,
        elem_a="sleeve_top",
        elem_b="radial_arm_casting",
        name="head sleeve surrounds the arm at rest",
    )
    with ctx.pose({head_slide: 0.620}):
        ctx.expect_overlap(
            head,
            arm,
            axes="x",
            min_overlap=0.25,
            elem_a="sleeve_top",
            elem_b="radial_arm_casting",
            name="head sleeve stays engaged at full slide",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.620}):
        extended_head_pos = ctx.part_world_position(head)
    ctx.check(
        "drill head slides outward along the radial arm",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] > rest_head_pos[0] + 0.55,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({quill_drop: 0.180}):
        dropped_quill_pos = ctx.part_world_position(quill)
    ctx.check(
        "quill drops along the drill axis",
        rest_quill_pos is not None
        and dropped_quill_pos is not None
        and dropped_quill_pos[2] < rest_quill_pos[2] - 0.15,
        details=f"rest={rest_quill_pos}, dropped={dropped_quill_pos}",
    )

    with ctx.pose({arm_sweep: math.pi / 2.0}):
        swept_head_pos = ctx.part_world_position(head)
    ctx.check(
        "radial arm sweeps the head around the round column",
        swept_head_pos is not None
        and swept_head_pos[1] > 0.30
        and abs(swept_head_pos[0]) < 0.08,
        details=f"swept={swept_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
