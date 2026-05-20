from __future__ import annotations

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
    model = ArticulatedObject(name="wide_inspection_portal")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.22, 0.26, 1.0))
    dark_rail = Material("blackened_linear_rail", rgba=(0.03, 0.035, 0.04, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    anodized = Material("anodized_carriage", rgba=(0.56, 0.61, 0.65, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    optic_red = Material("red_optic", rgba=(0.85, 0.05, 0.03, 1.0))

    frame = model.part("portal_frame")
    # A 1.8 m wide gantry portal built from rectangular hollow-section members.
    for x in (-0.90, 0.90):
        frame.visual(
            Box((0.10, 0.08, 1.22)),
            origin=Origin(xyz=(x, -0.26, 0.61)),
            material=painted_steel,
            name=f"front_post_{x:+.1f}",
        )
        frame.visual(
            Box((0.10, 0.08, 1.22)),
            origin=Origin(xyz=(x, 0.26, 0.61)),
            material=painted_steel,
            name=f"rear_post_{x:+.1f}",
        )
        frame.visual(
            Box((0.12, 0.64, 0.10)),
            origin=Origin(xyz=(x, 0.0, 1.19)),
            material=painted_steel,
            name=f"side_top_rail_{x:+.1f}",
        )
        frame.visual(
            Box((0.14, 0.72, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.04)),
            material=painted_steel,
            name=f"side_foot_{x:+.1f}",
        )

    frame.visual(
        Box((1.88, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=painted_steel,
        name="crossbeam_box",
    )
    frame.visual(
        Box((1.66, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=dark_rail,
        name="guide_rail",
    )
    frame.visual(
        Box((1.78, 0.022, 0.035)),
        origin=Origin(xyz=(0.0, -0.101, 1.265)),
        material=dark_rail,
        name="front_beam_seam",
    )
    frame.visual(
        Box((1.78, 0.022, 0.035)),
        origin=Origin(xyz=(0.0, 0.101, 1.265)),
        material=dark_rail,
        name="rear_beam_seam",
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.22, 0.22, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=safety_yellow,
        name="shuttle_body",
    )
    shuttle.visual(
        Box((0.20, 0.036, 0.21)),
        origin=Origin(xyz=(0.0, -0.058, -0.010)),
        material=safety_yellow,
        name="rail_cheek_front",
    )
    shuttle.visual(
        Box((0.20, 0.036, 0.21)),
        origin=Origin(xyz=(0.0, 0.058, -0.010)),
        material=safety_yellow,
        name="rail_cheek_rear",
    )
    for x in (-0.065, 0.065):
        for y in (-0.055, 0.055):
            shuttle.visual(
                Cylinder(radius=0.018, length=0.020),
                origin=Origin(xyz=(x, y, 0.015), rpy=(1.5708, 0.0, 0.0)),
                material=rubber,
                name=f"guide_roller_{x:+.2f}_{y:+.2f}",
            )
    shuttle.visual(
        Box((0.10, 0.030, 0.34)),
        origin=Origin(xyz=(0.0, -0.120, -0.230)),
        material=dark_rail,
        name="vertical_rail",
    )
    shuttle.visual(
        Box((0.14, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, -0.105, -0.405)),
        material=safety_yellow,
        name="lower_stop_block",
    )

    carriage = model.part("tool_carriage")
    carriage.visual(
        Box((0.12, 0.035, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=anodized,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.024, 0.064, 0.145)),
        origin=Origin(xyz=(-0.062, 0.023, -0.085)),
        material=anodized,
        name="side_gib_0",
    )
    carriage.visual(
        Box((0.024, 0.064, 0.145)),
        origin=Origin(xyz=(0.062, 0.023, -0.085)),
        material=anodized,
        name="side_gib_1",
    )
    carriage.visual(
        Box((0.07, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, -0.002, -0.182)),
        material=dark_rail,
        name="tool_holder",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.115),
        origin=Origin(xyz=(0.0, -0.002, -0.245)),
        material=dark_rail,
        name="inspection_probe",
    )
    carriage.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, -0.002, -0.3075)),
        material=optic_red,
        name="probe_lens",
    )

    model.articulation(
        "frame_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shuttle,
        origin=Origin(xyz=(-0.60, 0.0, 1.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=1.20),
    )
    model.articulation(
        "shuttle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.165, -0.115)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("portal_frame")
    shuttle = object_model.get_part("shuttle")
    carriage = object_model.get_part("tool_carriage")
    shuttle_slide = object_model.get_articulation("frame_to_shuttle")
    vertical_slide = object_model.get_articulation("shuttle_to_carriage")

    ctx.expect_gap(
        frame,
        shuttle,
        axis="z",
        positive_elem="guide_rail",
        negative_elem="shuttle_body",
        min_gap=0.015,
        max_gap=0.041,
        name="shuttle hangs just below the guide rail",
    )
    ctx.expect_overlap(
        shuttle,
        frame,
        axes="xz",
        elem_a="rail_cheek_front",
        elem_b="guide_rail",
        min_overlap=0.06,
        name="front cheek wraps the rail in profile",
    )
    ctx.expect_overlap(
        carriage,
        shuttle,
        axes="yz",
        elem_a="side_gib_1",
        elem_b="vertical_rail",
        min_overlap=0.02,
        name="carriage gib stays engaged with the vertical rail",
    )
    ctx.expect_gap(
        carriage,
        shuttle,
        axis="x",
        positive_elem="side_gib_1",
        negative_elem="vertical_rail",
        min_gap=-0.000001,
        max_gap=0.001,
        name="carriage gib rides on the rail side face",
    )
    ctx.expect_contact(
        shuttle,
        frame,
        elem_a="rail_cheek_front",
        elem_b="guide_rail",
        contact_tol=0.001,
        name="front cheek bears against the guide rail",
    )

    rest_shuttle = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: 1.20}):
        far_shuttle = ctx.part_world_position(shuttle)
        ctx.expect_within(
            shuttle,
            frame,
            axes="x",
            inner_elem="shuttle_body",
            outer_elem="crossbeam_box",
            margin=0.08,
            name="shuttle travel remains under the crossbeam",
        )
    ctx.check(
        "shuttle translates across the beam",
        rest_shuttle is not None
        and far_shuttle is not None
        and far_shuttle[0] > rest_shuttle[0] + 1.0,
        details=f"rest={rest_shuttle}, far={far_shuttle}",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({vertical_slide: 0.22}):
        lowered_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            shuttle,
            axes="z",
            elem_a="side_gib_1",
            elem_b="vertical_rail",
            min_overlap=0.05,
            name="lowered carriage remains captured on the vertical rail",
        )
    ctx.check(
        "tool carriage translates vertically downward",
        rest_carriage is not None
        and lowered_carriage is not None
        and lowered_carriage[2] < rest_carriage[2] - 0.18,
        details=f"rest={rest_carriage}, lowered={lowered_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
