from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    frame_paint = model.material("frame_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    track_steel = model.material("track_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.58, 0.61, 0.64, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    frame = model.part("frame_support")
    frame.visual(
        Box((0.09, 0.08, 2.16)),
        origin=Origin(xyz=(0.045, -0.07, 1.08)),
        material=frame_paint,
        name="receiver_post",
    )
    frame.visual(
        Box((0.09, 0.08, 2.16)),
        origin=Origin(xyz=(1.62, -0.07, 1.08)),
        material=frame_paint,
        name="main_support_post",
    )
    frame.visual(
        Box((0.09, 0.08, 2.16)),
        origin=Origin(xyz=(2.78, -0.07, 1.08)),
        material=frame_paint,
        name="stack_post",
    )
    frame.visual(
        Box((2.825, 0.08, 0.08)),
        origin=Origin(xyz=(1.4125, -0.07, 2.12)),
        material=frame_paint,
        name="header_beam",
    )
    frame.visual(
        Box((1.205, 0.06, 0.07)),
        origin=Origin(xyz=(2.20, -0.07, 0.33)),
        material=frame_paint,
        name="stack_zone_lower_rail",
    )
    frame.visual(
        Box((2.82, 0.10, 0.016)),
        origin=Origin(xyz=(1.41, 0.0, 2.06)),
        material=track_steel,
        name="track_top",
    )
    frame.visual(
        Box((2.82, 0.012, 0.088)),
        origin=Origin(xyz=(1.41, -0.044, 2.008)),
        material=track_steel,
        name="track_left_flange",
    )
    frame.visual(
        Box((2.82, 0.012, 0.088)),
        origin=Origin(xyz=(1.41, 0.044, 2.008)),
        material=track_steel,
        name="track_right_flange",
    )
    for index, x_pos in enumerate((0.34, 1.42, 2.46), start=1):
        frame.visual(
            Box((0.07, 0.13, 0.064)),
            origin=Origin(xyz=(x_pos, -0.005, 2.099)),
            material=frame_paint,
            name=f"track_bracket_{index}",
        )
    frame.visual(
        Box((0.05, 0.06, 0.26)),
        origin=Origin(xyz=(0.1175, -0.03, 1.905)),
        material=track_steel,
        name="receiver_stop",
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.87, 0.18, 2.20)),
        mass=140.0,
        origin=Origin(xyz=(1.435, -0.03, 1.10)),
    )

    leaf = model.part("gate_leaf")
    leaf.visual(
        Box((1.42, 0.035, 0.05)),
        origin=Origin(xyz=(0.71, 0.0, 0.025)),
        material=gate_paint,
        name="bottom_rail",
    )
    leaf.visual(
        Box((1.42, 0.035, 0.05)),
        origin=Origin(xyz=(0.71, 0.0, 1.725)),
        material=gate_paint,
        name="top_rail",
    )
    leaf.visual(
        Box((0.05, 0.035, 1.75)),
        origin=Origin(xyz=(0.025, 0.0, 0.875)),
        material=gate_paint,
        name="left_stile",
    )
    leaf.visual(
        Box((0.05, 0.035, 1.75)),
        origin=Origin(xyz=(1.395, 0.0, 0.875)),
        material=gate_paint,
        name="right_stile",
    )
    leaf.visual(
        Box((1.32, 0.025, 0.04)),
        origin=Origin(xyz=(0.71, 0.0, 0.93)),
        material=gate_paint,
        name="mid_rail",
    )
    for index, x_pos in enumerate((0.16, 0.31, 0.46, 0.61, 0.76, 0.91, 1.06, 1.21, 1.26), start=1):
        leaf.visual(
            Box((0.02, 0.015, 1.65)),
            origin=Origin(xyz=(x_pos, 0.0, 0.875)),
            material=gate_paint,
            name=f"picket_{index}",
        )
    leaf.visual(
        Box((0.045, 0.01, 0.23)),
        origin=Origin(xyz=(0.37, 0.0, 1.865)),
        material=gate_paint,
        name="front_hanger_strap",
    )
    leaf.visual(
        Box((0.045, 0.01, 0.23)),
        origin=Origin(xyz=(1.05, 0.0, 1.865)),
        material=gate_paint,
        name="rear_hanger_strap",
    )
    leaf.visual(
        Box((0.10, 0.018, 0.03)),
        origin=Origin(xyz=(0.37, 0.0, 1.995)),
        material=track_steel,
        name="front_carriage",
    )
    leaf.visual(
        Box((0.10, 0.018, 0.03)),
        origin=Origin(xyz=(1.05, 0.0, 1.995)),
        material=track_steel,
        name="rear_carriage",
    )
    for wheel_name, x_pos in (
        ("front_left_wheel", 0.34),
        ("front_right_wheel", 0.40),
        ("rear_left_wheel", 1.02),
        ("rear_right_wheel", 1.08),
    ):
        leaf.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(x_pos, 0.0, 2.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name=wheel_name,
        )
    leaf.inertial = Inertial.from_geometry(
        Box((1.42, 0.05, 2.04)),
        mass=58.0,
        origin=Origin(xyz=(0.71, 0.0, 1.02)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.115, 0.0, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.55,
            lower=0.0,
            upper=1.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_support")
    leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_leaf")

    ctx.expect_gap(
        leaf,
        frame,
        axis="x",
        positive_elem="left_stile",
        negative_elem="receiver_post",
        min_gap=0.015,
        max_gap=0.05,
        name="closed leaf sits just clear of the receiver post",
    )
    ctx.expect_contact(
        frame,
        leaf,
        elem_a="track_top",
        elem_b="front_left_wheel",
        contact_tol=1e-5,
        name="front trolley wheel contacts the running surface of the top track",
    )
    ctx.expect_contact(
        frame,
        leaf,
        elem_a="track_top",
        elem_b="rear_right_wheel",
        contact_tol=1e-5,
        name="rear trolley wheel also contacts the track running surface",
    )
    ctx.expect_overlap(
        frame,
        leaf,
        axes="x",
        elem_a="track_top",
        elem_b="rear_carriage",
        min_overlap=0.10,
        name="rear carriage remains under the exposed track in the closed pose",
    )

    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 1.12}):
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="stack_post",
            negative_elem="right_stile",
            min_gap=0.05,
            max_gap=0.12,
            name="opened leaf stops before the stack-side post",
        )
        ctx.expect_overlap(
            frame,
            leaf,
            axes="x",
            elem_a="track_top",
            elem_b="front_carriage",
            min_overlap=0.10,
            name="front carriage still reads as captured in the track at full open",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="track_top",
            elem_b="rear_right_wheel",
            contact_tol=1e-5,
            name="rear trolley wheel stays in contact with the track at full extension",
        )
        open_pos = ctx.part_world_position(leaf)

    ctx.check(
        "gate leaf slides to the right when opened",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, open={open_pos}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
