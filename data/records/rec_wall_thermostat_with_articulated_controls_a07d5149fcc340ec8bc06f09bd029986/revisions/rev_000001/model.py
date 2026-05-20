from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    housing = model.material("housing", rgba=(0.90, 0.89, 0.84, 1.0))
    plate = model.material("plate", rgba=(0.82, 0.80, 0.75, 1.0))
    dial_plastic = model.material("dial_plastic", rgba=(0.95, 0.94, 0.90, 1.0))
    pointer = model.material("pointer", rgba=(0.37, 0.31, 0.24, 1.0))
    slider_plastic = model.material("slider_plastic", rgba=(0.49, 0.41, 0.30, 1.0))
    internal = model.material("internal", rgba=(0.18, 0.18, 0.18, 1.0))

    plate_width = 0.122
    plate_height = 0.087
    plate_thickness = 0.002

    body_width = 0.108
    body_height = 0.076
    body_depth = 0.022
    body_radius = 0.010

    slot_width = 0.038
    slot_height = 0.008
    slot_radius = 0.003
    slot_y = -0.022

    front_z = plate_thickness + body_depth

    body = model.part("body")

    shell_profile = rounded_rect_profile(body_width, body_height, body_radius)
    slot_profile = _offset_profile(
        rounded_rect_profile(slot_width, slot_height, slot_radius),
        dy=slot_y,
    )
    shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            shell_profile,
            [slot_profile],
            body_depth,
            cap=True,
            center=False,
            closed=True,
        ),
        "thermostat_shell",
    )
    body.visual(
        Box((plate_width, plate_height, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=plate,
        name="wall_plate",
    )
    body.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Box((slot_width, 0.0015, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                slot_y + (slot_height + 0.0015) * 0.5,
                plate_thickness + body_depth * 0.5,
            )
        ),
        material=internal,
        name="guide_upper",
    )
    body.visual(
        Box((slot_width, 0.0015, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                slot_y - (slot_height + 0.0015) * 0.5,
                plate_thickness + body_depth * 0.5,
            )
        ),
        material=internal,
        name="guide_lower",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.016,
                body_style="skirted",
                top_diameter=0.046,
                base_diameter=0.056,
                crown_radius=0.0025,
                edge_radius=0.0012,
                side_draft_deg=6.0,
                center=False,
            ),
            "thermostat_dial",
        ),
        material=dial_plastic,
        name="dial_body",
    )
    dial.visual(
        Box((0.004, 0.018, 0.0015)),
        origin=Origin(xyz=(0.0, 0.013, 0.01675)),
        material=pointer,
        name="pointer",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.016, 0.008, 0.0055)),
        origin=Origin(xyz=(0.0, 0.0, 0.00275)),
        material=slider_plastic,
        name="thumb",
    )
    slider.visual(
        Box((0.008, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=slider_plastic,
        name="stem",
    )
    slider.visual(
        Box((0.018, 0.006, 0.0035)),
        origin=Origin(xyz=(0.0, 0.0, -0.00925)),
        material=slider_plastic,
        name="shuttle",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.014, front_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0),
    )
    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.0, slot_y, front_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=-0.010,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    slider = object_model.get_part("slider")
    dial_joint = object_model.get_articulation("body_to_dial")
    slider_joint = object_model.get_articulation("body_to_slider")

    slider_limits = slider_joint.motion_limits
    lower = -0.010 if slider_limits is None or slider_limits.lower is None else slider_limits.lower
    upper = 0.010 if slider_limits is None or slider_limits.upper is None else slider_limits.upper

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_body",
        negative_elem="housing_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial seats on housing face",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_body",
        elem_b="housing_shell",
        min_overlap=0.045,
        name="dial stays centered on thermostat body",
    )
    ctx.expect_gap(
        dial,
        slider,
        axis="y",
        min_gap=0.0008,
        name="dial clears the mode slider",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.9}):
        ctx.expect_gap(
            dial,
            body,
            axis="z",
            positive_elem="dial_body",
            negative_elem="housing_shell",
            max_gap=0.0005,
            max_penetration=0.0,
            name="dial remains seated while rotated",
        )
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "dial spins about a fixed center",
        dial_rest is not None
        and dial_rotated is not None
        and abs(dial_rest[0] - dial_rotated[0]) < 1e-6
        and abs(dial_rest[1] - dial_rotated[1]) < 1e-6
        and abs(dial_rest[2] - dial_rotated[2]) < 1e-6,
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    ctx.expect_gap(
        slider,
        body,
        axis="z",
        positive_elem="thumb",
        negative_elem="housing_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="slider thumb sits on front slot face",
    )
    ctx.expect_within(
        slider,
        body,
        axes="x",
        inner_elem="shuttle",
        outer_elem="housing_shell",
        margin=0.0005,
        name="slider shuttle stays laterally within the housing",
    )
    ctx.expect_gap(
        body,
        slider,
        axis="y",
        positive_elem="guide_upper",
        negative_elem="shuttle",
        min_gap=0.0005,
        max_gap=0.0015,
        name="slider shuttle clears upper guide rail at center",
    )
    ctx.expect_gap(
        slider,
        body,
        axis="y",
        positive_elem="shuttle",
        negative_elem="guide_lower",
        min_gap=0.0005,
        max_gap=0.0015,
        name="slider shuttle clears lower guide rail at center",
    )

    with ctx.pose({slider_joint: lower}):
        ctx.expect_within(
            slider,
            body,
            axes="x",
            inner_elem="shuttle",
            outer_elem="housing_shell",
            margin=0.0005,
            name="slider shuttle stays within the housing at low mode",
        )
        ctx.expect_gap(
            body,
            slider,
            axis="y",
            positive_elem="guide_upper",
            negative_elem="shuttle",
            min_gap=0.0005,
            max_gap=0.0015,
            name="slider shuttle clears upper guide rail at low mode",
        )
        ctx.expect_gap(
            slider,
            body,
            axis="y",
            positive_elem="shuttle",
            negative_elem="guide_lower",
            min_gap=0.0005,
            max_gap=0.0015,
            name="slider shuttle clears lower guide rail at low mode",
        )
        slider_low = ctx.part_world_position(slider)

    with ctx.pose({slider_joint: upper}):
        ctx.expect_within(
            slider,
            body,
            axes="x",
            inner_elem="shuttle",
            outer_elem="housing_shell",
            margin=0.0005,
            name="slider shuttle stays within the housing at high mode",
        )
        ctx.expect_gap(
            body,
            slider,
            axis="y",
            positive_elem="guide_upper",
            negative_elem="shuttle",
            min_gap=0.0005,
            max_gap=0.0015,
            name="slider shuttle clears upper guide rail at high mode",
        )
        ctx.expect_gap(
            slider,
            body,
            axis="y",
            positive_elem="shuttle",
            negative_elem="guide_lower",
            min_gap=0.0005,
            max_gap=0.0015,
            name="slider shuttle clears lower guide rail at high mode",
        )
        ctx.expect_gap(
            slider,
            body,
            axis="z",
            positive_elem="thumb",
            negative_elem="housing_shell",
            max_gap=0.0005,
            max_penetration=0.0,
            name="slider thumb remains seated at high mode",
        )
        slider_high = ctx.part_world_position(slider)

    ctx.check(
        "slider moves laterally across the slot",
        slider_low is not None
        and slider_high is not None
        and slider_high[0] > slider_low[0] + 0.015
        and abs(slider_high[1] - slider_low[1]) < 1e-6
        and abs(slider_high[2] - slider_low[2]) < 1e-6,
        details=f"low={slider_low}, high={slider_high}",
    )

    return ctx.report()


object_model = build_object_model()
