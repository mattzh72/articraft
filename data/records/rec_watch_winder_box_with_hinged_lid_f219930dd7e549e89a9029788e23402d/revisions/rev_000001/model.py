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
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.28, 0.17, 0.10, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.18, 0.11, 0.07, 1.0))
    suede = model.material("suede", rgba=(0.42, 0.31, 0.22, 1.0))
    cushion = model.material("cushion", rgba=(0.20, 0.14, 0.10, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.64, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.51, 0.30, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.77, 0.82, 0.28))

    body_width = 0.180
    body_depth = 0.145
    body_height = 0.100
    wall_t = 0.008
    front_wall_t = 0.012
    base_t = 0.010
    liner_t = 0.003
    cradle_axis_z = 0.047

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=walnut,
        name="shell_base",
    )
    body.visual(
        Box((wall_t, body_depth, body_height - base_t)),
        origin=Origin(
            xyz=(-body_width / 2.0 + wall_t / 2.0, 0.0, base_t + (body_height - base_t) / 2.0)
        ),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_depth, body_height - base_t)),
        origin=Origin(
            xyz=(body_width / 2.0 - wall_t / 2.0, 0.0, base_t + (body_height - base_t) / 2.0)
        ),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_t, wall_t, body_height - base_t)),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + wall_t / 2.0, base_t + (body_height - base_t) / 2.0)
        ),
        material=walnut_dark,
        name="back_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_t, front_wall_t, body_height - base_t)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - front_wall_t / 2.0,
                base_t + (body_height - base_t) / 2.0,
            )
        ),
        material=walnut_dark,
        name="front_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_t - 0.010, body_depth - 2.0 * wall_t - 0.020, liner_t)),
        origin=Origin(
            xyz=(0.0, 0.004, base_t + liner_t / 2.0)
        ),
        material=suede,
        name="base_liner",
    )
    body.visual(
        Box((body_width - 2.0 * wall_t - 0.010, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.046, base_t + 0.020)),
        material=suede,
        name="front_inner_pad",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + wall_t + 0.006, cradle_axis_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_mount",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + wall_t + 0.014, cradle_axis_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="spindle_bushing",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid_frame_t = 0.009
    frame_band = 0.014
    glass_overlap = 0.004
    lid.visual(
        Box((body_width, frame_band, lid_frame_t)),
        origin=Origin(xyz=(0.0, frame_band / 2.0, lid_frame_t / 2.0)),
        material=walnut_dark,
        name="rear_frame",
    )
    lid.visual(
        Box((body_width, frame_band, lid_frame_t)),
        origin=Origin(xyz=(0.0, body_depth - frame_band / 2.0, lid_frame_t / 2.0)),
        material=walnut_dark,
        name="front_frame",
    )
    lid.visual(
        Box((frame_band, body_depth - frame_band, lid_frame_t)),
        origin=Origin(
            xyz=(-body_width / 2.0 + frame_band / 2.0, body_depth / 2.0, lid_frame_t / 2.0)
        ),
        material=walnut_dark,
        name="left_frame",
    )
    lid.visual(
        Box((frame_band, body_depth - frame_band, lid_frame_t)),
        origin=Origin(
            xyz=(body_width / 2.0 - frame_band / 2.0, body_depth / 2.0, lid_frame_t / 2.0)
        ),
        material=walnut_dark,
        name="right_frame",
    )
    lid.visual(
        Box(
            (
                body_width - 2.0 * frame_band + glass_overlap,
                body_depth - 2.0 * frame_band + glass_overlap,
                0.003,
            )
        ),
        origin=Origin(xyz=(0.0, body_depth / 2.0, 0.0015)),
        material=glass,
        name="glass_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, lid_frame_t)),
        mass=0.45,
        origin=Origin(xyz=(0.0, body_depth / 2.0, lid_frame_t / 2.0)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="spindle",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hub_collar",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rotor_plate",
    )
    cradle.visual(
        Box((0.030, 0.020, 0.042)),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material=cushion,
        name="cushion_saddle",
    )
    cradle.visual(
        Box((0.054, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, 0.056, 0.0)),
        material=cushion,
        name="cushion_core",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.054),
        origin=Origin(xyz=(0.0, 0.056, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion,
        name="cushion_top_roll",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.054),
        origin=Origin(xyz=(0.0, 0.056, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion,
        name="cushion_bottom_roll",
    )
    cradle.visual(
        Box((0.014, 0.034, 0.066)),
        origin=Origin(xyz=(0.0, 0.057, 0.0)),
        material=suede,
        name="retaining_band",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.072, 0.090, 0.070)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -body_depth / 2.0 + wall_t + 0.012, cradle_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid seats flush on the box body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.11,
            name="lid covers the compact body opening",
        )
        ctx.expect_contact(
            body,
            cradle,
            elem_a="spindle_bushing",
            elem_b="spindle",
            name="cradle spindle is mounted on the rear bushing",
        )

    ctx.expect_within(
        cradle,
        body,
        axes="xyz",
        margin=0.0,
        name="cradle stays within the presentation box envelope",
    )

    closed_front = None
    open_front = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.04,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
