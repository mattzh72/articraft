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

    wood = model.material("wood", rgba=(0.27, 0.17, 0.10, 1.0))
    wood_trim = model.material("wood_trim", rgba=(0.16, 0.10, 0.06, 1.0))
    lining = model.material("lining", rgba=(0.78, 0.72, 0.63, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.80, 0.86, 0.28))
    brass = model.material("brass", rgba=(0.73, 0.62, 0.34, 1.0))

    outer_w = 0.23
    outer_d = 0.18
    body_h = 0.125
    lid_h = 0.055
    wall = 0.012
    bottom_t = 0.012
    lid_wall = 0.010
    lid_top_t = 0.008
    hinge_radius = 0.0045
    rear_y = outer_d * 0.5

    body = model.part("body")

    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=wood,
        name="base_bottom",
    )
    body.visual(
        Box((outer_w, wall, body_h)),
        origin=Origin(xyz=(0.0, -(outer_d - wall) * 0.5, body_h * 0.5)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_w, wall, body_h - 0.012)),
        origin=Origin(
            xyz=(0.0, (outer_d - wall) * 0.5, (body_h - 0.012) * 0.5),
        ),
        material=wood,
        name="rear_wall_lower",
    )
    body.visual(
        Box((0.076, wall, 0.018)),
        origin=Origin(xyz=(-0.077, (outer_d - wall) * 0.5, body_h - 0.009)),
        material=wood,
        name="rear_left_shoulder",
    )
    body.visual(
        Box((0.076, wall, 0.018)),
        origin=Origin(xyz=(0.077, (outer_d - wall) * 0.5, body_h - 0.009)),
        material=wood,
        name="rear_right_shoulder",
    )
    body.visual(
        Box((wall, outer_d - 2.0 * wall, body_h)),
        origin=Origin(
            xyz=(-(outer_w - wall) * 0.5, 0.0, body_h * 0.5),
        ),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall, outer_d - 2.0 * wall, body_h)),
        origin=Origin(
            xyz=((outer_w - wall) * 0.5, 0.0, body_h * 0.5),
        ),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall, outer_d - 2.0 * wall, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.002)),
        material=wood_trim,
        name="top_rim",
    )
    body.visual(
        Box((outer_w - 2.0 * wall - 0.006, outer_d - 2.0 * wall - 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.003)),
        material=lining,
        name="floor_pad",
    )
    body.visual(
        Box((outer_w - 2.0 * wall - 0.006, 0.008, body_h - bottom_t - 0.010)),
        origin=Origin(
            xyz=(0.0, (outer_d * 0.5) - wall - 0.004, body_h * 0.5 + 0.001),
        ),
        material=lining,
        name="rear_lining",
    )
    spindle_axis_z = 0.072
    spindle_axis_y = rear_y - wall - 0.010
    body.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(
            xyz=(0.0, spindle_axis_y + 0.004, spindle_axis_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_bushing",
    )

    hinge_barrel_len = 0.040
    hinge_x = 0.060
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_barrel_len),
        origin=Origin(
            xyz=(-hinge_x, rear_y - wall * 0.15, body_h + hinge_radius * 0.75),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_barrel_len),
        origin=Origin(
            xyz=(hinge_x, rear_y - wall * 0.15, body_h + hinge_radius * 0.75),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="right_hinge_barrel",
    )

    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h + 0.02)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, (body_h + 0.02) * 0.5)),
    )

    lid = model.part("lid")
    window_w = outer_w - 0.060
    window_d = outer_d - 0.072
    frame_band = 0.018
    top_z = lid_h - lid_top_t * 0.5 - 0.007
    lid.visual(
        Box((outer_w, frame_band, lid_top_t)),
        origin=Origin(xyz=(0.0, -frame_band * 0.5, top_z)),
        material=wood,
        name="top_frame_back",
    )
    lid.visual(
        Box((outer_w, frame_band, lid_top_t)),
        origin=Origin(xyz=(0.0, -outer_d + frame_band * 0.5, top_z)),
        material=wood,
        name="top_frame_front",
    )
    lid.visual(
        Box((frame_band, window_d, lid_top_t)),
        origin=Origin(xyz=(-(outer_w - frame_band) * 0.5, -outer_d * 0.5, top_z)),
        material=wood,
        name="top_frame_left",
    )
    lid.visual(
        Box((frame_band, window_d, lid_top_t)),
        origin=Origin(xyz=((outer_w - frame_band) * 0.5, -outer_d * 0.5, top_z)),
        material=wood,
        name="top_frame_right",
    )
    lid.visual(
        Box((outer_w - 0.024, outer_d - 0.024, 0.003)),
        origin=Origin(
            xyz=(0.0, -outer_d * 0.5, lid_h - lid_top_t * 0.5 - 0.0085),
        ),
        material=glass,
        name="window_glass",
    )
    lid.visual(
        Box((0.070, lid_wall, lid_h - 0.010)),
        origin=Origin(xyz=(0.0, -lid_wall * 0.5, 0.019125)),
        material=wood,
        name="rear_rail",
    )
    lid.visual(
        Box((outer_w, lid_wall, lid_h - 0.006)),
        origin=Origin(xyz=(0.0, -outer_d + lid_wall * 0.5, 0.021125)),
        material=wood,
        name="front_rail",
    )
    lid.visual(
        Box((lid_wall, outer_d - 2.0 * lid_wall, lid_h - 0.008)),
        origin=Origin(xyz=(-(outer_w - lid_wall) * 0.5, -outer_d * 0.5, 0.020125)),
        material=wood,
        name="left_rail",
    )
    lid.visual(
        Box((lid_wall, outer_d - 2.0 * lid_wall, lid_h - 0.008)),
        origin=Origin(xyz=((outer_w - lid_wall) * 0.5, -outer_d * 0.5, 0.020125)),
        material=wood,
        name="right_rail",
    )
    lid.visual(
        Box((outer_w - 0.020, outer_d - 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -outer_d * 0.5, 0.001)),
        material=lining,
        name="lid_lining",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.050),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="center_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, lid_h)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -outer_d * 0.5, lid_h * 0.5 - 0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(0.0, rear_y - wall * 0.15, body_h + hinge_radius * 0.75),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="spindle_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.005, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=wood_trim,
        name="drive_hub",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.016, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=lining,
        name="rear_collar",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(
            xyz=(0.0, -0.046, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=lining,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.056, 0.012, 0.056)),
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=lining,
        name="cushion_core",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.080)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, spindle_axis_y, spindle_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        min_overlap=0.18,
        name="lid spans the body width",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_rail",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid front edge seats on the box body",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        margin=0.0,
        name="cradle stays inside the presentation cavity footprint",
    )
    ctx.check(
        "cradle spins on a front-to-back spindle",
        cradle_spin.articulation_type == ArticulationType.CONTINUOUS
        and cradle_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={cradle_spin.articulation_type}, axis={cradle_spin.axis}",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({lid_hinge: math.radians(95.0)}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_rail")
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.0,
            name="cradle remains nested while the lid is opened",
        )

    ctx.check(
        "lid opens upward",
        closed_front is not None
        and opened_front is not None
        and opened_front[0][2] > closed_front[0][2] + 0.09,
        details=f"closed_front={closed_front}, opened_front={opened_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
