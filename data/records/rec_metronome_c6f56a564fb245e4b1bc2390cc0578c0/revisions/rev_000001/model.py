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
    model = ArticulatedObject(name="pyramid_metronome")

    wood = model.material("wood", rgba=(0.42, 0.24, 0.10, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.23, 0.12, 0.05, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.66, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")

    body.visual(
        Box((0.156, 0.098, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=wood_dark,
        name="base_lower",
    )
    body.visual(
        Box((0.132, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=wood,
        name="base_middle",
    )
    body.visual(
        Box((0.118, 0.074, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=wood,
        name="base_upper",
    )

    side_tilt = math.atan((0.054 - 0.014) / 0.186)
    face_tilt = math.atan((0.034 - 0.012) / 0.186)

    body.visual(
        Box((0.006, 0.070, 0.188)),
        origin=Origin(xyz=(-0.034, 0.0, 0.123), rpy=(0.0, side_tilt, 0.0)),
        material=wood,
        name="left_side",
    )
    body.visual(
        Box((0.006, 0.070, 0.188)),
        origin=Origin(xyz=(0.034, 0.0, 0.123), rpy=(0.0, -side_tilt, 0.0)),
        material=wood,
        name="right_side",
    )
    body.visual(
        Box((0.082, 0.006, 0.188)),
        origin=Origin(xyz=(0.0, 0.022, 0.123), rpy=(face_tilt, 0.0, 0.0)),
        material=wood,
        name="rear_panel",
    )

    body.visual(
        Box((0.022, 0.006, 0.152)),
        origin=Origin(xyz=(-0.023, -0.024, 0.111), rpy=(-face_tilt, 0.0, 0.0)),
        material=wood,
        name="front_left_stile",
    )
    body.visual(
        Box((0.022, 0.006, 0.152)),
        origin=Origin(xyz=(0.023, -0.024, 0.111), rpy=(-face_tilt, 0.0, 0.0)),
        material=wood,
        name="front_right_stile",
    )
    body.visual(
        Box((0.070, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, 0.041), rpy=(-face_tilt, 0.0, 0.0)),
        material=wood,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.016, 0.002, 0.122)),
        origin=Origin(xyz=(0.0, -0.017, 0.110), rpy=(-face_tilt, 0.0, 0.0)),
        material=brass,
        name="scale_plate",
    )
    body.visual(
        Box((0.010, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.024, 0.046)),
        material=brass,
        name="scale_lower_mount",
    )
    body.visual(
        Box((0.008, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, -0.012, 0.187)),
        material=brass,
        name="scale_upper_mount",
    )
    body.visual(
        Box((0.046, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.207)),
        material=wood_dark,
        name="apex_block",
    )
    body.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(-0.010, -0.016, 0.209)),
        material=wood,
        name="left_slot_cheek",
    )
    body.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, -0.016, 0.209)),
        material=wood,
        name="right_slot_cheek",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.024),
        origin=Origin(xyz=(0.0, -0.021, 0.207), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.156, 0.098, 0.222)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0014, length=0.178),
        origin=Origin(xyz=(0.0, -0.014, -0.089)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Box((0.008, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.014, -0.017)),
        material=black,
        name="rod_head",
    )
    pendulum.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, -0.014, -0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lower_bob",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.178)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.014, -0.089)),
    )

    slider = model.part("tempo_weight")
    slider.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tempo_disc",
    )
    slider.visual(
        Box((0.010, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="weight_clamp",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.024, 0.012, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="key_shaft",
    )
    key.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_collar",
    )
    key.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_grip",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.022, 0.044, 0.022)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )

    pendulum_joint = model.articulation(
        "body_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.021, 0.207)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=1.8,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "pendulum_to_tempo_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider,
        origin=Origin(xyz=(0.0, -0.014, -0.040)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "body_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=key,
        origin=Origin(xyz=(0.0, 0.034, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pendulum = object_model.get_part("pendulum")
    slider = object_model.get_part("tempo_weight")
    key = object_model.get_part("winding_key")
    pendulum_joint = object_model.get_articulation("body_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_tempo_weight")

    ctx.expect_gap(
        body,
        pendulum,
        axis="y",
        positive_elem="scale_plate",
        negative_elem="pendulum_rod",
        min_gap=0.008,
        max_gap=0.020,
        name="pendulum rod runs just in front of the front scale",
    )
    ctx.expect_overlap(
        slider,
        pendulum,
        axes="xy",
        elem_a="weight_clamp",
        elem_b="pendulum_rod",
        min_overlap=0.002,
        name="tempo weight clamp stays centered on the rod",
    )

    with ctx.pose({slider_joint: 0.0, pendulum_joint: 0.0}):
        ctx.expect_overlap(
            slider,
            pendulum,
            axes="z",
            elem_a="weight_clamp",
            elem_b="pendulum_rod",
            min_overlap=0.010,
            name="upper tempo setting still captures the rod",
        )
        rest_weight_pos = ctx.part_world_position(slider)

    with ctx.pose({slider_joint: 0.080, pendulum_joint: 0.0}):
        ctx.expect_overlap(
            slider,
            pendulum,
            axes="z",
            elem_a="weight_clamp",
            elem_b="pendulum_rod",
            min_overlap=0.010,
            name="lower tempo setting still captures the rod",
        )
        low_weight_pos = ctx.part_world_position(slider)

    ctx.check(
        "tempo weight slides downward along the pendulum rod",
        rest_weight_pos is not None
        and low_weight_pos is not None
        and abs(low_weight_pos[0] - rest_weight_pos[0]) < 1e-6
        and abs(low_weight_pos[1] - rest_weight_pos[1]) < 1e-6
        and low_weight_pos[2] < rest_weight_pos[2] - 0.06,
        details=f"rest={rest_weight_pos}, low={low_weight_pos}",
    )

    with ctx.pose({slider_joint: 0.040, pendulum_joint: 0.0}):
        centered_weight_pos = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: 0.040, pendulum_joint: 0.30}):
        swung_weight_pos = ctx.part_world_position(slider)

    ctx.check(
        "pendulum swings laterally from the apex pivot",
        centered_weight_pos is not None
        and swung_weight_pos is not None
        and swung_weight_pos[0] < centered_weight_pos[0] - 0.02,
        details=f"centered={centered_weight_pos}, swung={swung_weight_pos}",
    )

    ctx.expect_gap(
        key,
        body,
        axis="y",
        positive_elem="key_grip",
        negative_elem="rear_panel",
        min_gap=0.010,
        max_gap=0.030,
        name="rear winding key protrudes beyond the rear panel",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
