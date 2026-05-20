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
    model = ArticulatedObject(name="box_metronome")

    wood = model.material("wood", rgba=(0.47, 0.30, 0.18, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.28, 0.18, 0.11, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.65, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    cream = model.material("cream", rgba=(0.92, 0.89, 0.81, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.10, 1.0))

    base_w = 0.160
    base_d = 0.120
    base_t = 0.018

    housing_w = 0.112
    housing_d = 0.082
    housing_h = 0.218
    wall_t = 0.006

    body = model.part("body")
    body.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=dark_wood,
        name="base_plinth",
    )
    body.visual(
        Box((housing_w, housing_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t + wall_t * 0.5)),
        material=wood,
        name="housing_floor",
    )
    body.visual(
        Box((wall_t, housing_d, housing_h)),
        origin=Origin(
            xyz=(-housing_w * 0.5 + wall_t * 0.5, 0.0, base_t + housing_h * 0.5)
        ),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, housing_d, housing_h)),
        origin=Origin(
            xyz=(housing_w * 0.5 - wall_t * 0.5, 0.0, base_t + housing_h * 0.5)
        ),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, housing_h)),
        origin=Origin(
            xyz=(0.0, -housing_d * 0.5 + wall_t * 0.5, base_t + housing_h * 0.5)
        ),
        material=wood,
        name="back_wall",
    )
    body.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, 0.030)),
        origin=Origin(
            xyz=(0.0, housing_d * 0.5 - wall_t * 0.5, base_t + housing_h - 0.015)
        ),
        material=wood,
        name="front_header",
    )
    body.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, 0.028)),
        origin=Origin(
            xyz=(0.0, housing_d * 0.5 - wall_t * 0.5, base_t + 0.014)
        ),
        material=wood,
        name="front_sill",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.043),
        origin=Origin(
            xyz=(-0.0285, 0.024, base_t + housing_h - 0.031),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_pendulum_journal",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.043),
        origin=Origin(
            xyz=(0.0285, 0.024, base_t + housing_h - 0.031),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_pendulum_journal",
    )
    body.visual(
        Box((0.016, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.038, base_t + housing_h - 0.031)),
        material=steel,
        name="pivot_support_block",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.026),
        origin=Origin(
            xyz=(-0.037, -housing_d * 0.5, base_t + housing_h + 0.0055),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.026),
        origin=Origin(
            xyz=(0.037, -housing_d * 0.5, base_t + housing_h + 0.0055),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_t + housing_h)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, (base_t + housing_h) * 0.5)),
    )

    dial_face = model.part("dial_face")
    dial_face.visual(
        Box((0.036, 0.004, 0.150)),
        origin=Origin(xyz=(0.0, 0.028, 0.075)),
        material=cream,
        name="tempo_board",
    )
    dial_face.visual(
        Box((0.028, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.0315, 0.146)),
        material=black,
        name="scale_cap",
    )
    for x_pos, z_pos in ((-0.010, 0.030), (0.010, 0.118)):
        dial_face.visual(
            Cylinder(radius=0.0025, length=0.026),
            origin=Origin(
                xyz=(x_pos, 0.013, z_pos),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
            name=f"standoff_{'upper' if z_pos > 0.1 else 'lower'}_{'right' if x_pos > 0 else 'left'}",
        )
    dial_face.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.156)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.016, 0.078)),
    )
    model.articulation(
        "body_to_dial_face",
        ArticulationType.FIXED,
        parent=body,
        child=dial_face,
        origin=Origin(xyz=(0.0, -housing_d * 0.5 + wall_t, base_t + 0.044)),
    )

    lid = model.part("lid")
    lid_w = 0.118
    lid_d = 0.088
    lid_t = 0.010
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, 0.0055 + lid_d * 0.5, 0.0)),
        material=wood,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.0055, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.006, lid_d - 0.002, 0.018)),
        origin=Origin(xyz=(-0.059, 0.0485, -0.014)),
        material=wood,
        name="left_skirt",
    )
    lid.visual(
        Box((0.006, lid_d - 0.002, 0.018)),
        origin=Origin(xyz=(0.059, 0.0485, -0.014)),
        material=wood,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_w, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.0055 + lid_d - 0.003, -0.015)),
        material=wood,
        name="front_skirt",
    )
    lid.visual(
        Box((0.030, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0695, -0.007)),
        material=black,
        name="key_escutcheon",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.026)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0055 + lid_d * 0.5, -0.008)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -housing_d * 0.5, base_t + housing_h + 0.0055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="key_shaft",
    )
    key.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=brass,
        name="key_hub",
    )
    key.visual(
        Box((0.024, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=brass,
        name="key_arm",
    )
    key.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(
            xyz=(0.014, 0.0, -0.013),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="grip_knob",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.028, 0.012, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.008, 0.0, -0.009)),
    )
    model.articulation(
        "key_turn",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=key,
        origin=Origin(xyz=(0.0, 0.0695, -0.005)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=9.0),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.004, 0.006, 0.176)),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.0, -0.170),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="pointer_bob",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.176)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
    )
    model.articulation(
        "pendulum_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.024, base_t + housing_h - 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-0.25,
            upper=0.25,
        ),
    )

    weight = model.part("slider_weight")
    weight.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.016, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="front_mass",
    )
    weight.visual(
        Box((0.020, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=brass,
        name="rear_clamp",
    )
    weight.visual(
        Box((0.007, 0.018, 0.020)),
        origin=Origin(xyz=(-0.009, 0.003, 0.0)),
        material=brass,
        name="left_cheek",
    )
    weight.visual(
        Box((0.007, 0.018, 0.020)),
        origin=Origin(xyz=(0.009, 0.003, 0.0)),
        material=brass,
        name="right_cheek",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.030, 0.034, 0.024)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=0.10,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    key = object_model.get_part("winding_key")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("slider_weight")

    lid_hinge = object_model.get_articulation("lid_hinge")
    key_turn = object_model.get_articulation("key_turn")
    pendulum_swing = object_model.get_articulation("pendulum_swing")
    weight_slide = object_model.get_articulation("weight_slide")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="back_wall",
        min_gap=0.0,
        max_gap=0.001,
        name="closed lid sits on the housing top",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.08,
        name="closed lid covers the metronome housing",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: 1.0}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid opens upward and rearward",
        closed_front is not None
        and opened_front is not None
        and opened_front[0][2] > closed_front[0][2] + 0.06
        and opened_front[1][1] < closed_front[1][1] - 0.010,
        details=f"closed_front={closed_front}, opened_front={opened_front}",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.090}):
        low_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "pendulum weight slides downward along the rod",
        rest_weight_pos is not None
        and low_weight_pos is not None
        and low_weight_pos[2] < rest_weight_pos[2] - 0.05,
        details=f"rest={rest_weight_pos}, low={low_weight_pos}",
    )

    centered_bob = ctx.part_element_world_aabb(pendulum, elem="pointer_bob")
    with ctx.pose({pendulum_swing: 0.20}):
        swung_bob = ctx.part_element_world_aabb(pendulum, elem="pointer_bob")
    ctx.check(
        "pendulum swings laterally from its upper pivot",
        centered_bob is not None
        and swung_bob is not None
        and swung_bob[0][0] < centered_bob[0][0] - 0.012,
        details=f"centered={centered_bob}, swung={swung_bob}",
    )

    key_rest = ctx.part_element_world_aabb(key, elem="grip_knob")
    with ctx.pose({key_turn: 1.0}):
        key_rotated = ctx.part_element_world_aabb(key, elem="grip_knob")
    ctx.check(
        "winding key rotates in the lid face",
        key_rest is not None
        and key_rotated is not None
        and (
            abs(((key_rotated[0][0] + key_rotated[1][0]) * 0.5) - ((key_rest[0][0] + key_rest[1][0]) * 0.5))
            > 0.004
            or abs(((key_rotated[0][1] + key_rotated[1][1]) * 0.5) - ((key_rest[0][1] + key_rest[1][1]) * 0.5))
            > 0.004
        ),
        details=f"rest={key_rest}, rotated={key_rotated}",
    )

    key_limits = key_turn.motion_limits
    ctx.check(
        "winding key uses continuous motion limits",
        key_turn.joint_type == ArticulationType.CONTINUOUS
        and key_limits is not None
        and key_limits.lower is None
        and key_limits.upper is None,
        details=f"joint_type={key_turn.joint_type}, limits={key_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
