from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_metronome")

    body_mat = model.material("warm_black_plastic", rgba=(0.015, 0.014, 0.012, 1.0))
    face_mat = model.material("satin_ivory_face", rgba=(0.86, 0.82, 0.70, 1.0))
    mark_mat = model.material("printed_black_marks", rgba=(0.02, 0.018, 0.015, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    brass_mat = model.material("aged_brass", rgba=(0.72, 0.52, 0.20, 1.0))
    rubber_mat = model.material("matte_rubber", rgba=(0.03, 0.03, 0.032, 1.0))

    housing_w = 0.070
    housing_d = 0.035
    housing_h = 0.105

    housing = model.part("housing")
    housing.visual(
        Box((housing_w, housing_d, housing_h)),
        origin=Origin(xyz=(0.0, 0.0, housing_h / 2.0)),
        material=body_mat,
        name="rectangular_body",
    )

    # Raised front tempo scale and printed ticks make the compact body read as a metronome.
    housing.visual(
        Box((0.044, 0.003, 0.074)),
        origin=Origin(xyz=(0.0, -housing_d / 2.0 - 0.0002, 0.055)),
        material=face_mat,
        name="tempo_scale_face",
    )
    housing.visual(
        Box((0.005, 0.0035, 0.062)),
        origin=Origin(xyz=(0.0, -housing_d / 2.0 - 0.0007, 0.056)),
        material=mark_mat,
        name="tempo_center_slot",
    )
    for idx, z in enumerate((0.030, 0.042, 0.054, 0.066, 0.078)):
        housing.visual(
            Box((0.021 if idx % 2 == 0 else 0.015, 0.0038, 0.0016)),
            origin=Origin(xyz=(0.015, -housing_d / 2.0 - 0.0010, z)),
            material=mark_mat,
            name=f"tempo_tick_{idx}",
        )

    # Top fork for the pendulum pivot: two cheeks leave a real center gap for the swinging hub.
    pivot_z = housing_h + 0.006
    housing.visual(
        Box((0.018, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, pivot_z)),
        material=body_mat,
        name="top_pivot_cheek_0",
    )
    housing.visual(
        Box((0.018, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.008, pivot_z)),
        material=body_mat,
        name="top_pivot_cheek_1",
    )

    # Rear hinge barrels for the spring clip, split into outer knuckles around the moving center barrel.
    clip_hinge_y = housing_d / 2.0 + 0.0045
    clip_hinge_z = 0.076
    housing.visual(
        Cylinder(radius=0.0046, length=0.013),
        origin=Origin(
            xyz=(-0.025, clip_hinge_y, clip_hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_mat,
        name="rear_hinge_barrel_0",
    )
    housing.visual(
        Cylinder(radius=0.0046, length=0.013),
        origin=Origin(
            xyz=(0.025, clip_hinge_y, clip_hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_mat,
        name="rear_hinge_barrel_1",
    )

    housing.visual(
        Box((0.054, 0.007, 0.021)),
        origin=Origin(xyz=(0.0, housing_d / 2.0 + 0.0024, 0.028)),
        material=rubber_mat,
        name="fixed_rear_grip",
    )

    # Side winding-key collar is fixed to the housing; the key itself is a continuous child.
    key_origin = (housing_w / 2.0 + 0.0038, -0.001, 0.057)
    housing.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(key_origin[0] - 0.0022, key_origin[1], key_origin[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="winding_collar",
    )

    pendulum = model.part("pendulum_rod")
    pendulum.visual(
        Cylinder(radius=0.0040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="pivot_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.0014, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=brass_mat,
        name="pendulum_rod",
    )
    pendulum.visual(
        Box((0.006, 0.0022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brass_mat,
        name="rod_root_tang",
    )

    weight_tube = (
        cq.Workplane("XY")
        .circle(0.0090)
        .circle(0.0029)
        .extrude(0.018, both=True)
    )
    weight = model.part("sliding_weight")
    weight.visual(
        mesh_from_cadquery(weight_tube, "sliding_weight_tube", tolerance=0.0004),
        material=brass_mat,
        name="weight_tube",
    )
    weight.visual(
        Cylinder(radius=0.0021, length=0.0108),
        origin=Origin(xyz=(0.0067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="thumb_screw",
    )

    clip_arm = model.part("clip_arm")
    clip_arm.visual(
        Cylinder(radius=0.0043, length=0.037),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="center_hinge_barrel",
    )
    clip_arm.visual(
        Box((0.030, 0.009, 0.010)),
        origin=Origin(xyz=(0.0, 0.0045, -0.004)),
        material=steel_mat,
        name="hinge_bridge",
    )
    clip_arm.visual(
        Box((0.052, 0.006, 0.078)),
        origin=Origin(xyz=(0.0, 0.010, -0.039)),
        material=steel_mat,
        name="spring_clip_leaf",
    )
    clip_arm.visual(
        Box((0.048, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.006, -0.069)),
        material=rubber_mat,
        name="moving_grip_pad",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0030, length=0.013),
        origin=Origin(xyz=(0.0065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="key_shaft",
    )
    winding_key.visual(
        Box((0.0045, 0.028, 0.012)),
        origin=Origin(xyz=(0.0150, 0.0, 0.0)),
        material=brass_mat,
        name="butterfly_key",
    )
    winding_key.visual(
        Cylinder(radius=0.0055, length=0.0045),
        origin=Origin(xyz=(0.0150, -0.014, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_mat,
        name="key_lobe_0",
    )
    winding_key.visual(
        Cylinder(radius=0.0055, length=0.0045),
        origin=Origin(xyz=(0.0150, 0.014, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_mat,
        name="key_lobe_1",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.045),
    )
    model.articulation(
        "housing_to_clip",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=clip_arm,
        origin=Origin(xyz=(0.0, clip_hinge_y, clip_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=0.75),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=key_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum_rod")
    weight = object_model.get_part("sliding_weight")
    clip_arm = object_model.get_part("clip_arm")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_slide = object_model.get_articulation("pendulum_to_weight")
    clip_joint = object_model.get_articulation("housing_to_clip")
    key_joint = object_model.get_articulation("housing_to_key")

    ctx.expect_gap(
        pendulum,
        housing,
        axis="z",
        min_gap=0.004,
        max_gap=0.012,
        positive_elem="pendulum_rod",
        negative_elem="rectangular_body",
        name="pendulum rod protrudes from housing top",
    )
    ctx.expect_contact(
        pendulum,
        housing,
        elem_a="pivot_hub",
        elem_b="top_pivot_cheek_0",
        contact_tol=0.0008,
        name="pendulum hub is captured in top fork",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="pendulum_rod",
        outer_elem="weight_tube",
        margin=0.0,
        name="sliding weight is centered around rod",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        elem_a="weight_tube",
        elem_b="pendulum_rod",
        min_overlap=0.016,
        name="weight remains engaged on rod at rest",
    )
    ctx.expect_contact(
        weight,
        pendulum,
        elem_a="thumb_screw",
        elem_b="pendulum_rod",
        contact_tol=0.0005,
        name="thumb screw bears on pendulum rod",
    )
    ctx.expect_contact(
        clip_arm,
        housing,
        elem_a="center_hinge_barrel",
        elem_b="rear_hinge_barrel_0",
        contact_tol=0.0008,
        name="clip center barrel seats against rear hinge knuckle",
    )
    ctx.expect_gap(
        clip_arm,
        housing,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="moving_grip_pad",
        negative_elem="fixed_rear_grip",
        name="clip leaves a narrow music-stand grip gap",
    )
    ctx.expect_contact(
        winding_key,
        housing,
        elem_a="key_shaft",
        elem_b="winding_collar",
        contact_tol=0.0008,
        name="winding key shaft seats in side collar",
    )

    rod_rest = ctx.part_element_world_aabb(pendulum, elem="pendulum_rod")
    with ctx.pose({pendulum_joint: 0.30}):
        rod_swept = ctx.part_element_world_aabb(pendulum, elem="pendulum_rod")
    ctx.check(
        "pendulum swings side to side",
        rod_rest is not None
        and rod_swept is not None
        and rod_swept[1][0] > rod_rest[1][0] + 0.020,
        details=f"rest={rod_rest}, swept={rod_swept}",
    )

    weight_rest = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.045}):
        weight_high = ctx.part_world_position(weight)
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            elem_a="weight_tube",
            elem_b="pendulum_rod",
            min_overlap=0.010,
            name="raised weight still remains on rod",
        )
    ctx.check(
        "sliding weight travels upward along rod",
        weight_rest is not None and weight_high is not None and weight_high[2] > weight_rest[2] + 0.040,
        details=f"rest={weight_rest}, high={weight_high}",
    )

    clip_rest = ctx.part_element_world_aabb(clip_arm, elem="moving_grip_pad")
    with ctx.pose({clip_joint: 0.65}):
        clip_open = ctx.part_element_world_aabb(clip_arm, elem="moving_grip_pad")
    ctx.check(
        "rear clip arm opens away from housing",
        clip_rest is not None
        and clip_open is not None
        and clip_open[1][1] > clip_rest[1][1] + 0.020,
        details=f"rest={clip_rest}, open={clip_open}",
    )

    key_rest = ctx.part_element_world_aabb(winding_key, elem="butterfly_key")
    with ctx.pose({key_joint: math.pi / 2.0}):
        key_turned = ctx.part_element_world_aabb(winding_key, elem="butterfly_key")
    ctx.check(
        "winding key rotates continuously about side shaft",
        key_rest is not None
        and key_turned is not None
        and (key_turned[1][2] - key_turned[0][2]) > (key_rest[1][2] - key_rest[0][2]) + 0.010,
        details=f"rest={key_rest}, turned={key_turned}",
    )

    return ctx.report()


object_model = build_object_model()
