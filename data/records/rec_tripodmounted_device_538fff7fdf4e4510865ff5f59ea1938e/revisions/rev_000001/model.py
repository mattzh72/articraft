from __future__ import annotations

import math

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
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _add_leg_geometry(part, material, foot_material) -> None:
    strut_start = (0.012, 0.0, -0.005)
    foot_center = (0.082, 0.0, -0.024)
    strut_vec = (
        foot_center[0] - strut_start[0],
        foot_center[1] - strut_start[1],
        foot_center[2] - strut_start[2],
    )
    strut_len = math.sqrt(sum(component * component for component in strut_vec))
    strut_pitch = math.atan2(strut_vec[0], strut_vec[2])

    part.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="hinge_knuckle",
    )
    part.visual(
        Box((0.014, 0.008, 0.008)),
        origin=Origin(xyz=(0.007, 0.0, -0.004)),
        material=material,
        name="hinge_tab",
    )
    part.visual(
        Cylinder(radius=0.0052, length=strut_len),
        origin=Origin(
            xyz=(
                (strut_start[0] + foot_center[0]) * 0.5,
                0.0,
                (strut_start[2] + foot_center[2]) * 0.5,
            ),
            rpy=(0.0, strut_pitch, 0.0),
        ),
        material=material,
        name="leg_strut",
    )
    part.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=foot_center),
        material=foot_material,
        name="foot_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_camera_tripod")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.40, 0.41, 0.43, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.10, 0.14, 0.18, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=matte_black,
        name="hub_shell",
    )
    crown.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=matte_black,
        name="upper_collar",
    )
    crown.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_graphite,
        name="pan_seat",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            Box((0.014, 0.010, 0.008)),
            origin=Origin(xyz=(0.020 * c, 0.020 * s, 0.035), rpy=(0.0, 0.0, angle)),
            material=matte_black,
            name=f"leg_mount_{index}",
        )

    pan = model.part("pan")
    pan.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="pan_base",
    )
    pan.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_graphite,
        name="head_stem",
    )
    pan.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_graphite,
        name="ball_head",
    )
    for side, y_sign in (("negative", -1.0), ("positive", 1.0)):
        y_pos = 0.017 * y_sign
        pan.visual(
            Box((0.016, 0.006, 0.024)),
            origin=Origin(xyz=(0.0, y_pos, 0.029)),
            material=matte_black,
            name=f"{side}_arm",
        )
    pan.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(
            xyz=(0.0, -0.0215, 0.041),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="tilt_cap",
    )
    pan.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(
            xyz=(0.0, 0.0215, 0.041),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="knob_seat",
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.008)),
        material=dark_graphite,
        name="pivot_block",
    )
    camera.visual(
        Box((0.012, 0.010, 0.008)),
        origin=Origin(xyz=(0.012, -0.009, 0.004)),
        material=dark_graphite,
        name="trunnion_pad_0",
    )
    camera.visual(
        Box((0.012, 0.010, 0.008)),
        origin=Origin(xyz=(0.012, 0.009, 0.004)),
        material=dark_graphite,
        name="trunnion_pad_1",
    )
    camera.visual(
        Box((0.052, 0.024, 0.028)),
        origin=Origin(xyz=(0.036, 0.0, 0.019)),
        material=warm_gray,
        name="body_shell",
    )
    camera.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(
            xyz=(0.069, 0.0, 0.019),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_graphite,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(
            xyz=(0.078, 0.0, 0.019),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="lens_bezel",
    )
    camera.visual(
        Box((0.010, 0.008, 0.004)),
        origin=Origin(xyz=(0.023, 0.006, 0.035)),
        material=dark_graphite,
        name="top_button",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(
            xyz=(0.0, 0.004, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_graphite,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="grip",
    )
    knob.visual(
        Box((0.004, 0.004, 0.006)),
        origin=Origin(xyz=(0.005, 0.012, 0.0)),
        material=matte_black,
        name="grip_tab",
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        _add_leg_geometry(leg, matte_black, rubber_black)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), 0.031), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.5,
                lower=0.0,
                upper=1.15,
            ),
        )

    model.articulation(
        "crown_to_pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "pan_to_knob",
        ArticulationType.CONTINUOUS,
        parent=pan,
        child=knob,
        origin=Origin(xyz=(0.0, 0.024, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan = object_model.get_part("pan")
    camera = object_model.get_part("camera")
    knob = object_model.get_part("knob")
    leg_0 = object_model.get_part("leg_0")

    pan_joint = object_model.get_articulation("crown_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")
    knob_joint = object_model.get_articulation("pan_to_knob")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    ctx.expect_gap(
        camera,
        crown,
        axis="z",
        min_gap=0.035,
        name="camera clears the tripod crown",
    )
    ctx.expect_overlap(
        camera,
        pan,
        axes="y",
        min_overlap=0.020,
        name="camera stays centered between the tilt arms",
    )
    ctx.expect_origin_gap(
        knob,
        pan,
        axis="y",
        min_gap=0.020,
        max_gap=0.028,
        name="side knob sits off one side of the head",
    )

    foot_details = []
    feet_on_table = True
    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        if foot_aabb is None:
            feet_on_table = False
            foot_details.append(f"leg_{index}: missing foot aabb")
            continue
        foot_min_z = foot_aabb[0][2]
        foot_max_z = foot_aabb[1][2]
        on_table = -0.001 <= foot_min_z <= 0.002 and foot_max_z <= 0.015
        feet_on_table &= on_table
        foot_details.append(
            f"leg_{index}: min_z={foot_min_z:.4f}, max_z={foot_max_z:.4f}, on_table={on_table}"
        )
    ctx.check(
        "tripod feet rest on the tabletop",
        feet_on_table,
        details="; ".join(foot_details),
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_bezel"))
    with ctx.pose({tilt_joint: 0.95}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_bezel"))
    tilt_ok = (
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.035
        and tilted_lens[0] < rest_lens[0] - 0.015
    )
    ctx.check(
        "camera tilts upward about the head hinge",
        tilt_ok,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_bezel"))
    pan_ok = (
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[0] - 0.010
        and abs(panned_lens[0]) < 0.020
    )
    ctx.check(
        "head pans around the vertical axis",
        pan_ok,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    rest_knob_tab = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_tab"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_knob_tab = _aabb_center(ctx.part_element_world_aabb(knob, elem="grip_tab"))
    knob_ok = (
        rest_knob_tab is not None
        and turned_knob_tab is not None
        and abs(turned_knob_tab[0] - rest_knob_tab[0]) > 0.003
        and abs(turned_knob_tab[2] - rest_knob_tab[2]) > 0.003
    )
    ctx.check(
        "side knob visibly rotates on its shaft",
        knob_ok,
        details=f"rest_tab={rest_knob_tab}, turned_tab={turned_knob_tab}",
    )

    rest_foot = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    with ctx.pose({leg_joint: 1.15}):
        folded_foot = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    leg_ok = (
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.050
    )
    ctx.check(
        "one tripod leg folds upward from the standing pose",
        leg_ok,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
