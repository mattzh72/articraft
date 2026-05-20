from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)
import cadquery as cq


def _oval_points(x: float, ry: float, rz: float, segments: int) -> list[tuple[float, float, float]]:
    return [
        (x, ry * math.cos(2.0 * math.pi * i / segments), rz * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _oval_solid(x0: float, x1: float, ry: float, rz: float, *, segments: int = 64) -> MeshGeometry:
    """Closed oval cylinder with its extrusion along local X."""
    geom = MeshGeometry()
    left = [geom.add_vertex(*p) for p in _oval_points(x0, ry, rz, segments)]
    right = [geom.add_vertex(*p) for p in _oval_points(x1, ry, rz, segments)]
    c0 = geom.add_vertex(x0, 0.0, 0.0)
    c1 = geom.add_vertex(x1, 0.0, 0.0)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(left[i], right[i], right[j])
        geom.add_face(left[i], right[j], left[j])
        geom.add_face(c0, left[j], left[i])
        geom.add_face(c1, right[i], right[j])
    return geom


def _oval_ring(
    x0: float,
    x1: float,
    outer_ry: float,
    outer_rz: float,
    inner_ry: float,
    inner_rz: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Watertight elliptical annular cushion, extruded along local X."""
    geom = MeshGeometry()
    o0 = [geom.add_vertex(*p) for p in _oval_points(x0, outer_ry, outer_rz, segments)]
    o1 = [geom.add_vertex(*p) for p in _oval_points(x1, outer_ry, outer_rz, segments)]
    i0 = [geom.add_vertex(*p) for p in _oval_points(x0, inner_ry, inner_rz, segments)]
    i1 = [geom.add_vertex(*p) for p in _oval_points(x1, inner_ry, inner_rz, segments)]

    for k in range(segments):
        j = (k + 1) % segments
        # Outer wall.
        geom.add_face(o0[k], o1[k], o1[j])
        geom.add_face(o0[k], o1[j], o0[j])
        # Inner opening wall.
        geom.add_face(i0[k], i1[j], i1[k])
        geom.add_face(i0[k], i0[j], i1[j])
        # Annular faces at both x planes.
        geom.add_face(o0[k], o0[j], i0[j])
        geom.add_face(o0[k], i0[j], i0[k])
        geom.add_face(o1[k], i1[k], i1[j])
        geom.add_face(o1[k], i1[j], o1[j])
    return geom


def _add_headband_visuals(model: ArticulatedObject, headband) -> None:
    rail = model.get_material("brushed_steel") if hasattr(model, "get_material") else "brushed_steel"
    soft = model.get_material("soft_black") if hasattr(model, "get_material") else "soft_black"
    plastic = model.get_material("satin_black") if hasattr(model, "get_material") else "satin_black"

    rail_path_front = [
        (-0.112, 0.028, 0.108),
        (-0.084, 0.028, 0.182),
        (0.000, 0.028, 0.213),
        (0.084, 0.028, 0.182),
        (0.112, 0.028, 0.108),
    ]
    rail_path_rear = [(x, -0.028, z) for x, _, z in rail_path_front]
    cushion_path = [
        (-0.090, 0.000, 0.088),
        (-0.060, 0.000, 0.163),
        (0.000, 0.000, 0.195),
        (0.060, 0.000, 0.163),
        (0.090, 0.000, 0.088),
    ]

    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_path_front, radius=0.0048, samples_per_segment=16, radial_segments=18
            ),
            "front_split_rail",
        ),
        material=rail,
        name="front_split_rail",
    )
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_path_rear, radius=0.0048, samples_per_segment=16, radial_segments=18
            ),
            "rear_split_rail",
        ),
        material=rail,
        name="rear_split_rail",
    )
    headband.visual(
        mesh_from_geometry(
            sweep_profile_along_spline(
                cushion_path,
                profile=rounded_rect_profile(0.032, 0.007, 0.0025),
                samples_per_segment=14,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "inner_cushion",
        ),
        material=soft,
        name="inner_cushion",
    )

    # Small molded cross ties make the split band visibly one connected frame.
    for x, name in [(-0.112, "left"), (0.0, "top"), (0.112, "right")]:
        z = 0.120 if x else 0.212
        headband.visual(
            Box((0.028, 0.068, 0.010)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=plastic,
            name=f"{name}_rail_tie",
        )

    # Telescoping guide channels: two side cheeks plus an upper bridge, leaving
    # an open center slot for each support arm to slide without interpenetration.
    headband.visual(
        Box((0.022, 0.010, 0.100)),
        origin=Origin(xyz=(-0.112, 0.031, 0.085)),
        material=plastic,
        name="left_guide_front",
    )
    headband.visual(
        Box((0.022, 0.010, 0.100)),
        origin=Origin(xyz=(-0.112, -0.031, 0.085)),
        material=plastic,
        name="left_guide_rear",
    )
    headband.visual(
        Box((0.028, 0.072, 0.010)),
        origin=Origin(xyz=(-0.112, 0.0, 0.132)),
        material=plastic,
        name="left_guide_top",
    )
    headband.visual(
        Box((0.022, 0.010, 0.100)),
        origin=Origin(xyz=(0.112, 0.031, 0.085)),
        material=plastic,
        name="right_guide_front",
    )
    headband.visual(
        Box((0.022, 0.010, 0.100)),
        origin=Origin(xyz=(0.112, -0.031, 0.085)),
        material=plastic,
        name="right_guide_rear",
    )
    headband.visual(
        Box((0.028, 0.072, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, 0.132)),
        material=plastic,
        name="right_guide_top",
    )


def _add_arm_visuals(arm, side_name: str, material, accent) -> None:
    arm.visual(
        Box((0.012, 0.006, 0.126)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=material,
        name="slider_strip",
    )
    # Tiny detent pads read as length marks/buttons on the sliding metal strip.
    for i, z in enumerate([-0.030, -0.055, -0.080]):
        arm.visual(
            Box((0.013, 0.0018, 0.005)),
            origin=Origin(xyz=(0.0, 0.0039, z)),
            material=accent,
            name=f"detent_{i}",
        )
    arm.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.032, -0.128)),
        material=material,
        name="front_fold_cheek",
    )
    arm.visual(
        Box((0.018, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, -0.032, -0.128)),
        material=material,
        name="rear_fold_cheek",
    )
    arm.visual(
        Box((0.016, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        material=material,
        name="fold_bridge",
    )


def _add_yoke_visuals(yoke, material, pin_material) -> None:
    yoke.visual(
        Cylinder(radius=0.0085, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="fold_barrel",
    )
    yoke.visual(
        Box((0.013, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=material,
        name="center_stem",
    )
    yoke.visual(
        Box((0.019, 0.110, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=material,
        name="top_bridge",
    )
    yoke.visual(
        Box((0.014, 0.008, 0.077)),
        origin=Origin(xyz=(0.0, 0.053, -0.051)),
        material=material,
        name="front_yoke_arm",
    )
    yoke.visual(
        Cylinder(radius=0.0115, length=0.006),
        origin=Origin(xyz=(0.0, 0.053, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="front_pivot_boss",
    )
    yoke.visual(
        Box((0.014, 0.008, 0.077)),
        origin=Origin(xyz=(0.0, -0.053, -0.051)),
        material=material,
        name="rear_yoke_arm",
    )
    yoke.visual(
        Cylinder(radius=0.0115, length=0.006),
        origin=Origin(xyz=(0.0, -0.053, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="rear_pivot_boss",
    )


def _add_earcup_visuals(cup, side: str, shell_mat, pad_mat, grille_mat, pin_mat) -> None:
    inward = 1.0 if side == "left" else -1.0
    shell_x0, shell_x1 = ((-0.030, 0.010) if side == "left" else (-0.010, 0.030))
    pad_x0, pad_x1 = ((0.010, 0.031) if side == "left" else (-0.031, -0.010))
    grille_x0, grille_x1 = ((0.009, 0.013) if side == "left" else (-0.013, -0.009))

    cup.visual(
        mesh_from_geometry(_oval_solid(shell_x0, shell_x1, 0.040, 0.052), f"{side}_cup_shell"),
        material=shell_mat,
        name="cup_shell",
    )
    cup.visual(
        mesh_from_geometry(_oval_ring(pad_x0, pad_x1, 0.044, 0.056, 0.025, 0.034), f"{side}_ear_pad"),
        material=pad_mat,
        name="ear_pad",
    )
    cup.visual(
        mesh_from_geometry(_oval_solid(grille_x0, grille_x1, 0.021, 0.030), f"{side}_driver_grille"),
        material=grille_mat,
        name="driver_grille",
    )
    cup.visual(
        Cylinder(radius=0.0068, length=0.098),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="trunnion",
    )
    # A small raised logo cap on the outside face clarifies cup orientation.
    cup.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(-inward * 0.0315, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_mat,
        name="outer_badge",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_monitoring_headphones")

    satin_black = model.material("satin_black", color=(0.015, 0.015, 0.018, 1.0))
    soft_black = model.material("soft_black", color=(0.002, 0.002, 0.003, 1.0))
    dark_plastic = model.material("dark_cup_plastic", color=(0.055, 0.058, 0.062, 1.0))
    rubber_pad = model.material("leatherette_pad", color=(0.006, 0.005, 0.004, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.60, 0.62, 0.64, 1.0))
    dark_metal = model.material("gunmetal_hinge", color=(0.20, 0.21, 0.22, 1.0))
    grille = model.material("perforated_grille", color=(0.010, 0.011, 0.012, 1.0))

    headband = model.part("headband")
    _add_headband_visuals(model, headband)

    left_arm = model.part("left_arm")
    right_arm = model.part("right_arm")
    _add_arm_visuals(left_arm, "left", brushed_steel, dark_metal)
    _add_arm_visuals(right_arm, "right", brushed_steel, dark_metal)

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    _add_yoke_visuals(left_yoke, satin_black, dark_metal)
    _add_yoke_visuals(right_yoke, satin_black, dark_metal)

    left_earcup = model.part("left_earcup")
    right_earcup = model.part("right_earcup")
    _add_earcup_visuals(left_earcup, "left", dark_plastic, rubber_pad, grille, dark_metal)
    _add_earcup_visuals(right_earcup, "right", dark_plastic, rubber_pad, grille, dark_metal)

    slide_limits = MotionLimits(effort=28.0, velocity=0.16, lower=0.0, upper=0.045)
    model.articulation(
        "left_arm_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_arm,
        origin=Origin(xyz=(-0.112, 0.0, 0.108)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=slide_limits,
    )
    model.articulation(
        "right_arm_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_arm,
        origin=Origin(xyz=(0.112, 0.0, 0.108)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=slide_limits,
    )

    fold_limits = MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.15)
    model.articulation(
        "left_yoke_fold",
        ArticulationType.REVOLUTE,
        parent=left_arm,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=fold_limits,
    )
    model.articulation(
        "right_yoke_fold",
        ArticulationType.REVOLUTE,
        parent=right_arm,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.128)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=fold_limits,
    )

    cup_tilt_limits = MotionLimits(effort=4.0, velocity=2.5, lower=-0.38, upper=0.38)
    model.articulation(
        "left_cup_tilt",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=cup_tilt_limits,
    )
    model.articulation(
        "right_cup_tilt",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=cup_tilt_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_earcup")
    right_cup = object_model.get_part("right_earcup")
    left_slide = object_model.get_articulation("left_arm_slide")
    right_slide = object_model.get_articulation("right_arm_slide")
    left_fold = object_model.get_articulation("left_yoke_fold")
    right_fold = object_model.get_articulation("right_yoke_fold")

    for side, arm in [("left", left_arm), ("right", right_arm)]:
        ctx.expect_within(
            arm,
            headband,
            axes="xy",
            inner_elem="slider_strip",
            outer_elem=f"{side}_guide_top",
            margin=0.001,
            name=f"{side} slider centered in guide slot",
        )
        ctx.expect_overlap(
            arm,
            headband,
            axes="z",
            elem_a="slider_strip",
            elem_b=f"{side}_guide_front",
            min_overlap=0.060,
            name=f"{side} slider retained when retracted",
        )

    rest_left_z = ctx.part_world_position(left_arm)[2]
    rest_right_z = ctx.part_world_position(right_arm)[2]
    with ctx.pose({left_slide: 0.045, right_slide: 0.045}):
        ctx.expect_overlap(
            left_arm,
            headband,
            axes="z",
            elem_a="slider_strip",
            elem_b="left_guide_front",
            min_overlap=0.030,
            name="left slider retained at full extension",
        )
        ctx.expect_overlap(
            right_arm,
            headband,
            axes="z",
            elem_a="slider_strip",
            elem_b="right_guide_front",
            min_overlap=0.030,
            name="right slider retained at full extension",
        )
        ext_left_z = ctx.part_world_position(left_arm)[2]
        ext_right_z = ctx.part_world_position(right_arm)[2]

    ctx.check(
        "support arms slide downward",
        ext_left_z < rest_left_z - 0.035 and ext_right_z < rest_right_z - 0.035,
        details=f"rest=({rest_left_z:.3f},{rest_right_z:.3f}), extended=({ext_left_z:.3f},{ext_right_z:.3f})",
    )

    ctx.expect_gap(
        left_yoke,
        left_cup,
        axis="y",
        positive_elem="front_pivot_boss",
        negative_elem="trunnion",
        max_gap=0.0015,
        max_penetration=0.0,
        name="left cup trunnion seats in front yoke boss",
    )
    ctx.expect_gap(
        right_yoke,
        right_cup,
        axis="y",
        positive_elem="front_pivot_boss",
        negative_elem="trunnion",
        max_gap=0.0015,
        max_penetration=0.0,
        name="right cup trunnion seats in front yoke boss",
    )

    left_rest_x = ctx.part_world_position(left_cup)[0]
    right_rest_x = ctx.part_world_position(right_cup)[0]
    with ctx.pose({left_fold: 0.9, right_fold: 0.9}):
        left_folded_x = ctx.part_world_position(left_cup)[0]
        right_folded_x = ctx.part_world_position(right_cup)[0]
    ctx.check(
        "fold hinges move cups inward",
        left_folded_x > left_rest_x + 0.040 and right_folded_x < right_rest_x - 0.040,
        details=(
            f"left {left_rest_x:.3f}->{left_folded_x:.3f}, "
            f"right {right_rest_x:.3f}->{right_folded_x:.3f}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
