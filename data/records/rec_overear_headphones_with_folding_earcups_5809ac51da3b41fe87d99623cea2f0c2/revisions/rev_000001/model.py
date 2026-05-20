from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


SLEEVE_X = 0.105
SLEEVE_BOTTOM_Z = 0.165
SLEEVE_CENTER_Z = 0.195
SLIDE_TRAVEL = 0.030
FOLD_HINGE_Z = -0.040
CUP_CENTER_Z = -0.070


def _extrusion_along_x(geometry: MeshGeometry) -> MeshGeometry:
    """Re-map a profile extruded on local Z so extrusion thickness runs along X."""
    mapped = MeshGeometry()
    mapped.vertices = [(z, x, y) for (x, y, z) in geometry.vertices]
    mapped.faces = list(geometry.faces)
    return mapped


def _oval_solid(width_y: float, height_z: float, thickness_x: float) -> MeshGeometry:
    return _extrusion_along_x(
        ExtrudeGeometry(
            superellipse_profile(width_y, height_z, exponent=2.45, segments=64),
            thickness_x,
            cap=True,
            center=True,
        )
    )


def _oval_ring(
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    thickness_x: float,
) -> MeshGeometry:
    return _extrusion_along_x(
        ExtrudeWithHolesGeometry(
            superellipse_profile(outer_y, outer_z, exponent=2.35, segments=72),
            [superellipse_profile(inner_y, inner_z, exponent=2.35, segments=72)],
            thickness_x,
            cap=True,
            center=True,
        )
    )


def _sleeve_cadquery() -> cq.Workplane:
    outer_x, outer_y, height = 0.022, 0.026, 0.060
    clear_x, clear_y = 0.012, 0.014
    outer = cq.Workplane("XY").box(outer_x, outer_y, height)
    slot = cq.Workplane("XY").box(clear_x, clear_y, height + 0.012)
    return outer.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_over_ear_headphones")

    black = model.material("matte_black", rgba=(0.006, 0.006, 0.007, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    foam = model.material("soft_black_foam", rgba=(0.002, 0.002, 0.002, 1.0))
    metal = model.material("brushed_dark_metal", rgba=(0.38, 0.40, 0.42, 1.0))
    accent = model.material("satin_graphite", rgba=(0.09, 0.095, 0.105, 1.0))

    sleeve_mesh = mesh_from_cadquery(_sleeve_cadquery(), "adjuster_sleeve")
    cup_shell_mesh = mesh_from_geometry(_oval_solid(0.076, 0.092, 0.030), "cup_shell")
    cushion_mesh = mesh_from_geometry(_oval_ring(0.078, 0.096, 0.046, 0.062, 0.014), "ear_cushion")
    outer_cap_mesh = mesh_from_geometry(_oval_solid(0.062, 0.076, 0.006), "outer_cap")
    cloth_mesh = mesh_from_geometry(_oval_solid(0.041, 0.056, 0.002), "acoustic_cloth")

    headband = model.part("headband")
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-SLEEVE_X, 0.0, 0.226),
                    (-0.072, 0.0, 0.242),
                    (0.0, 0.0, 0.252),
                    (0.072, 0.0, 0.242),
                    (SLEEVE_X, 0.0, 0.226),
                ],
                radius=0.006,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "spring_headband",
        ),
        material=black,
        name="spring_band",
    )
    headband.visual(
        Box((0.118, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        material=foam,
        name="head_cushion",
    )
    for side, x in (("left", -SLEEVE_X), ("right", SLEEVE_X)):
        headband.visual(
            sleeve_mesh,
            origin=Origin(xyz=(x, 0.0, SLEEVE_CENTER_Z)),
            material=charcoal,
            name=f"{side}_sleeve",
        )
        headband.visual(
            Box((0.026, 0.030, 0.008)),
            origin=Origin(xyz=(x, 0.0, SLEEVE_CENTER_Z + 0.034)),
            material=charcoal,
            name=f"{side}_sleeve_cap",
        )
    for x in (-0.045, 0.045):
        headband.visual(
            Box((0.018, 0.026, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.234)),
            material=black,
            name=f"pad_strut_{'neg' if x < 0 else 'pos'}",
        )

    def add_arm(side: str, sleeve_x: float):
        arm = model.part(f"{side}_arm")
        arm.visual(
            Box((0.012, 0.014, 0.084)),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=metal,
            name="slide_rail",
        )
        arm.visual(
            Box((0.012, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, FOLD_HINGE_Z + 0.009)),
            material=metal,
            name="hinge_block",
        )
        for y, label in ((-0.015, "rear"), (0.015, "front")):
            arm.visual(
                Cylinder(radius=0.006, length=0.020),
                origin=Origin(xyz=(0.0, y, FOLD_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"{label}_fold_knuckle",
            )
        slide = model.articulation(
            f"{side}_slide",
            ArticulationType.PRISMATIC,
            parent=headband,
            child=arm,
            origin=Origin(xyz=(sleeve_x, 0.0, SLEEVE_BOTTOM_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=SLIDE_TRAVEL),
        )
        return arm, slide

    def add_yoke(side: str, arm, inward_axis_y: float):
        yoke = model.part(f"{side}_yoke")
        yoke.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=metal,
            name="hinge_post",
        )
        yoke.visual(
            Box((0.012, 0.090, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=metal,
            name="yoke_bridge",
        )
        yoke.visual(
            Box((0.010, 0.006, 0.056)),
            origin=Origin(xyz=(0.0, -0.043, -0.050)),
            material=metal,
            name="rear_yoke_arm",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.0, -0.040, CUP_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="rear_pivot_washer",
        )
        yoke.visual(
            Box((0.010, 0.006, 0.056)),
            origin=Origin(xyz=(0.0, 0.043, -0.050)),
            material=metal,
            name="front_yoke_arm",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.0, 0.040, CUP_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="front_pivot_washer",
        )
        fold = model.articulation(
            f"{side}_fold",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=yoke,
            origin=Origin(xyz=(0.0, 0.0, FOLD_HINGE_Z)),
            axis=(0.0, inward_axis_y, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.55),
        )
        return yoke, fold

    def add_cup(side: str, yoke, inner_sign: float):
        cup = model.part(f"{side}_cup")
        cup.visual(cup_shell_mesh, material=charcoal, name="cup_shell")
        cup.visual(
            cushion_mesh,
            origin=Origin(xyz=(0.022 * inner_sign, 0.0, 0.0)),
            material=foam,
            name="ear_cushion",
        )
        cup.visual(
            outer_cap_mesh,
            origin=Origin(xyz=(-0.018 * inner_sign, 0.0, 0.0)),
            material=accent,
            name="outer_cap",
        )
        cup.visual(
            cloth_mesh,
            origin=Origin(xyz=(0.014 * inner_sign, 0.0, 0.0)),
            material=black,
            name="acoustic_cloth",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="rear_pivot_boss",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, 0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="front_pivot_boss",
        )
        swivel = model.articulation(
            f"{side}_swivel",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, CUP_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=-0.35, upper=0.35),
        )
        return cup, swivel

    left_arm, _ = add_arm("left", -SLEEVE_X)
    right_arm, _ = add_arm("right", SLEEVE_X)
    left_yoke, _ = add_yoke("left", left_arm, -1.0)
    right_yoke, _ = add_yoke("right", right_arm, 1.0)
    add_cup("left", left_yoke, inner_sign=1.0)
    add_cup("right", right_yoke, inner_sign=-1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")
    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")

    for side, arm in (("left", left_arm), ("right", right_arm)):
        ctx.allow_overlap(
            headband,
            arm,
            elem_a=f"{side}_sleeve",
            elem_b="slide_rail",
            reason=(
                "The telescoping rail is intentionally represented as a clipped "
                "member sliding inside the adjuster sleeve proxy."
            ),
        )

    for side, arm in (("left", left_arm), ("right", right_arm)):
        ctx.expect_within(
            arm,
            headband,
            axes="xy",
            inner_elem="slide_rail",
            outer_elem=f"{side}_sleeve",
            margin=0.002,
            name=f"{side} rail stays inside sleeve slot footprint",
        )
        ctx.expect_overlap(
            arm,
            headband,
            axes="z",
            elem_a="slide_rail",
            elem_b=f"{side}_sleeve",
            min_overlap=0.045,
            name=f"{side} rail has collapsed retained insertion",
        )

    rest_left = ctx.part_world_position(left_cup)
    rest_right = ctx.part_world_position(right_cup)
    rest_left_arm = ctx.part_world_position(left_arm)

    with ctx.pose({left_slide: SLIDE_TRAVEL, right_slide: SLIDE_TRAVEL}):
        for side, arm in (("left", left_arm), ("right", right_arm)):
            ctx.expect_within(
                arm,
                headband,
                axes="xy",
                inner_elem="slide_rail",
                outer_elem=f"{side}_sleeve",
                margin=0.002,
                name=f"{side} rail stays clipped when extended",
            )
            ctx.expect_overlap(
                arm,
                headband,
                axes="z",
                elem_a="slide_rail",
                elem_b=f"{side}_sleeve",
                min_overlap=0.018,
                name=f"{side} rail remains inserted at max size",
            )
        extended_left_arm = ctx.part_world_position(left_arm)

    ctx.check(
        "size adjusters extend downward",
        rest_left_arm is not None
        and extended_left_arm is not None
        and extended_left_arm[2] < rest_left_arm[2] - 0.020,
        details=f"rest={rest_left_arm}, extended={extended_left_arm}",
    )

    with ctx.pose({left_fold: 1.20, right_fold: 1.20}):
        folded_left = ctx.part_world_position(left_cup)
        folded_right = ctx.part_world_position(right_cup)
    ctx.check(
        "left fold hinge tucks cup inward",
        rest_left is not None and folded_left is not None and folded_left[0] > rest_left[0] + 0.040,
        details=f"rest={rest_left}, folded={folded_left}",
    )
    ctx.check(
        "right fold hinge tucks cup inward",
        rest_right is not None and folded_right is not None and folded_right[0] < rest_right[0] - 0.040,
        details=f"rest={rest_right}, folded={folded_right}",
    )

    for side, yoke, cup in (("left", left_yoke, left_cup), ("right", right_yoke, right_cup)):
        ctx.expect_contact(
            yoke,
            cup,
            elem_a="front_pivot_washer",
            elem_b="front_pivot_boss",
            contact_tol=0.003,
            name=f"{side} front cup pivot is seated in yoke",
        )
        ctx.expect_contact(
            yoke,
            cup,
            elem_a="rear_pivot_washer",
            elem_b="rear_pivot_boss",
            contact_tol=0.003,
            name=f"{side} rear cup pivot is seated in yoke",
        )

    return ctx.report()


object_model = build_object_model()
