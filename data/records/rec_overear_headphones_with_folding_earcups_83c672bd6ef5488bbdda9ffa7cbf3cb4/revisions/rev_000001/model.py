from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_over_ear_headphones")

    matte_black = Material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    satin_black = Material("satin_black", rgba=(0.035, 0.035, 0.038, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.18, 0.18, 0.17, 1.0))
    soft_leather = Material("soft_leather", rgba=(0.02, 0.018, 0.016, 1.0))
    speaker_cloth = Material("speaker_cloth", rgba=(0.006, 0.006, 0.007, 1.0))
    accent = Material("subtle_logo", rgba=(0.42, 0.42, 0.40, 1.0))

    headband = model.part("headband")

    outer_band = sweep_profile_along_spline(
        [
            (-0.116, 0.0, 0.148),
            (-0.096, 0.0, 0.184),
            (-0.055, 0.0, 0.212),
            (0.000, 0.0, 0.238),
            (0.055, 0.0, 0.212),
            (0.096, 0.0, 0.184),
            (0.116, 0.0, 0.148),
        ],
        profile=rounded_rect_profile(0.018, 0.008, radius=0.003, corner_segments=5),
        samples_per_segment=12,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(
        mesh_from_geometry(outer_band, "outer_headband"),
        material=satin_black,
        name="outer_band",
    )

    cushion = sweep_profile_along_spline(
        [
            (-0.073, 0.0, 0.128),
            (-0.043, 0.0, 0.164),
            (0.000, 0.0, 0.178),
            (0.043, 0.0, 0.164),
            (0.073, 0.0, 0.128),
        ],
        profile=rounded_rect_profile(0.052, 0.017, radius=0.007, corner_segments=7),
        samples_per_segment=12,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(
        mesh_from_geometry(cushion, "padded_headband"),
        material=soft_leather,
        name="crown_pad",
    )

    # Small covered strap blocks tie the soft pad visibly to the structural band.
    for idx, x in enumerate((-0.045, 0.0, 0.045)):
        z = 0.178 if x == 0.0 else 0.158
        headband.visual(
            Box((0.012, 0.032, 0.074)),
            origin=Origin(xyz=(x, 0.0, z + 0.032)),
            material=soft_leather,
            name=f"pad_stay_{idx}",
        )

    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        # Flat reinforced lower arms and hinge knuckles at the end of the band.
        headband.visual(
            Box((0.015, 0.020, 0.080)),
            origin=Origin(xyz=(sx * 0.116, 0.0, 0.116)),
            material=matte_black,
            name=f"{side_name}_arm",
        )
        headband.visual(
            Box((0.016, 0.074, 0.010)),
            origin=Origin(xyz=(sx * 0.116, 0.0, 0.074)),
            material=matte_black,
            name=f"{side_name}_hinge_bridge",
        )
        for y in (-0.025, 0.025):
            headband.visual(
                Cylinder(radius=0.011, length=0.014),
                origin=Origin(
                    xyz=(sx * 0.116, y, 0.058),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"{side_name}_fold_knuckle_{'rear' if y < 0 else 'front'}",
            )

    def add_yoke(side_name: str, sx: float):
        yoke = model.part(f"{side_name}_yoke")
        yoke.visual(
            Cylinder(radius=0.010, length=0.036),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.014, 0.018, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=matte_black,
            name="fold_stem",
        )
        yoke.visual(
            Box((0.016, 0.132, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.033)),
            material=dark_metal,
            name="upper_bridge",
        )
        for y in (-0.061, 0.061):
            yoke.visual(
                Box((0.012, 0.008, 0.058)),
                origin=Origin(xyz=(0.0, y, -0.061)),
                material=dark_metal,
                name=f"{'rear' if y < 0 else 'front'}_fork",
            )
            yoke.visual(
                Cylinder(radius=0.009, length=0.006),
                origin=Origin(
                    xyz=(0.0, y, -0.087),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"{'rear' if y < 0 else 'front'}_pivot_boss",
            )

        fold_axis = (0.0, -1.0, 0.0) if sx < 0.0 else (0.0, 1.0, 0.0)
        model.articulation(
            f"{side_name}_fold",
            ArticulationType.REVOLUTE,
            parent=headband,
            child=yoke,
            origin=Origin(xyz=(sx * 0.116, 0.0, 0.058)),
            axis=fold_axis,
            motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=0.0, upper=0.82),
        )

        cup = model.part(f"{side_name}_cup")
        cup.visual(
            Cylinder(radius=0.050, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="cup_shell",
        )
        cup.visual(
            Cylinder(radius=0.043, length=0.010),
            origin=Origin(
                xyz=(-sx * 0.020, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_black,
            name="outer_cap",
        )
        cup.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(
                xyz=(-sx * 0.026, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=accent,
            name="round_badge",
        )
        cushion_ring = TorusGeometry(radius=0.039, tube=0.010, radial_segments=20, tubular_segments=48)
        cup.visual(
            mesh_from_geometry(cushion_ring, f"{side_name}_ear_pad"),
            origin=Origin(
                xyz=(sx * 0.025, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=soft_leather,
            name="ear_pad",
        )
        cup.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(
                xyz=(sx * 0.029, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=speaker_cloth,
            name="speaker_cloth",
        )
        for y in (-0.052, 0.052):
            cup.visual(
                Cylinder(radius=0.006, length=0.010),
                origin=Origin(
                    xyz=(0.0, y, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"{'rear' if y < 0 else 'front'}_trunnion",
            )
        cup.visual(
            Box((0.010, 0.010, 0.018)),
            origin=Origin(xyz=(sx * 0.008, -0.038, -0.043)),
            material=satin_black,
            name="cable_socket",
        )

        model.articulation(
            f"{side_name}_swivel",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -0.087)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.42, upper=0.42),
        )

    add_yoke("left", -1.0)
    add_yoke("right", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_swivel = object_model.get_articulation("left_swivel")
    right_swivel = object_model.get_articulation("right_swivel")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    for joint in (left_fold, right_fold, left_swivel, right_swivel):
        ctx.check(
            f"{joint.name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type is {joint.articulation_type}",
        )

    rest_left = ctx.part_world_position(left_cup)
    rest_right = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 0.80, right_fold: 0.80}):
        folded_left = ctx.part_world_position(left_cup)
        folded_right = ctx.part_world_position(right_cup)
        ctx.expect_gap(
            right_cup,
            left_cup,
            axis="x",
            min_gap=0.003,
            name="folded cups remain separated",
        )
    ctx.check(
        "left fold moves cup inward",
        rest_left is not None and folded_left is not None and folded_left[0] > rest_left[0] + 0.045,
        details=f"rest={rest_left}, folded={folded_left}",
    )
    ctx.check(
        "right fold moves cup inward",
        rest_right is not None and folded_right is not None and folded_right[0] < rest_right[0] - 0.045,
        details=f"rest={rest_right}, folded={folded_right}",
    )

    left_pad_rest = ctx.part_element_world_aabb(left_cup, elem="ear_pad")
    right_pad_rest = ctx.part_element_world_aabb(right_cup, elem="ear_pad")
    with ctx.pose({left_swivel: 0.35, right_swivel: -0.35}):
        left_pad_tilt = ctx.part_element_world_aabb(left_cup, elem="ear_pad")
        right_pad_tilt = ctx.part_element_world_aabb(right_cup, elem="ear_pad")

    def aabb_center_z(bounds):
        return None if bounds is None else 0.5 * (bounds[0][2] + bounds[1][2])

    ctx.check(
        "left cup swivel tilts pad",
        left_pad_rest is not None
        and left_pad_tilt is not None
        and abs(aabb_center_z(left_pad_tilt) - aabb_center_z(left_pad_rest)) > 0.006,
        details=f"rest={left_pad_rest}, tilted={left_pad_tilt}",
    )
    ctx.check(
        "right cup swivel tilts pad",
        right_pad_rest is not None
        and right_pad_tilt is not None
        and abs(aabb_center_z(right_pad_tilt) - aabb_center_z(right_pad_rest)) > 0.006,
        details=f"rest={right_pad_rest}, tilted={right_pad_tilt}",
    )

    return ctx.report()


object_model = build_object_model()
