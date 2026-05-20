from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="studio_over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.017, 1.0))
    soft_foam = model.material("soft_foam", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.023, 0.022, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    grille_fabric = model.material("grille_fabric", rgba=(0.035, 0.038, 0.040, 1.0))

    headband = model.part("headband")
    band_path = [
        (-0.190, 0.0, 0.525),
        (-0.135, 0.0, 0.590),
        (0.000, 0.0, 0.615),
        (0.135, 0.0, 0.590),
        (0.190, 0.0, 0.525),
    ]
    headband.visual(
        mesh_from_geometry(
            sweep_profile_along_spline(
                band_path,
                profile=rounded_rect_profile(0.042, 0.018, radius=0.006),
                samples_per_segment=14,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "padded_headband_band",
        ),
        material=matte_black,
        name="outer_band",
    )
    cushion_path = [
        (-0.145, 0.0, 0.512),
        (-0.085, 0.0, 0.558),
        (0.000, 0.0, 0.574),
        (0.085, 0.0, 0.558),
        (0.145, 0.0, 0.512),
    ]
    headband.visual(
        mesh_from_geometry(
            sweep_profile_along_spline(
                cushion_path,
                profile=rounded_rect_profile(0.052, 0.014, radius=0.006),
                samples_per_segment=12,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "headband_inner_cushion",
        ),
        material=dark_rubber,
        name="inner_cushion",
    )
    for side_name, side in (("left", -1.0), ("right", 1.0)):
        headband.visual(
            Box((0.050, 0.055, 0.040)),
            origin=Origin(xyz=(side * 0.190, 0.0, 0.531)),
            material=matte_black,
            name=f"{side_name}_end_block",
        )

    sleeve_parts = {}
    extender_parts = {}
    yoke_parts = {}
    cup_parts = {}

    for side_name, side in (("left", -1.0), ("right", 1.0)):
        sleeve = model.part(f"{side_name}_sleeve")
        sleeve_parts[side_name] = sleeve
        sleeve.visual(
            Box((0.044, 0.006, 0.110)),
            origin=Origin(xyz=(0.0, -0.020, 0.055)),
            material=matte_black,
            name="rear_wall",
        )
        sleeve.visual(
            Box((0.044, 0.006, 0.110)),
            origin=Origin(xyz=(0.0, 0.020, 0.055)),
            material=matte_black,
            name="front_wall",
        )
        sleeve.visual(
            Box((0.006, 0.046, 0.110)),
            origin=Origin(xyz=(-0.022, 0.0, 0.055)),
            material=matte_black,
            name="inner_wall",
        )
        sleeve.visual(
            Box((0.006, 0.046, 0.110)),
            origin=Origin(xyz=(0.022, 0.0, 0.055)),
            material=matte_black,
            name="outer_wall",
        )
        sleeve.visual(
            Box((0.046, 0.048, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.115)),
            material=matte_black,
            name="top_cap",
        )
        sleeve.visual(
            Box((0.035, 0.004, 0.082)),
            origin=Origin(xyz=(0.0, 0.024, 0.049)),
            material=gunmetal,
            name="front_slot_trim",
        )
        model.articulation(
            f"headband_to_{side_name}_sleeve",
            ArticulationType.FIXED,
            parent=headband,
            child=sleeve,
            origin=Origin(xyz=(side * 0.190, 0.0, 0.390)),
        )

        extender = model.part(f"{side_name}_extender")
        extender_parts[side_name] = extender
        extender.visual(
            Box((0.020, 0.008, 0.154)),
            origin=Origin(xyz=(0.0, 0.0, -0.0055)),
            material=brushed_steel,
            name="slider_strip",
        )
        for x, label in ((-0.0145, "inner"), (0.0145, "outer")):
            extender.visual(
                Box((0.009, 0.010, 0.060)),
                origin=Origin(xyz=(x, 0.0, 0.030)),
                material=brushed_steel,
                name=f"{label}_slide_shoe",
            )
        for idx, z in enumerate((-0.012, -0.036, -0.060, -0.076)):
            extender.visual(
                Box((0.016, 0.004, 0.004)),
                origin=Origin(xyz=(0.0, 0.0055, z)),
                material=dark_rubber,
                name=f"index_mark_{idx}",
            )
        extender.visual(
            Box((0.026, 0.088, 0.015)),
            origin=Origin(xyz=(0.0, 0.0, -0.074)),
            material=gunmetal,
            name="fold_leaf",
        )
        for y, label in ((-0.032, "rear"), (0.032, "front")):
            extender.visual(
                Box((0.026, 0.018, 0.028)),
                origin=Origin(xyz=(0.0, y, -0.088)),
                material=gunmetal,
                name=f"{label}_fold_cheek",
            )
        for y, label in ((-0.032, "rear"), (0.032, "front")):
            extender.visual(
                Cylinder(radius=0.011, length=0.018),
                origin=Origin(xyz=(0.0, y, -0.095), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=f"{label}_fold_knuckle",
            )
        model.articulation(
            f"{side_name}_extender_slide",
            ArticulationType.PRISMATIC,
            parent=sleeve,
            child=extender,
            origin=Origin(),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.10, lower=0.0, upper=0.040),
        )

        yoke = model.part(f"{side_name}_yoke")
        yoke_parts[side_name] = yoke
        yoke.visual(
            Cylinder(radius=0.012, length=0.046),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="center_fold_barrel",
        )
        yoke.visual(
            Box((0.022, 0.028, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=matte_black,
            name="hinge_neck",
        )
        yoke.visual(
            Box((0.028, 0.215, 0.015)),
            origin=Origin(xyz=(0.0, 0.0, -0.046)),
            material=matte_black,
            name="yoke_bridge",
        )
        for y, label in ((-0.102, "rear"), (0.102, "front")):
            yoke.visual(
                Box((0.023, 0.012, 0.090)),
                origin=Origin(xyz=(0.0, y, -0.092)),
                material=matte_black,
                name=f"{label}_yoke_arm",
            )
            yoke.visual(
                Cylinder(radius=0.014, length=0.014),
                origin=Origin(xyz=(0.0, y, -0.145), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=f"{label}_pivot_boss",
            )
        model.articulation(
            f"{side_name}_fold_hinge",
            ArticulationType.REVOLUTE,
            parent=extender,
            child=yoke,
            origin=Origin(xyz=(0.0, 0.0, -0.095)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.55),
        )

        cup = model.part(f"{side_name}_cup")
        cup_parts[side_name] = cup
        outer_x = side * 0.032
        inner_x = -side * 0.036
        cup.visual(
            Cylinder(radius=0.088, length=0.056),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=matte_black,
            name="cup_shell",
        )
        cup.visual(
            Cylinder(radius=0.070, length=0.008),
            origin=Origin(xyz=(outer_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=gunmetal,
            name="outer_disc",
        )
        cup.visual(
            mesh_from_geometry(TorusGeometry(0.076, 0.004, radial_segments=18, tubular_segments=48), f"{side_name}_outer_ring"),
            origin=Origin(xyz=(outer_x - side * 0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name="outer_accent_ring",
        )
        cup.visual(
            mesh_from_geometry(TorusGeometry(0.064, 0.018, radial_segments=20, tubular_segments=52), f"{side_name}_ear_pad"),
            origin=Origin(xyz=(inner_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=soft_foam,
            name="ear_pad",
        )
        cup.visual(
            Cylinder(radius=0.050, length=0.006),
            origin=Origin(xyz=(inner_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=grille_fabric,
            name="driver_grille",
        )
        cup.visual(
            Cylinder(radius=0.0055, length=0.212),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="pivot_pin",
        )
        cup_parts[side_name] = cup
        model.articulation(
            f"{side_name}_cup_swivel",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -0.145)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for side_name in ("left", "right"):
        sleeve = object_model.get_part(f"{side_name}_sleeve")
        extender = object_model.get_part(f"{side_name}_extender")
        yoke = object_model.get_part(f"{side_name}_yoke")
        cup = object_model.get_part(f"{side_name}_cup")
        slide = object_model.get_articulation(f"{side_name}_extender_slide")

        ctx.expect_within(
            extender,
            sleeve,
            axes="xy",
            inner_elem="slider_strip",
            margin=0.0,
            name=f"{side_name} slider is laterally captured in sleeve",
        )
        ctx.expect_overlap(
            extender,
            sleeve,
            axes="z",
            elem_a="slider_strip",
            min_overlap=0.055,
            name=f"{side_name} collapsed slider remains inserted",
        )

        rest_pos = ctx.part_world_position(extender)
        with ctx.pose({slide: 0.040}):
            ctx.expect_within(
                extender,
                sleeve,
                axes="xy",
                inner_elem="slider_strip",
                margin=0.0,
                name=f"{side_name} extended slider stays inside sleeve",
            )
            ctx.expect_overlap(
                extender,
                sleeve,
                axes="z",
                elem_a="slider_strip",
                min_overlap=0.025,
                name=f"{side_name} extended slider retains insertion",
            )
            extended_pos = ctx.part_world_position(extender)

        ctx.check(
            f"{side_name} size adjuster slides downward",
            rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.030,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        for boss in ("rear_pivot_boss", "front_pivot_boss"):
            ctx.allow_overlap(
                cup,
                yoke,
                elem_a="pivot_pin",
                elem_b=boss,
                reason="The cup trunnion pin is intentionally captured inside the yoke pivot boss.",
            )
            ctx.expect_overlap(
                cup,
                yoke,
                axes="yz",
                elem_a="pivot_pin",
                elem_b=boss,
                min_overlap=0.006,
                name=f"{side_name} cup pin is seated in {boss}",
            )

    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 1.35, right_fold: 1.35}):
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "fold hinges collapse cups inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.050
        and right_folded[0] < right_rest[0] - 0.050,
        details=f"left_rest={left_rest}, left_folded={left_folded}, right_rest={right_rest}, right_folded={right_folded}",
    )

    return ctx.report()


object_model = build_object_model()
