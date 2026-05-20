from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _earcup_meshes(prefix: str):
    """Return mesh-backed oval cup elements, oriented with depth along local Y."""
    housing = ExtrudeGeometry(
        superellipse_profile(0.088, 0.112, exponent=2.45, segments=64),
        0.038,
        cap=True,
        center=True,
    ).rotate_x(pi / 2.0)
    cushion = ExtrudeWithHolesGeometry(
        superellipse_profile(0.084, 0.108, exponent=2.2, segments=64),
        [superellipse_profile(0.050, 0.071, exponent=2.0, segments=48)],
        0.020,
        cap=True,
        center=True,
    ).rotate_x(pi / 2.0)
    cloth = ExtrudeGeometry(
        superellipse_profile(0.046, 0.067, exponent=2.0, segments=48),
        0.004,
        cap=True,
        center=True,
    ).rotate_x(pi / 2.0)
    return (
        _mesh(housing, f"{prefix}_housing"),
        _mesh(cushion, f"{prefix}_cushion"),
        _mesh(cloth, f"{prefix}_speaker_cloth"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_over_ear_headphones")

    satin_black = model.material("satin_black", rgba=(0.018, 0.019, 0.021, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.036, 0.038, 1.0))
    soft_foam = model.material("soft_foam", rgba=(0.010, 0.010, 0.011, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.115, 0.12, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_cloth = model.material("dark_cloth", rgba=(0.020, 0.021, 0.024, 1.0))

    band = model.part("band")
    headband_shell = sweep_profile_along_spline(
        [
            (-0.095, 0.0, 0.232),
            (-0.112, 0.0, 0.244),
            (-0.060, 0.0, 0.254),
            (0.000, 0.0, 0.260),
            (0.060, 0.0, 0.254),
            (0.112, 0.0, 0.244),
            (0.095, 0.0, 0.232),
        ],
        profile=rounded_rect_profile(0.030, 0.012, 0.0035, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    band.visual(_mesh(headband_shell, "outer_headband"), material=satin_black, name="outer_headband")

    underside_pad = sweep_profile_along_spline(
        [
            (-0.078, 0.0, 0.219),
            (-0.052, 0.0, 0.236),
            (0.000, 0.0, 0.245),
            (0.052, 0.0, 0.236),
            (0.078, 0.0, 0.219),
        ],
        profile=rounded_rect_profile(0.036, 0.009, 0.004, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    band.visual(_mesh(underside_pad, "head_pad"), material=soft_foam, name="head_pad")

    side_x = (-0.095, 0.095)
    for index, x in enumerate(side_x):
        band.visual(
            Box((0.034, 0.006, 0.080)),
            origin=Origin(xyz=(x, 0.016, 0.194)),
            material=satin_black,
            name=f"sleeve_front_{index}",
        )
        band.visual(
            Box((0.034, 0.006, 0.080)),
            origin=Origin(xyz=(x, -0.016, 0.194)),
            material=satin_black,
            name=f"sleeve_rear_{index}",
        )
        band.visual(
            Box((0.036, 0.038, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.238)),
            material=satin_black,
            name=f"sleeve_top_{index}",
        )
        band.visual(
            Box((0.036, 0.006, 0.010)),
            origin=Origin(xyz=(x, 0.016, 0.153)),
            material=satin_black,
            name=f"sleeve_lip_front_{index}",
        )
        band.visual(
            Box((0.036, 0.006, 0.010)),
            origin=Origin(xyz=(x, -0.016, 0.153)),
            material=satin_black,
            name=f"sleeve_lip_rear_{index}",
        )
        band.visual(
            Box((0.020, 0.003, 0.006)),
            origin=Origin(xyz=(x, 0.0205, 0.205)),
            material=brushed_metal,
            name=f"scale_mark_{index}",
        )

    cup_meshes = [_earcup_meshes(f"earcup_{index}") for index in range(2)]
    hinge_axes = ((0.0, -1.0, 0.0), (0.0, 1.0, 0.0))

    for index, x in enumerate(side_x):
        extension = model.part(f"extension_{index}")
        extension.visual(
            Box((0.014, 0.0265, 0.080)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=brushed_metal,
            name="slider_blade",
        )
        extension.visual(
            Box((0.020, 0.014, 0.036)),
            origin=Origin(xyz=(0.0, 0.0, -0.029)),
            material=brushed_metal,
            name="yoke_stem",
        )
        extension.visual(
            Box((0.037, 0.100, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.047)),
            material=brushed_metal,
            name="yoke_bridge",
        )
        extension.visual(
            Box((0.038, 0.007, 0.044)),
            origin=Origin(xyz=(0.0, 0.046, -0.060)),
            material=brushed_metal,
            name="front_yoke_cheek",
        )
        extension.visual(
            Box((0.038, 0.007, 0.044)),
            origin=Origin(xyz=(0.0, -0.046, -0.060)),
            material=brushed_metal,
            name="rear_yoke_cheek",
        )
        extension.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(xyz=(0.0, 0.052, -0.060), rpy=(pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="front_hinge_cap",
        )
        extension.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(xyz=(0.0, -0.052, -0.060), rpy=(pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="rear_hinge_cap",
        )
        for mark in range(3):
            extension.visual(
                Box((0.012, 0.0018, 0.0035)),
                origin=Origin(xyz=(0.0, 0.0057, 0.030 - mark * 0.017)),
                material=matte_black,
                name=f"detent_mark_{mark}",
            )

        earcup = model.part(f"earcup_{index}")
        housing_mesh, cushion_mesh, cloth_mesh = cup_meshes[index]
        earcup.visual(
            housing_mesh,
            origin=Origin(xyz=(0.0, 0.004, -0.063)),
            material=charcoal,
            name="cup_shell",
        )
        earcup.visual(
            cushion_mesh,
            origin=Origin(xyz=(0.0, -0.027, -0.063)),
            material=soft_foam,
            name="ear_cushion",
        )
        earcup.visual(
            cloth_mesh,
            origin=Origin(xyz=(0.0, -0.039, -0.063)),
            material=dark_cloth,
            name="speaker_cloth",
        )
        earcup.visual(
            Box((0.026, 0.033, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=charcoal,
            name="hinge_lug",
        )
        earcup.visual(
            Cylinder(radius=0.0105, length=0.087),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="hinge_barrel",
        )
        earcup.visual(
            Box((0.052, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, 0.021, -0.109)),
            material=matte_black,
            name="outer_trim",
        )

        model.articulation(
            f"band_to_extension_{index}",
            ArticulationType.PRISMATIC,
            parent=band,
            child=extension,
            origin=Origin(xyz=(x, 0.0, 0.178)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.10, lower=0.0, upper=0.044),
        )
        model.articulation(
            f"extension_to_earcup_{index}",
            ArticulationType.REVOLUTE,
            parent=extension,
            child=earcup,
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
            axis=hinge_axes[index],
            motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=0.0, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    band = object_model.get_part("band")
    for index in range(2):
        extension = object_model.get_part(f"extension_{index}")
        earcup = object_model.get_part(f"earcup_{index}")
        slide = object_model.get_articulation(f"band_to_extension_{index}")
        fold = object_model.get_articulation(f"extension_to_earcup_{index}")
        sleeve = f"sleeve_front_{index}"

        ctx.expect_overlap(
            extension,
            band,
            axes="xz",
            elem_a="slider_blade",
            elem_b=sleeve,
            min_overlap=0.010,
            name=f"extension_{index} starts inside sleeve",
        )
        ctx.expect_overlap(
            earcup,
            extension,
            axes="y",
            elem_a="hinge_barrel",
            elem_b="front_yoke_cheek",
            min_overlap=0.0005,
            name=f"earcup_{index} hinge is captured by yoke",
        )

        rest_pos = ctx.part_world_position(extension)
        with ctx.pose({slide: 0.044}):
            ctx.expect_overlap(
                extension,
                band,
                axes="z",
                elem_a="slider_blade",
                elem_b=sleeve,
                min_overlap=0.020,
                name=f"extension_{index} remains inserted when extended",
            )
            extended_pos = ctx.part_world_position(extension)
        ctx.check(
            f"extension_{index} slides downward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] < rest_pos[2] - 0.035,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        rest_aabb = ctx.part_element_world_aabb(earcup, elem="cup_shell")
        rest_x = _aabb_center_x(rest_aabb)
        with ctx.pose({fold: 0.70}):
            folded_aabb = ctx.part_element_world_aabb(earcup, elem="cup_shell")
            folded_x = _aabb_center_x(folded_aabb)
        ctx.check(
            f"earcup_{index} folds inward",
            rest_x is not None
            and folded_x is not None
            and abs(folded_x) < abs(rest_x) - 0.035,
            details=f"rest_x={rest_x}, folded_x={folded_x}",
        )

    with ctx.pose(
        {
            object_model.get_articulation("extension_to_earcup_0"): 0.70,
            object_model.get_articulation("extension_to_earcup_1"): 0.70,
        }
    ):
        ctx.expect_gap(
            object_model.get_part("earcup_1"),
            object_model.get_part("earcup_0"),
            axis="x",
            min_gap=0.001,
            name="folded earcups clear each other",
        )

    return ctx.report()


object_model = build_object_model()
