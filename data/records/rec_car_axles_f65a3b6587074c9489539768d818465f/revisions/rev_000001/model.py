from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _add_hub_geometry(part, *, side_sign: float, hub_material, cap_material) -> None:
    spindle_length = 0.090
    part.visual(
        Cylinder(radius=0.046, length=spindle_length),
        origin=Origin(
            xyz=(side_sign * spindle_length * 0.5, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub_material,
        name="hub_spindle",
    )
    part.visual(
        Cylinder(radius=0.096, length=0.024),
        origin=Origin(
            xyz=(side_sign * 0.102, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hub_material,
        name="hub_flange",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.048),
        origin=Origin(
            xyz=(side_sign * 0.056, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=cap_material,
        name="grease_cap",
    )


def _add_saddle_geometry(part, *, steel, accent) -> None:
    part.visual(
        Box((0.130, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="saddle_base",
    )
    part.visual(
        Box((0.022, 0.170, 0.028)),
        origin=Origin(xyz=(-0.044, 0.0, 0.032)),
        material=steel,
        name="left_guide",
    )
    part.visual(
        Box((0.022, 0.170, 0.028)),
        origin=Origin(xyz=(0.044, 0.0, 0.032)),
        material=steel,
        name="right_guide",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=accent,
        name="center_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rear_beam_axle")

    axle_paint = model.material("axle_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    cap_black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))

    axle_housing = model.part("axle_housing")
    axle_housing.visual(
        Box((1.440, 0.180, 0.180)),
        material=axle_paint,
        name="axle_beam",
    )
    axle_housing.visual(
        Cylinder(radius=0.072, length=0.140),
        origin=Origin(xyz=(-0.720, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_paint,
        name="left_bearing_sleeve",
    )
    axle_housing.visual(
        Cylinder(radius=0.072, length=0.140),
        origin=Origin(xyz=(0.720, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_paint,
        name="right_bearing_sleeve",
    )
    axle_housing.visual(
        Box((0.360, 0.200, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0625)),
        material=axle_paint,
        name="center_reinforcement",
    )
    axle_housing.inertial = Inertial.from_geometry(
        Box((1.580, 0.220, 0.240)),
        mass=115.0,
    )

    left_hub = model.part("left_hub")
    _add_hub_geometry(
        left_hub,
        side_sign=-1.0,
        hub_material=machined_steel,
        cap_material=cap_black,
    )
    left_hub.inertial = Inertial.from_geometry(
        Box((0.120, 0.200, 0.200)),
        mass=18.0,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
    )

    right_hub = model.part("right_hub")
    _add_hub_geometry(
        right_hub,
        side_sign=1.0,
        hub_material=machined_steel,
        cap_material=cap_black,
    )
    right_hub.inertial = Inertial.from_geometry(
        Box((0.120, 0.200, 0.200)),
        mass=18.0,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    left_saddle = model.part("left_saddle")
    _add_saddle_geometry(left_saddle, steel=axle_paint, accent=machined_steel)
    left_saddle.inertial = Inertial.from_geometry(
        Box((0.130, 0.170, 0.050)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    right_saddle = model.part("right_saddle")
    _add_saddle_geometry(right_saddle, steel=axle_paint, accent=machined_steel)
    right_saddle.inertial = Inertial.from_geometry(
        Box((0.130, 0.170, 0.050)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=left_hub,
        origin=Origin(xyz=(-0.790, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=40.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_housing,
        child=right_hub,
        origin=Origin(xyz=(0.790, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=40.0),
    )
    model.articulation(
        "left_saddle_mount",
        ArticulationType.FIXED,
        parent=axle_housing,
        child=left_saddle,
        origin=Origin(xyz=(-0.440, 0.0, 0.090)),
    )
    model.articulation(
        "right_saddle_mount",
        ArticulationType.FIXED,
        parent=axle_housing,
        child=right_saddle,
        origin=Origin(xyz=(0.440, 0.0, 0.090)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    axle_housing = object_model.get_part("axle_housing")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")
    left_saddle = object_model.get_part("left_saddle")
    right_saddle = object_model.get_part("right_saddle")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    ctx.check(
        "hub spins use continuous bearings",
        left_hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(abs(left_hub_spin.axis[0]) - 1.0) < 1e-9
        and abs(abs(right_hub_spin.axis[0]) - 1.0) < 1e-9
        and abs(left_hub_spin.axis[1]) < 1e-9
        and abs(left_hub_spin.axis[2]) < 1e-9
        and abs(right_hub_spin.axis[1]) < 1e-9
        and abs(right_hub_spin.axis[2]) < 1e-9,
        details=(
            f"left_type={left_hub_spin.articulation_type}, left_axis={left_hub_spin.axis}, "
            f"right_type={right_hub_spin.articulation_type}, right_axis={right_hub_spin.axis}"
        ),
    )

    ctx.expect_gap(
        left_saddle,
        axle_housing,
        axis="z",
        positive_elem="saddle_base",
        negative_elem="axle_beam",
        max_gap=0.001,
        max_penetration=0.0,
        name="left saddle sits on top of the axle beam",
    )
    ctx.expect_overlap(
        left_saddle,
        axle_housing,
        axes="xy",
        elem_a="saddle_base",
        elem_b="axle_beam",
        min_overlap=0.120,
        name="left saddle is planted on the beam footprint",
    )
    ctx.expect_gap(
        right_saddle,
        axle_housing,
        axis="z",
        positive_elem="saddle_base",
        negative_elem="axle_beam",
        max_gap=0.001,
        max_penetration=0.0,
        name="right saddle sits on top of the axle beam",
    )
    ctx.expect_overlap(
        right_saddle,
        axle_housing,
        axes="xy",
        elem_a="saddle_base",
        elem_b="axle_beam",
        min_overlap=0.120,
        name="right saddle is planted on the beam footprint",
    )

    ctx.expect_gap(
        axle_housing,
        left_hub,
        axis="x",
        positive_elem="left_bearing_sleeve",
        negative_elem="hub_spindle",
        max_gap=0.001,
        max_penetration=0.0,
        name="left hub seats against the left bearing sleeve",
    )
    ctx.expect_overlap(
        axle_housing,
        left_hub,
        axes="yz",
        elem_a="left_bearing_sleeve",
        elem_b="hub_spindle",
        min_overlap=0.090,
        name="left hub spindle stays coaxial with the sleeve",
    )
    ctx.expect_gap(
        right_hub,
        axle_housing,
        axis="x",
        positive_elem="hub_spindle",
        negative_elem="right_bearing_sleeve",
        max_gap=0.001,
        max_penetration=0.0,
        name="right hub seats against the right bearing sleeve",
    )
    ctx.expect_overlap(
        axle_housing,
        right_hub,
        axes="yz",
        elem_a="right_bearing_sleeve",
        elem_b="hub_spindle",
        min_overlap=0.090,
        name="right hub spindle stays coaxial with the sleeve",
    )

    ctx.expect_origin_distance(
        left_saddle,
        right_saddle,
        axes="x",
        min_dist=0.80,
        max_dist=0.92,
        name="leaf spring saddles are widely spaced across the axle",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
