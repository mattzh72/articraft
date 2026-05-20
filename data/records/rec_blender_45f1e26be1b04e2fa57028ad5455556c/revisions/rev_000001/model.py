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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ice_shaver")

    stainless = model.material("stainless", rgba=(0.77, 0.80, 0.82, 1.0))
    dark_body = model.material("dark_body", rgba=(0.17, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.85, 0.87, 0.90, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.28, 0.24, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=stainless,
        name="base_plinth",
    )
    housing.visual(
        Box((0.012, 0.228, 0.15)),
        origin=Origin(xyz=(-0.134, 0.0, 0.115)),
        material=stainless,
        name="left_wall",
    )
    housing.visual(
        Box((0.012, 0.228, 0.15)),
        origin=Origin(xyz=(0.134, 0.0, 0.115)),
        material=stainless,
        name="right_wall",
    )
    housing.visual(
        Box((0.256, 0.012, 0.15)),
        origin=Origin(xyz=(0.0, -0.114, 0.115)),
        material=stainless,
        name="rear_wall",
    )
    housing.visual(
        Box((0.256, 0.012, 0.07)),
        origin=Origin(xyz=(0.0, 0.114, 0.075)),
        material=stainless,
        name="front_lower_wall",
    )
    housing.visual(
        Box((0.256, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.114, 0.181)),
        material=stainless,
        name="front_rim",
    )
    housing.visual(
        Box((0.10, 0.06, 0.012)),
        origin=Origin(xyz=(0.0, 0.150, 0.055)),
        material=stainless,
        name="ice_chute",
    )
    housing.visual(
        Box((0.028, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, -0.135, 0.290)),
        material=dark_body,
        name="guide_column",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_body,
        name="blade_spindle_support",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.28, 0.24, 0.40)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.092, 0.134, 0.010)),
        origin=Origin(xyz=(-0.086, 0.117, 0.005)),
        material=stainless,
        name="lid_panel",
    )
    lid.visual(
        Box((0.092, 0.134, 0.010)),
        origin=Origin(xyz=(0.086, 0.117, 0.005)),
        material=stainless,
        name="right_lid_cover",
    )
    lid.visual(
        Box((0.264, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.171, 0.005)),
        material=stainless,
        name="front_lid_bridge",
    )
    lid.visual(
        Box((0.010, 0.160, 0.024)),
        origin=Origin(xyz=(-0.127, 0.082, -0.007)),
        material=stainless,
        name="left_lid_skirt",
    )
    lid.visual(
        Box((0.010, 0.160, 0.024)),
        origin=Origin(xyz=(0.127, 0.082, -0.007)),
        material=stainless,
        name="right_lid_skirt",
    )
    lid.visual(
        Box((0.18, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.175, -0.002)),
        material=dark_body,
        name="front_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.264, 0.184, 0.028)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.092, 0.004)),
    )

    press_arm = model.part("press_arm")
    press_arm.visual(
        Box((0.012, 0.024, 0.078)),
        origin=Origin(xyz=(-0.021, 0.012, -0.038)),
        material=dark_body,
        name="left_slider",
    )
    press_arm.visual(
        Box((0.012, 0.024, 0.078)),
        origin=Origin(xyz=(0.021, 0.012, -0.038)),
        material=dark_body,
        name="right_slider",
    )
    press_arm.visual(
        Box((0.054, 0.016, 0.078)),
        origin=Origin(xyz=(0.0, 0.024, -0.038)),
        material=dark_body,
        name="front_bridge",
    )
    press_arm.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(xyz=(0.0, 0.024, 0.017)),
        material=dark_body,
        name="handle_post",
    )
    press_arm.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.024, 0.049)),
        material=rubber,
        name="press_knob",
    )
    press_arm.visual(
        Box((0.020, 0.016, 0.116)),
        origin=Origin(xyz=(0.0, 0.040, -0.135)),
        material=dark_body,
        name="drop_link",
    )
    press_arm.visual(
        Box((0.060, 0.092, 0.014)),
        origin=Origin(xyz=(0.0, 0.094, -0.200)),
        material=dark_body,
        name="press_plate",
    )
    press_arm.visual(
        Cylinder(radius=0.048, length=0.015),
        origin=Origin(xyz=(0.0, 0.135, -0.2025)),
        material=rubber,
        name="press_pad",
    )
    press_arm.inertial = Inertial.from_geometry(
        Box((0.07, 0.22, 0.25)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.08, -0.10)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.055, length=0.008),
        material=blade_metal,
        name="blade_disk",
    )
    blade.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_body,
        name="blade_hub",
    )
    blade.visual(
        Box((0.085, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=blade_metal,
        name="blade_fin_a",
    )
    blade.visual(
        Box((0.085, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, 0.0, pi / 2.0)),
        material=blade_metal,
        name="blade_fin_b",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.02),
        mass=0.25,
    )

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, -0.100, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "housing_to_press_arm",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=press_arm,
        origin=Origin(xyz=(0.0, -0.135, 0.380)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.09,
        ),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=40.0,
        ),
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

    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    press_arm = object_model.get_part("press_arm")
    blade = object_model.get_part("blade")

    lid_hinge = object_model.get_articulation("housing_to_lid")
    press_slide = object_model.get_articulation("housing_to_press_arm")
    blade_spin = object_model.get_articulation("housing_to_blade")

    ctx.expect_overlap(
        lid,
        housing,
        axes="xy",
        min_overlap=0.18,
        name="closed lid covers the loading opening",
    )
    ctx.expect_gap(
        press_arm,
        blade,
        axis="z",
        positive_elem="press_pad",
        negative_elem="blade_disk",
        min_gap=0.09,
        name="raised press pad clears the blade",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward for top loading",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.09,
        details=f"closed={closed_lid_aabb}, open={opened_lid_aabb}",
    )

    rest_pad = ctx.part_element_world_aabb(press_arm, elem="press_pad")
    with ctx.pose({press_slide: 0.09}):
        lowered_pad = ctx.part_element_world_aabb(press_arm, elem="press_pad")
        ctx.expect_overlap(
            press_arm,
            blade,
            axes="xy",
            elem_a="press_pad",
            elem_b="blade_disk",
            min_overlap=0.09,
            name="lowered press pad stays centered over the blade",
        )
        ctx.expect_gap(
            press_arm,
            blade,
            axis="z",
            positive_elem="press_pad",
            negative_elem="blade_disk",
            min_gap=0.01,
            max_gap=0.03,
            name="lowered press pad stops just above the blade",
        )
    ctx.check(
        "press arm descends along the guide column",
        rest_pad is not None
        and lowered_pad is not None
        and lowered_pad[0][2] < rest_pad[0][2] - 0.07,
        details=f"rest={rest_pad}, lowered={lowered_pad}",
    )

    blade_limits = blade_spin.motion_limits
    ctx.check(
        "blade uses continuous vertical rotation",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 0.0, 1.0)
        and blade_limits is not None
        and blade_limits.lower is None
        and blade_limits.upper is None,
        details=(
            f"type={blade_spin.articulation_type}, axis={blade_spin.axis}, "
            f"limits={blade_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
