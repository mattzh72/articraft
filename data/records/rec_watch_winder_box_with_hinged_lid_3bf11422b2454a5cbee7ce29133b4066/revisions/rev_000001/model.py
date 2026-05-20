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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_loop(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    lacquer_wood = model.material("lacquer_wood", rgba=(0.30, 0.18, 0.10, 1.0))
    lining = model.material("lining", rgba=(0.58, 0.49, 0.38, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.38, 0.42, 0.45))
    pillow_leather = model.material("pillow_leather", rgba=(0.34, 0.25, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.240, 0.190, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=lacquer_wood,
        name="base_shell",
    )
    body.visual(
        Box((0.240, 0.012, 0.096)),
        origin=Origin(xyz=(0.0, 0.089, 0.062)),
        material=lacquer_wood,
        name="left_wall",
    )
    body.visual(
        Box((0.240, 0.012, 0.096)),
        origin=Origin(xyz=(0.0, -0.089, 0.062)),
        material=lacquer_wood,
        name="right_wall",
    )
    body.visual(
        Box((0.012, 0.166, 0.096)),
        origin=Origin(xyz=(-0.114, 0.0, 0.062)),
        material=lacquer_wood,
        name="rear_wall",
    )
    body.visual(
        Box((0.012, 0.166, 0.050)),
        origin=Origin(xyz=(0.114, 0.0, 0.039)),
        material=lacquer_wood,
        name="front_apron",
    )
    body.visual(
        Box((0.204, 0.154, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=lining,
        name="lining_floor",
    )
    body.visual(
        Box((0.040, 0.080, 0.052)),
        origin=Origin(xyz=(-0.088, 0.0, 0.040)),
        material=lining,
        name="motor_housing",
    )
    body.visual(
        Box((0.024, 0.044, 0.020)),
        origin=Origin(xyz=(-0.056, 0.0, 0.060)),
        material=satin_metal,
        name="spindle_bracket",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(-0.027, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="spindle_shaft",
    )
    body.visual(
        Box((0.014, 0.040, 0.018)),
        origin=Origin(xyz=(-0.120, 0.056, 0.109)),
        material=satin_metal,
        name="hinge_leaf_left",
    )
    body.visual(
        Box((0.014, 0.040, 0.018)),
        origin=Origin(xyz=(-0.120, -0.056, 0.109)),
        material=satin_metal,
        name="hinge_leaf_right",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(-0.126, 0.056, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle_left",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(-0.126, -0.056, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.240, 0.190, 0.110)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.014, 0.070, 0.034)),
        origin=Origin(xyz=(0.007, 0.0, 0.017)),
        material=lacquer_wood,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.232, 0.010, 0.034)),
        origin=Origin(xyz=(0.116, 0.090, 0.017)),
        material=lacquer_wood,
        name="left_skirt",
    )
    lid.visual(
        Box((0.232, 0.010, 0.034)),
        origin=Origin(xyz=(0.116, -0.090, 0.017)),
        material=lacquer_wood,
        name="right_skirt",
    )
    lid.visual(
        Box((0.010, 0.190, 0.034)),
        origin=Origin(xyz=(0.227, 0.0, 0.017)),
        material=lacquer_wood,
        name="front_skirt",
    )
    lid.visual(
        Box((0.220, 0.012, 0.008)),
        origin=Origin(xyz=(0.110, 0.084, 0.034)),
        material=lacquer_wood,
        name="top_left_rail",
    )
    lid.visual(
        Box((0.220, 0.012, 0.008)),
        origin=Origin(xyz=(0.110, -0.084, 0.034)),
        material=lacquer_wood,
        name="top_right_rail",
    )
    lid.visual(
        Box((0.012, 0.166, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.034)),
        material=lacquer_wood,
        name="top_rear_rail",
    )
    lid.visual(
        Box((0.012, 0.166, 0.008)),
        origin=Origin(xyz=(0.224, 0.0, 0.034)),
        material=lacquer_wood,
        name="top_front_rail",
    )
    lid.visual(
        Box((0.206, 0.152, 0.004)),
        origin=Origin(xyz=(0.110, 0.0, 0.030)),
        material=smoked_glass,
        name="window_pane",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle_center",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.240, 0.190, 0.040)),
        mass=0.9,
        origin=Origin(xyz=(0.120, 0.0, 0.020)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.126, 0.0, 0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    cradle = model.part("cradle")
    pillow_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_loop(0.030, 0.058, 0.080, 0.014),
                _yz_loop(0.056, 0.070, 0.092, 0.018),
                _yz_loop(0.082, 0.060, 0.082, 0.015),
            ]
        ),
        "cradle_pillow",
    )
    cradle.visual(
        Box((0.008, 0.108, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=satin_metal,
        name="carrier_upper",
    )
    cradle.visual(
        Box((0.008, 0.108, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=satin_metal,
        name="carrier_lower",
    )
    cradle.visual(
        Box((0.008, 0.012, 0.052)),
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        material=satin_metal,
        name="carrier_left",
    )
    cradle.visual(
        Box((0.008, 0.012, 0.052)),
        origin=Origin(xyz=(0.0, -0.048, 0.0)),
        material=satin_metal,
        name="carrier_right",
    )
    cradle.visual(
        Box((0.032, 0.020, 0.050)),
        origin=Origin(xyz=(0.018, 0.036, 0.0)),
        material=satin_metal,
        name="left_arm",
    )
    cradle.visual(
        Box((0.032, 0.020, 0.050)),
        origin=Origin(xyz=(0.018, -0.036, 0.0)),
        material=satin_metal,
        name="right_arm",
    )
    cradle.visual(
        pillow_mesh,
        material=pillow_leather,
        name="pillow_shell",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.086, 0.080, 0.094)),
        mass=0.35,
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(-0.027, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
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
    body = object_model.get_part("body")
    cradle = object_model.get_part("cradle")
    lid = object_model.get_part("lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="left_skirt",
            negative_elem="left_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid seats on left wall when closed",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: 1.10}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid opens upward",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.060,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    with ctx.pose({cradle_spin: 0.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="yz",
            margin=0.0,
            name="cradle stays within body width and height",
        )
        ctx.expect_gap(
            body,
            cradle,
            axis="x",
            positive_elem="front_apron",
            negative_elem="pillow_shell",
            min_gap=0.015,
            max_gap=0.060,
            name="cradle clears the front apron",
        )

    with ctx.pose({cradle_spin: math.pi / 2.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="yz",
            margin=0.0,
            name="rotated cradle remains within the body opening",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
