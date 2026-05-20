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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skeleton_metronome")

    walnut = model.material("walnut", rgba=(0.40, 0.24, 0.12, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    nickel = model.material("nickel", rgba=(0.78, 0.80, 0.82, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base_width = 0.170
    base_depth = 0.090
    base_thickness = 0.014
    post_width = 0.012
    post_depth = 0.016
    post_height = 0.148
    post_x = 0.058
    crossbar_length = 0.128
    crossbar_depth = 0.018
    crossbar_thickness = 0.012
    pivot_z = 0.154
    pendulum_plane_y = -0.014

    frame = model.part("frame")
    frame.visual(
        Box((base_width, base_depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=walnut,
        name="elem_base_plate",
    )
    frame.visual(
        Box((0.030, 0.030, 0.010)),
        origin=Origin(xyz=(-post_x, 0.0, 0.011)),
        material=walnut,
        name="elem_left_post_foot",
    )
    frame.visual(
        Box((0.030, 0.030, 0.010)),
        origin=Origin(xyz=(post_x, 0.0, 0.011)),
        material=walnut,
        name="elem_right_post_foot",
    )
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(-post_x, 0.0, 0.084)),
        material=blackened_steel,
        name="elem_left_post",
    )
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(post_x, 0.0, 0.084)),
        material=blackened_steel,
        name="elem_right_post",
    )
    frame.visual(
        Box((crossbar_length, crossbar_depth, crossbar_thickness)),
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        material=blackened_steel,
        name="elem_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.0020, length=0.026),
        origin=Origin(xyz=(0.0, pendulum_plane_y, pivot_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elem_pivot_pin",
    )
    frame.visual(
        Box((0.050, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.002)),
        material=brushed_steel,
        name="elem_rate_scale",
    )

    pendulum = model.part("pendulum")
    eyelet_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.0048, -0.0035), (0.0048, 0.0035)],
            [(0.0028, -0.0035), (0.0028, 0.0035)],
            segments=40,
        ),
        "pendulum_eyelet",
    )
    pendulum.visual(
        eyelet_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elem_eyelet",
    )
    pendulum.visual(
        Box((0.004, 0.0025, 0.235)),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=brushed_steel,
        name="elem_pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.118), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="elem_bob",
    )
    pendulum.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.086), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elem_top_stop",
    )

    slider_weight = model.part("slider_weight")
    weight_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0085, -0.013),
                (0.0108, -0.009),
                (0.0112, 0.0),
                (0.0108, 0.009),
                (0.0085, 0.013),
            ],
            [(0.0034, -0.013), (0.0034, 0.013)],
            segments=48,
        ),
        "slider_weight_collar",
    )
    slider_weight.visual(
        weight_mesh,
        material=nickel,
        name="elem_weight_collar",
    )
    slider_weight.visual(
        Box((0.008, 0.004, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=blackened_steel,
        name="elem_weight_lock",
    )

    winding_knob = model.part("winding_knob")
    winding_knob.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elem_knob_stem",
    )
    winding_knob.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="elem_knob_body",
    )
    winding_knob.visual(
        Box((0.005, 0.004, 0.010)),
        origin=Origin(xyz=(0.008, -0.018, 0.0)),
        material=knob_black,
        name="elem_knob_grip",
    )

    model.articulation(
        "frame_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pendulum,
        origin=Origin(xyz=(0.0, pendulum_plane_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "pendulum_to_slider_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=0.08,
            lower=0.0,
            upper=0.030,
        ),
    )
    model.articulation(
        "frame_to_winding_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=winding_knob,
        origin=Origin(xyz=(0.0, -(base_depth * 0.5), base_thickness * 0.5)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=8.0,
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

    frame = object_model.get_part("frame")
    pendulum = object_model.get_part("pendulum")
    slider_weight = object_model.get_part("slider_weight")
    winding_knob = object_model.get_part("winding_knob")

    pendulum_joint = object_model.get_articulation("frame_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_slider_weight")
    knob_joint = object_model.get_articulation("frame_to_winding_knob")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "pendulum is revolute",
        pendulum_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pendulum_joint.articulation_type}",
    )
    ctx.check(
        "slider weight is prismatic",
        weight_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={weight_joint.articulation_type}",
    )
    ctx.check(
        "winding knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    ctx.expect_contact(
        winding_knob,
        frame,
        elem_a="elem_knob_stem",
        elem_b="elem_base_plate",
        contact_tol=5e-4,
        name="winding knob stem seats against base plate",
    )
    ctx.expect_gap(
        pendulum,
        frame,
        axis="z",
        positive_elem="elem_bob",
        negative_elem="elem_base_plate",
        min_gap=0.004,
        max_gap=0.010,
        name="pendulum bob clears base plate at rest",
    )
    ctx.expect_origin_distance(
        slider_weight,
        pendulum,
        axes="xy",
        max_dist=1e-4,
        name="slider weight stays centered on pendulum rod at rest",
    )
    ctx.expect_overlap(
        slider_weight,
        pendulum,
        axes="z",
        elem_a="elem_weight_collar",
        elem_b="elem_pendulum_rod",
        min_overlap=0.020,
        name="slider weight remains engaged on pendulum rod at rest",
    )

    weight_rest = ctx.part_world_position(slider_weight)
    bob_rest = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="elem_bob"))

    weight_upper = weight_joint.motion_limits.upper if weight_joint.motion_limits is not None else None
    if weight_upper is not None:
        with ctx.pose({weight_joint: weight_upper}):
            ctx.expect_origin_distance(
                slider_weight,
                pendulum,
                axes="xy",
                max_dist=1e-4,
                name="slider weight stays centered on pendulum rod when raised",
            )
            ctx.expect_overlap(
                slider_weight,
                pendulum,
                axes="z",
                elem_a="elem_weight_collar",
                elem_b="elem_pendulum_rod",
                min_overlap=0.020,
                name="slider weight remains engaged on pendulum rod when raised",
            )
            weight_high = ctx.part_world_position(slider_weight)
        ctx.check(
            "slider weight moves upward with positive travel",
            weight_rest is not None
            and weight_high is not None
            and weight_high[2] > weight_rest[2] + 0.020,
            details=f"rest={weight_rest}, raised={weight_high}",
        )

    with ctx.pose({pendulum_joint: 0.20}):
        bob_swung = _aabb_center(ctx.part_element_world_aabb(pendulum, elem="elem_bob"))
    ctx.check(
        "positive pendulum angle swings bob toward +x",
        bob_rest is not None and bob_swung is not None and bob_swung[0] > bob_rest[0] + 0.015,
        details=f"rest={bob_rest}, swung={bob_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
