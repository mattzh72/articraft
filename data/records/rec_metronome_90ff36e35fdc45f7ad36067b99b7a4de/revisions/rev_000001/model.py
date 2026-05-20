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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_inverted_pendulum_metronome")

    housing_width = 0.180
    housing_depth = 0.092
    housing_height = 0.032
    lid_width = 0.164
    lid_depth = 0.076
    lid_height = 0.006
    foot_size = 0.018
    foot_height = 0.012

    pendulum_rod_radius = 0.0018
    pendulum_rod_length = 0.190
    pendulum_travel = 0.080
    weight_outer_radius = 0.012
    weight_inner_radius = pendulum_rod_radius
    weight_length = 0.028

    housing_mat = Material(name="housing_ivory", rgba=(0.91, 0.88, 0.78, 1.0))
    trim_mat = Material(name="trim_warm_gray", rgba=(0.52, 0.50, 0.47, 1.0))
    steel_mat = Material(name="steel", rgba=(0.72, 0.74, 0.77, 1.0))
    brass_mat = Material(name="brass", rgba=(0.80, 0.67, 0.28, 1.0))
    dark_mat = Material(name="dark_rubber", rgba=(0.18, 0.18, 0.18, 1.0))
    model.materials.extend((housing_mat, trim_mat, steel_mat, brass_mat, dark_mat))

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, housing_height)),
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0)),
        material=housing_mat,
        name="housing_shell",
    )
    housing.visual(
        Box((lid_width, lid_depth, lid_height)),
        origin=Origin(xyz=(0.0, 0.0, housing_height + lid_height / 2.0)),
        material=trim_mat,
        name="top_cap",
    )
    def add_foot(name: str, foot_x: float, foot_y: float) -> None:
        housing.visual(
            Box((foot_size, foot_size, foot_height)),
            origin=Origin(
                xyz=(
                    foot_x * (housing_width / 2.0 - foot_size / 2.0 - 0.010),
                    foot_y * (housing_depth / 2.0 - foot_size / 2.0 - 0.008),
                    -foot_height / 2.0,
                )
            ),
            material=dark_mat,
            name=name,
        )

    add_foot("foot_lb", -1.0, -1.0)
    add_foot("foot_lf", -1.0, 1.0)
    add_foot("foot_rb", 1.0, -1.0)
    add_foot("foot_rf", 1.0, 1.0)
    housing.visual(
        Box((0.024, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_mat,
        name="pivot_block",
    )
    housing.visual(
        Box((0.010, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.011, -0.007)),
        material=trim_mat,
        name="pivot_cheek_front",
    )
    housing.visual(
        Box((0.010, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.011, -0.007)),
        material=trim_mat,
        name="pivot_cheek_back",
    )
    housing.visual(
        Cylinder(radius=0.0025, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="pivot_shaft",
    )
    housing.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, housing_height + 0.002)),
        material=trim_mat,
        name="key_bushing",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Box((0.012, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel_mat,
        name="hanger_crossbar",
    )
    pendulum.visual(
        Box((0.006, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, -0.010)),
        material=steel_mat,
        name="hanger_front_ear",
    )
    pendulum.visual(
        Box((0.006, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.011, -0.010)),
        material=steel_mat,
        name="hanger_back_ear",
    )
    pendulum.visual(
        Cylinder(radius=pendulum_rod_radius, length=pendulum_rod_length),
        origin=Origin(xyz=(0.0, 0.0, -(0.011 + pendulum_rod_length / 2.0))),
        material=steel_mat,
        name="pendulum_rod",
    )
    pendulum.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0, -(0.011 + pendulum_rod_length + 0.004))),
        material=steel_mat,
        name="rod_tip",
    )

    weight_profile_outer = [
        (0.0105, -weight_length / 2.0),
        (weight_outer_radius, -0.011),
        (weight_outer_radius, 0.011),
        (0.0105, weight_length / 2.0),
    ]
    weight_profile_inner = [
        (weight_inner_radius, -weight_length / 2.0),
        (weight_inner_radius, weight_length / 2.0),
    ]
    weight_sleeve = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            weight_profile_outer,
            weight_profile_inner,
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ),
        "tempo_weight_sleeve",
    )

    weight = model.part("weight")
    weight.visual(
        weight_sleeve,
        material=brass_mat,
        name="weight_sleeve",
    )
    weight.visual(
        Box((0.005, 0.008, 0.006)),
        origin=Origin(xyz=(weight_outer_radius + 0.001, 0.0, 0.0)),
        material=brass_mat,
        name="weight_thumb",
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel_mat,
        name="key_shaft",
    )
    key.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel_mat,
        name="key_hub",
    )
    key.visual(
        Box((0.030, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel_mat,
        name="key_wing_x",
    )
    key.visual(
        Box((0.006, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel_mat,
        name="key_wing_y",
    )

    pendulum_joint = model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.08,
            lower=0.0,
            upper=pendulum_travel,
        ),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.0, 0.0, housing_height + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
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
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    ctx.check(
        "pendulum uses underside revolute hinge",
        pendulum_joint.articulation_type == ArticulationType.REVOLUTE
        and pendulum_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={pendulum_joint.articulation_type}, axis={pendulum_joint.axis}",
    )
    ctx.check(
        "weight uses rod-aligned slider",
        weight_joint.articulation_type == ArticulationType.PRISMATIC
        and weight_joint.axis == (0.0, 0.0, -1.0),
        details=f"type={weight_joint.articulation_type}, axis={weight_joint.axis}",
    )
    ctx.check(
        "winding key rotates continuously",
        key_joint.articulation_type == ArticulationType.CONTINUOUS
        and key_joint.motion_limits is not None
        and key_joint.motion_limits.lower is None
        and key_joint.motion_limits.upper is None
        and key_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"type={key_joint.articulation_type}, axis={key_joint.axis}, "
            f"limits={key_joint.motion_limits}"
        ),
    )

    with ctx.pose({pendulum_joint: 0.0, weight_joint: 0.0}):
        ctx.expect_gap(
            housing,
            pendulum,
            axis="z",
            min_gap=0.0005,
            max_gap=0.012,
            positive_elem="housing_shell",
            negative_elem="hanger_crossbar",
            name="pendulum hangs just below underside pivot",
        )
        ctx.expect_within(
            pendulum,
            housing,
            axes="xy",
            margin=0.010,
            inner_elem="pendulum_rod",
            outer_elem="housing_shell",
            name="pendulum stays within the housing footprint at rest",
        )
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="xy",
            min_overlap=0.003,
            elem_a="weight_sleeve",
            elem_b="pendulum_rod",
            name="weight remains centered around the pendulum rod",
        )
        ctx.expect_gap(
            key,
            housing,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="key_shaft",
            negative_elem="key_bushing",
            name="winding key rises from the housing top",
        )

        rest_weight_pos = ctx.part_world_position(weight)
        rest_bob_box = ctx.part_element_world_aabb(pendulum, elem="rod_tip")

    with ctx.pose({pendulum_joint: 0.24}):
        swung_right_bob_box = ctx.part_element_world_aabb(pendulum, elem="rod_tip")

    with ctx.pose({pendulum_joint: -0.24}):
        swung_left_bob_box = ctx.part_element_world_aabb(pendulum, elem="rod_tip")

    weight_upper = 0.0
    if weight_joint.motion_limits is not None and weight_joint.motion_limits.upper is not None:
        weight_upper = weight_joint.motion_limits.upper

    pendulum_upper = 0.24
    if pendulum_joint.motion_limits is not None and pendulum_joint.motion_limits.upper is not None:
        pendulum_upper = pendulum_joint.motion_limits.upper

    with ctx.pose({pendulum_joint: 0.0, weight_joint: weight_upper}):
        low_weight_pos = ctx.part_world_position(weight)

    with ctx.pose({pendulum_joint: pendulum_upper, weight_joint: weight_upper}):
        ctx.expect_gap(
            weight,
            housing,
            axis="x",
            min_gap=0.003,
            positive_elem="weight_sleeve",
            negative_elem="foot_lf",
            name="weight clears the left-side feet at full left swing",
        )

    with ctx.pose({pendulum_joint: -pendulum_upper, weight_joint: weight_upper}):
        ctx.expect_gap(
            housing,
            weight,
            axis="x",
            min_gap=0.003,
            positive_elem="foot_rf",
            negative_elem="weight_sleeve",
            name="weight clears the right-side feet at full right swing",
        )

    def aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    rest_bob_x = aabb_center_x(rest_bob_box)
    swung_right_bob_x = aabb_center_x(swung_right_bob_box)
    swung_left_bob_x = aabb_center_x(swung_left_bob_box)

    ctx.check(
        "pendulum swings side to side between poses",
        rest_bob_x is not None
        and swung_right_bob_x is not None
        and swung_left_bob_x is not None
        and swung_left_bob_x > rest_bob_x + 0.015
        and swung_right_bob_x < rest_bob_x - 0.015,
        details=(
            f"rest_x={rest_bob_x}, left_pose_x={swung_left_bob_x}, "
            f"right_pose_x={swung_right_bob_x}"
        ),
    )
    ctx.check(
        "slider weight moves downward to lengthen pendulum",
        rest_weight_pos is not None
        and low_weight_pos is not None
        and low_weight_pos[2] < rest_weight_pos[2] - 0.060,
        details=f"rest={rest_weight_pos}, lowered={low_weight_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
