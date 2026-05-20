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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_metronome")

    def rect_loop(width: float, height: float) -> list[tuple[float, float]]:
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (-half_w, -half_h),
            (half_w, -half_h),
            (half_w, half_h),
            (-half_w, half_h),
        ]

    housing_body = model.material("housing_body", rgba=(0.34, 0.23, 0.15, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.18, 0.12, 0.08, 1.0))
    pendulum_steel = model.material("pendulum_steel", rgba=(0.78, 0.78, 0.76, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.66, 0.30, 1.0))
    dark_detail = model.material("dark_detail", rgba=(0.12, 0.12, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.160, 0.072, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=housing_trim,
        name="base_plinth",
    )
    housing.visual(
        Box((0.010, 0.064, 0.050)),
        origin=Origin(xyz=(-0.066, 0.0, 0.028), rpy=(0.0, 0.12, 0.0)),
        material=housing_body,
        name="left_side_shell",
    )
    housing.visual(
        Box((0.010, 0.064, 0.050)),
        origin=Origin(xyz=(0.066, 0.0, 0.028), rpy=(0.0, -0.12, 0.0)),
        material=housing_body,
        name="right_side_shell",
    )
    housing.visual(
        Box((0.136, 0.008, 0.048)),
        origin=Origin(xyz=(0.0, 0.031, 0.030), rpy=(0.08, 0.0, 0.0)),
        material=housing_body,
        name="rear_shell",
    )
    housing.visual(
        Box((0.128, 0.058, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=housing_body,
        name="top_plate",
    )
    housing.visual(
        Box((0.126, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -0.031, 0.011)),
        material=housing_body,
        name="front_lower_bezel",
    )
    housing.visual(
        Box((0.126, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.031, 0.045)),
        material=housing_body,
        name="front_upper_bezel",
    )
    housing.visual(
        Box((0.014, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.028)),
        material=dark_detail,
        name="rear_pivot_carrier",
    )
    housing.visual(
        Box((0.028, 0.016, 0.002)),
        origin=Origin(xyz=(-0.046, 0.014, 0.001)),
        material=dark_detail,
        name="left_foot",
    )
    housing.visual(
        Box((0.028, 0.016, 0.002)),
        origin=Origin(xyz=(0.046, 0.014, 0.001)),
        material=dark_detail,
        name="right_foot",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.160, 0.072, 0.058)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_detail,
        name="pivot_shaft",
    )
    pendulum.visual(
        Box((0.006, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=dark_detail,
        name="connector_arm",
    )
    pendulum.visual(
        Box((0.004, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, -0.030, 0.050)),
        material=pendulum_steel,
        name="rod_strip",
    )
    pendulum.visual(
        Box((0.022, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.017, -0.018)),
        material=brass,
        name="lower_counterweight",
    )
    pendulum.visual(
        Box((0.006, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.024, -0.018)),
        material=brass,
        name="counterweight_bridge",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.024, 0.030, 0.160)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.020, 0.040)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=5.0,
            lower=-0.65,
            upper=0.65,
        ),
    )

    slider_weight = model.part("weight")
    weight_geom = ExtrudeWithHolesGeometry(
        rect_loop(0.026, 0.016),
        [rect_loop(0.0065, 0.0135)],
        0.005,
    ).rotate_x(math.pi / 2.0)
    slider_weight.visual(
        mesh_from_geometry(weight_geom, "metronome_slider_weight"),
        material=brass,
        name="slider_weight",
    )
    slider_weight.inertial = Inertial.from_geometry(
        Box((0.026, 0.005, 0.016)),
        mass=0.025,
    )

    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider_weight,
        origin=Origin(xyz=(0.0, -0.030, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.010, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00125)),
        material=brass,
        name="key_flange",
    )
    winding_key.visual(
        Box((0.024, 0.005, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0026)),
        material=brass,
        name="key_wing",
    )
    winding_key.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.0045),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.00225)),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=10.0),
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
    slider_weight = object_model.get_part("weight")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    slider_upper = slider_joint.motion_limits.upper or 0.0

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        housing,
        pendulum,
        axis="z",
        positive_elem="front_upper_bezel",
        negative_elem="connector_arm",
        min_gap=0.006,
        max_gap=0.014,
        name="pendulum arm clears the upper slot lip",
    )
    ctx.expect_gap(
        pendulum,
        housing,
        axis="z",
        positive_elem="connector_arm",
        negative_elem="front_lower_bezel",
        min_gap=0.004,
        max_gap=0.012,
        name="pendulum arm clears the lower slot lip",
    )
    ctx.expect_gap(
        winding_key,
        housing,
        axis="z",
        positive_elem="key_flange",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="winding key sits flush on the top face",
    )

    with ctx.pose({key_joint: math.pi / 2.0}):
        ctx.expect_gap(
            winding_key,
            housing,
            axis="z",
            positive_elem="key_flange",
            negative_elem="top_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="winding key stays flush while turning",
        )

    rest_weight_pos = ctx.part_world_position(slider_weight)
    with ctx.pose({slider_joint: slider_upper}):
        raised_weight_pos = ctx.part_world_position(slider_weight)

    ctx.check(
        "sliding weight moves upward along the pendulum rod",
        rest_weight_pos is not None
        and raised_weight_pos is not None
        and raised_weight_pos[2] > rest_weight_pos[2] + 0.045,
        details=f"rest={rest_weight_pos}, raised={raised_weight_pos}",
    )

    with ctx.pose({pendulum_joint: 0.42, slider_joint: slider_upper}):
        ctx.expect_overlap(
            slider_weight,
            pendulum,
            axes="xy",
            elem_a="slider_weight",
            elem_b="rod_strip",
            min_overlap=0.002,
            name="slider weight stays registered over the rod",
        )

    with ctx.pose({pendulum_joint: -0.45}):
        left_swing_center = element_center("pendulum", "rod_strip")
    with ctx.pose({pendulum_joint: 0.45}):
        right_swing_center = element_center("pendulum", "rod_strip")

    ctx.check(
        "pendulum swings side to side through the front slot",
        left_swing_center is not None
        and right_swing_center is not None
        and left_swing_center[0] < -0.018
        and right_swing_center[0] > 0.018,
        details=f"left={left_swing_center}, right={right_swing_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
