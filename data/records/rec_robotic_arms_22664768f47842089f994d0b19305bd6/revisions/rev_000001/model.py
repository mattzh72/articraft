from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        min(radius, width * 0.25, height * 0.25),
        corner_segments=8,
    )
    return [(x, y_center + y, z_center + z) for y, z in profile]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robot_arm")

    base_gray = model.material("base_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.90, 0.42, 0.10, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.14, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_gray,
        name="base_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.08, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=base_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.18, 0.20, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=base_gray,
        name="shoulder_deck",
    )
    pedestal.visual(
        Box((0.12, 0.016, 0.18)),
        origin=Origin(xyz=(0.0, 0.065, 0.33)),
        material=base_gray,
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.12, 0.016, 0.18)),
        origin=Origin(xyz=(0.0, -0.065, 0.33)),
        material=base_gray,
        name="right_cheek",
    )
    pedestal.visual(
        Box((0.06, 0.12, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.28)),
        material=dark_gray,
        name="rear_motor_block",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.30, 0.24, 0.42)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=arm_orange,
        name="shoulder_turret",
    )
    shoulder_link.visual(
        Cylinder(radius=0.052, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=dark_gray,
        name="shoulder_cap",
    )
    shoulder_link.visual(
        Box((0.10, 0.11, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.10)),
        material=dark_gray,
        name="shoulder_housing",
    )
    upper_arm_geom = section_loft(
        [
            _yz_section(0.05, width=0.10, height=0.12, radius=0.022, z_center=0.10),
            _yz_section(0.18, width=0.088, height=0.10, radius=0.020, z_center=0.10),
            _yz_section(0.30, width=0.076, height=0.082, radius=0.016, z_center=0.10),
        ]
    )
    shoulder_link.visual(
        _mesh("upper_arm_shell", upper_arm_geom),
        material=arm_orange,
        name="upper_arm_shell",
    )
    shoulder_link.visual(
        Cylinder(radius=0.042, length=0.105),
        origin=Origin(xyz=(0.30, 0.0, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="elbow_hub",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((0.36, 0.14, 0.18)),
        mass=16.0,
        origin=Origin(xyz=(0.16, 0.0, 0.10)),
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        Box((0.03, 0.074, 0.068)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        material=dark_gray,
        name="elbow_knuckle",
    )
    forearm_link.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="elbow_collar",
    )
    forearm_geom = section_loft(
        [
            _yz_section(0.06, width=0.074, height=0.070, radius=0.016, z_center=0.0),
            _yz_section(0.15, width=0.064, height=0.060, radius=0.014, z_center=0.0),
            _yz_section(0.24, width=0.056, height=0.052, radius=0.012, z_center=0.0),
        ]
    )
    forearm_link.visual(
        _mesh("forearm_shell", forearm_geom),
        material=arm_orange,
        name="forearm_shell",
    )
    forearm_link.visual(
        Box((0.14, 0.03, 0.012)),
        origin=Origin(xyz=(0.14, 0.0, 0.026)),
        material=base_gray,
        name="forearm_cable_cover",
    )
    forearm_link.visual(
        Cylinder(radius=0.031, length=0.072),
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="wrist_sleeve",
    )
    forearm_link.inertial = Inertial.from_geometry(
        Box((0.28, 0.09, 0.09)),
        mass=10.5,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="wrist_flange",
    )
    wrist_head.visual(
        Box((0.054, 0.070, 0.062)),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=dark_gray,
        name="wrist_motor_housing",
    )
    wrist_head.visual(
        Box((0.018, 0.058, 0.052)),
        origin=Origin(xyz=(0.107, 0.0, 0.0)),
        material=steel,
        name="tool_plate",
    )
    wrist_head.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.107, 0.0, -0.020)),
        material=black,
        name="tool_stem",
    )
    wrist_head.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.107, 0.0, -0.034)),
        material=black,
        name="tool_pad",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.11, 0.08, 0.09)),
        mass=3.8,
        origin=Origin(xyz=(0.078, 0.0, -0.008)),
    )

    model.articulation(
        "pedestal_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=-radians(170.0),
            upper=radians(170.0),
        ),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm_link,
        origin=Origin(xyz=(0.30, 0.0, 0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=2.2,
            lower=-radians(100.0),
            upper=radians(110.0),
        ),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=wrist_head,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=36.0,
            velocity=3.2,
            lower=-radians(180.0),
            upper=radians(180.0),
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
    pedestal = object_model.get_part("pedestal")
    shoulder_link = object_model.get_part("shoulder_link")
    forearm_link = object_model.get_part("forearm_link")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_joint = object_model.get_articulation("pedestal_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        shoulder_link,
        pedestal,
        axis="z",
        positive_elem="shoulder_turret",
        negative_elem="shoulder_deck",
        min_gap=0.0,
        max_gap=0.002,
        name="shoulder turret seats on the pedestal deck",
    )

    rest_pos = aabb_center(ctx.part_element_world_aabb(shoulder_link, elem="upper_arm_shell"))
    with ctx.pose({shoulder_joint: radians(55.0)}):
        turned_pos = aabb_center(ctx.part_element_world_aabb(shoulder_link, elem="upper_arm_shell"))
    ctx.check(
        "positive shoulder yaw swings the arm toward positive y",
        rest_pos is not None
        and turned_pos is not None
        and turned_pos[1] > rest_pos[1] + 0.05,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    forearm_rest = aabb_center(ctx.part_element_world_aabb(forearm_link, elem="forearm_shell"))
    with ctx.pose({elbow_joint: radians(70.0)}):
        forearm_raised = aabb_center(ctx.part_element_world_aabb(forearm_link, elem="forearm_shell"))
    ctx.check(
        "positive elbow motion lifts the forearm",
        forearm_rest is not None
        and forearm_raised is not None
        and forearm_raised[2] > forearm_rest[2] + 0.08,
        details=f"rest={forearm_rest}, raised={forearm_raised}",
    )

    ctx.expect_gap(
        wrist_head,
        forearm_link,
        axis="x",
        positive_elem="wrist_flange",
        negative_elem="wrist_sleeve",
        min_gap=0.0,
        max_gap=0.002,
        name="wrist flange seats on the forearm sleeve",
    )

    wrist_rest = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_pad"))
    with ctx.pose({wrist_joint: radians(80.0)}):
        wrist_rolled = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_pad"))
    ctx.check(
        "positive wrist roll swings the tool toward positive y",
        wrist_rest is not None
        and wrist_rolled is not None
        and wrist_rolled[1] > wrist_rest[1] + 0.02,
        details=f"rest={wrist_rest}, rolled={wrist_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
