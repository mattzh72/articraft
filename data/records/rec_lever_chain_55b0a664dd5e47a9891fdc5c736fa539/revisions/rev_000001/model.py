from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
from math import pi


def _add_box(part, size: tuple[float, float, float], center: tuple[float, float, float], material, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_cylinder_y(part, radius: float, length_y: float, center: tuple[float, float, float], material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length_y),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_lever_chain")

    pedestal_mat = model.material("pedestal_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    link1_mat = model.material("link_steel_dark", rgba=(0.45, 0.47, 0.50, 1.0))
    link2_mat = model.material("link_steel_mid", rgba=(0.57, 0.59, 0.62, 1.0))
    link3_mat = model.material("link_steel_light", rgba=(0.66, 0.68, 0.71, 1.0))

    link1_width = 0.024
    link2_width = 0.021
    link3_width = 0.018
    pedestal_cheek_y = 0.0085
    link1_tip_cheek_y = 0.0075
    link2_tip_cheek_y = 0.00675

    pedestal = model.part("pedestal")
    _add_box(pedestal, (0.160, 0.080, 0.018), (0.0, 0.0, 0.009), pedestal_mat, "base")
    _add_box(pedestal, (0.044, 0.026, 0.042), (0.0, 0.0, 0.039), pedestal_mat, "column")
    _add_box(pedestal, (0.026, 0.008, 0.022), (-0.006, -0.0085, 0.067), pedestal_mat, "left_post")
    _add_box(pedestal, (0.026, 0.008, 0.022), (-0.006, 0.0085, 0.067), pedestal_mat, "right_post")
    _add_box(pedestal, (0.024, 0.007, 0.026), (-0.004, -pedestal_cheek_y, 0.084), pedestal_mat, "left_cheek")
    _add_box(pedestal, (0.024, 0.007, 0.026), (-0.004, pedestal_cheek_y, 0.084), pedestal_mat, "right_cheek")
    _add_cylinder_y(pedestal, 0.018, 0.007, (0.0, -pedestal_cheek_y, 0.085), pedestal_mat, "left_bearing")
    _add_cylinder_y(pedestal, 0.018, 0.007, (0.0, pedestal_cheek_y, 0.085), pedestal_mat, "right_bearing")

    link1_len = 0.200
    link2_len = 0.170
    link3_len = 0.138

    link1 = model.part("link1")
    _add_cylinder_y(link1, 0.018, 0.010, (0.0, 0.0, 0.0), link1_mat, "root_barrel")
    _add_box(link1, (0.050, 0.014, 0.034), (0.025, 0.0, 0.0), link1_mat, "root_bridge")
    _add_box(link1, (0.110, 0.0055, 0.022), (0.105, -0.00925, 0.0), link1_mat, "left_rail")
    _add_box(link1, (0.110, 0.0055, 0.022), (0.105, 0.00925, 0.0), link1_mat, "right_rail")
    _add_box(link1, (0.022, 0.024, 0.030), (0.171, 0.0, 0.0), link1_mat, "tip_yoke")
    _add_box(link1, (0.018, 0.006, 0.030), (0.191, -link1_tip_cheek_y, 0.0), link1_mat, "left_tip_cheek")
    _add_box(link1, (0.018, 0.006, 0.030), (0.191, link1_tip_cheek_y, 0.0), link1_mat, "right_tip_cheek")
    _add_cylinder_y(link1, 0.016, 0.006, (link1_len, -link1_tip_cheek_y, 0.0), link1_mat, "left_tip_bearing")
    _add_cylinder_y(link1, 0.016, 0.006, (link1_len, link1_tip_cheek_y, 0.0), link1_mat, "right_tip_bearing")

    link2 = model.part("link2")
    _add_cylinder_y(link2, 0.016, 0.009, (0.0, 0.0, 0.0), link2_mat, "root_barrel")
    _add_box(link2, (0.044, 0.012, 0.030), (0.022, 0.0, 0.0), link2_mat, "root_bridge")
    _add_box(link2, (0.102, 0.0050, 0.020), (0.091, -0.0080, 0.0), link2_mat, "left_rail")
    _add_box(link2, (0.102, 0.0050, 0.020), (0.091, 0.0080, 0.0), link2_mat, "right_rail")
    _add_box(link2, (0.018, 0.021, 0.026), (0.145, 0.0, 0.0), link2_mat, "tip_yoke")
    _add_box(link2, (0.016, 0.0055, 0.026), (0.162, -link2_tip_cheek_y, 0.0), link2_mat, "left_tip_cheek")
    _add_box(link2, (0.016, 0.0055, 0.026), (0.162, link2_tip_cheek_y, 0.0), link2_mat, "right_tip_cheek")
    _add_cylinder_y(link2, 0.014, 0.0055, (link2_len, -link2_tip_cheek_y, 0.0), link2_mat, "left_tip_bearing")
    _add_cylinder_y(link2, 0.014, 0.0055, (link2_len, link2_tip_cheek_y, 0.0), link2_mat, "right_tip_bearing")

    link3 = model.part("link3")
    _add_cylinder_y(link3, 0.014, 0.008, (0.0, 0.0, 0.0), link3_mat, "root_barrel")
    _add_box(link3, (0.040, 0.011, 0.026), (0.020, 0.0, 0.0), link3_mat, "root_bridge")
    _add_box(link3, (0.082, 0.0045, 0.018), (0.079, -0.00675, 0.0), link3_mat, "left_rail")
    _add_box(link3, (0.082, 0.0045, 0.018), (0.079, 0.00675, 0.0), link3_mat, "right_rail")
    _add_box(link3, (0.018, 0.018, 0.022), (0.126, 0.0, 0.0), link3_mat, "tab_connector")
    _add_box(link3, (0.032, 0.011, 0.018), (0.151, 0.0, 0.0), link3_mat, "end_tab")

    common_limits = MotionLimits(effort=8.0, velocity=2.0, lower=-0.95, upper=1.10)

    model.articulation(
        "pedestal_to_link1",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(link1_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(link2_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")

    j1 = object_model.get_articulation("pedestal_to_link1")
    j2 = object_model.get_articulation("link1_to_link2")
    j3 = object_model.get_articulation("link2_to_link3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(pedestal, link1, name="pedestal_supports_link1")
    ctx.expect_contact(link1, link2, name="link1_supports_link2")
    ctx.expect_contact(link2, link3, name="link2_supports_link3")

    expected_axis = (0.0, -1.0, 0.0)
    joints_ok = all(tuple(j.axis) == expected_axis for j in (j1, j2, j3))
    limits_ok = all(
        j.motion_limits is not None
        and j.motion_limits.lower == -0.95
        and j.motion_limits.upper == 1.10
        for j in (j1, j2, j3)
    )
    ctx.check(
        "serial_revolute_axes_parallel",
        joints_ok and limits_ok,
        details="All three serial joints should share the same supported -Y hinge axis and motion range.",
    )

    def center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float:
        if aabb is None:
            return float("-inf")
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({j1: 0.0, j2: 0.0, j3: 0.0}):
        link1_rest = ctx.part_world_aabb(link1)
        link2_rest = ctx.part_world_aabb(link2)
        link3_rest = ctx.part_world_aabb(link3)

    with ctx.pose({j1: 0.70, j2: 0.0, j3: 0.0}):
        link2_from_joint1 = ctx.part_world_aabb(link2)

    with ctx.pose({j1: 0.0, j2: 0.70, j3: 0.0}):
        link3_from_joint2 = ctx.part_world_aabb(link3)

    with ctx.pose({j1: 0.0, j2: 0.0, j3: 0.70}):
        link3_from_joint3 = ctx.part_world_aabb(link3)

    ctx.check(
        "joint1_positive_motion_lifts_chain",
        center_z(link2_from_joint1) > center_z(link2_rest) + 0.010,
        details="Positive motion on the first hinge should raise the downstream chain.",
    )
    ctx.check(
        "joint2_positive_motion_lifts_child",
        center_z(link3_from_joint2) > center_z(link3_rest) + 0.008,
        details="Positive motion on the second hinge should raise the smaller child link.",
    )
    ctx.check(
        "joint3_positive_motion_lifts_terminal_link",
        center_z(link3_from_joint3) > center_z(link3_rest) + 0.006,
        details="Positive motion on the third hinge should raise the terminal link and end tab.",
    )

    if link1_rest is not None and link2_rest is not None and link3_rest is not None:
        link1_len = link1_rest[1][0] - link1_rest[0][0]
        link2_len = link2_rest[1][0] - link2_rest[0][0]
        link3_len = link3_rest[1][0] - link3_rest[0][0]
        ctx.check(
            "links_step_down_toward_tip",
            link1_len > link2_len > link3_len,
            details="The lever links should taper down in overall length toward the tip.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
