from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

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

POLE_TOP_Z = 12.2
BRANCH_START = (0.04, 0.0, 11.9)
LEFT_BRANCH_END = (0.92, -2.25, 13.55)
RIGHT_BRANCH_END = (0.92, 2.25, 13.55)
MOUNT_FRACTIONS = (0.24, 0.54, 0.84)
HEAD_REST_TILT = 0.42


def _lerp_point(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.hypot(dx, dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    center = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _tube_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    length, origin = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _tilted_origin(point: tuple[float, float, float], tilt: float) -> Origin:
    return Origin(xyz=_rotate_y(point, tilt), rpy=(0.0, tilt, 0.0))


def _add_mount(mast, joint_center: tuple[float, float, float], branch_point: tuple[float, float, float], metal, index: int) -> None:
    crossbeam_center = (joint_center[0] - 0.05, joint_center[1], joint_center[2] - 0.01)
    post_top = (joint_center[0] - 0.06, joint_center[1], joint_center[2] - 0.065)
    _tube_between(mast, branch_point, post_top, radius=0.028, material=metal, name=f"mount_{index}_post")
    mast.visual(
        Box((0.052, 0.22, 0.045)),
        origin=Origin(xyz=crossbeam_center),
        material=metal,
        name=f"mount_{index}_crossbeam",
    )
    mast.visual(
        Box((0.04, 0.08, 0.07)),
        origin=Origin(xyz=(joint_center[0] - 0.052, joint_center[1], joint_center[2] - 0.055)),
        material=metal,
        name=f"mount_{index}_web",
    )
    tab_thickness = 0.02
    tab_center_y = 0.10 + tab_thickness * 0.5
    for sign, suffix in ((-1.0, "neg"), (1.0, "pos")):
        mast.visual(
            Box((0.028, tab_thickness, 0.10)),
            origin=Origin(xyz=(joint_center[0] - 0.012, joint_center[1] + sign * tab_center_y, joint_center[2])),
            material=metal,
            name=f"mount_{index}_tab_{suffix}",
        )


def _add_floodlight_head(model: ArticulatedObject, index: int, housing_mat, bracket_mat, lens_mat):
    head = model.part(f"head_{index}")
    _tube_between(head, (0.0, -0.10, 0.0), (0.0, 0.10, 0.0), radius=0.022, material=bracket_mat, name="pivot_shaft")

    for side, side_name in ((-1.0, "left"), (1.0, "right")):
        _tube_between(
            head,
            _rotate_y((0.012, side * 0.08, 0.008), HEAD_REST_TILT),
            _rotate_y((0.255, side * 0.08, 0.078), HEAD_REST_TILT),
            radius=0.010,
            material=bracket_mat,
            name=f"yoke_{side_name}",
        )

    head.visual(
        Box((0.07, 0.172, 0.035)),
        origin=_tilted_origin((0.255, 0.0, 0.078), HEAD_REST_TILT),
        material=bracket_mat,
        name="yoke_bridge",
    )
    head.visual(
        Box((0.34, 0.18, 0.22)),
        origin=_tilted_origin((0.45, 0.0, -0.015), HEAD_REST_TILT),
        material=housing_mat,
        name="housing_shell",
    )
    head.visual(
        Box((0.03, 0.19, 0.23)),
        origin=_tilted_origin((0.63, 0.0, -0.015), HEAD_REST_TILT),
        material=bracket_mat,
        name="front_frame",
    )
    head.visual(
        Box((0.012, 0.174, 0.216)),
        origin=_tilted_origin((0.612, 0.0, -0.015), HEAD_REST_TILT),
        material=lens_mat,
        name="lens_glass",
    )
    head.visual(
        Box((0.15, 0.19, 0.018)),
        origin=_tilted_origin((0.54, 0.0, 0.082), HEAD_REST_TILT),
        material=bracket_mat,
        name="visor",
    )
    head.visual(
        Box((0.12, 0.16, 0.14)),
        origin=_tilted_origin((0.29, 0.0, 0.005), HEAD_REST_TILT),
        material=housing_mat,
        name="rear_driver",
    )
    for fin_index, x_pos in enumerate((0.235, 0.205, 0.175), start=1):
        head.visual(
            Box((0.012, 0.15, 0.17)),
            origin=_tilted_origin((x_pos, 0.0, 0.03), HEAD_REST_TILT),
            material=bracket_mat,
            name=f"heat_fin_{fin_index}",
        )

    return head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_field_floodlight_mast")

    galvanized = model.material("galvanized_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    painted_head = model.material("painted_head", rgba=(0.22, 0.24, 0.26, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.80, 0.88, 0.42))

    mast = model.part("mast")
    mast.visual(Cylinder(radius=0.26, length=0.14), origin=Origin(xyz=(0.0, 0.0, 0.07)), material=galvanized, name="base_flange")
    mast.visual(Cylinder(radius=0.18, length=7.0), origin=Origin(xyz=(0.0, 0.0, 3.5)), material=galvanized, name="pole_lower")
    mast.visual(Cylinder(radius=0.13, length=5.2), origin=Origin(xyz=(0.0, 0.0, 9.6)), material=galvanized, name="pole_upper")
    mast.visual(Cylinder(radius=0.21, length=0.35), origin=Origin(xyz=(0.04, 0.0, 12.175)), material=galvanized, name="crown_hub")
    mast.visual(
        Cylinder(radius=0.018, length=0.65),
        origin=Origin(xyz=(0.08, 0.0, POLE_TOP_Z + 0.325)),
        material=galvanized,
        name="lightning_spike",
    )
    _tube_between(mast, BRANCH_START, LEFT_BRANCH_END, 0.072, galvanized, "branch_left")
    _tube_between(mast, BRANCH_START, RIGHT_BRANCH_END, 0.072, galvanized, "branch_right")
    _tube_between(mast, (0.0, 0.0, 11.05), (0.52, -1.16, 12.55), 0.048, galvanized, "brace_left")
    _tube_between(mast, (0.0, 0.0, 11.05), (0.52, 1.16, 12.55), 0.048, galvanized, "brace_right")

    head_specs: list[tuple[int, tuple[float, float, float]]] = []
    head_index = 1
    for branch_end in (LEFT_BRANCH_END, RIGHT_BRANCH_END):
        for fraction in MOUNT_FRACTIONS:
            branch_point = _lerp_point(BRANCH_START, branch_end, fraction)
            joint_center = (branch_point[0] + 0.20, branch_point[1], branch_point[2] - 0.03)
            _add_mount(mast, joint_center, branch_point, galvanized, head_index)
            head_specs.append((head_index, joint_center))
            head_index += 1

    for head_index, joint_center in head_specs:
        head = _add_floodlight_head(model, head_index, painted_head, galvanized, lens_glass)
        model.articulation(
            f"head_{head_index}_tilt",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=head,
            origin=Origin(xyz=joint_center),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.30, upper=0.70),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pole_upper = mast.get_visual("pole_upper")
    heads = [object_model.get_part(f"head_{index}") for index in range(1, 7)]
    joints = [object_model.get_articulation(f"head_{index}_tilt") for index in range(1, 7)]
    head_specs = []
    for index, (head, joint) in enumerate(zip(heads, joints), start=1):
        head_specs.append(
            {
                "index": index,
                "head": head,
                "joint": joint,
                "shaft": head.get_visual("pivot_shaft"),
                "housing": head.get_visual("housing_shell"),
                "lens": head.get_visual("lens_glass"),
                "left_tab": mast.get_visual(f"mount_{index}_tab_neg"),
                "right_tab": mast.get_visual(f"mount_{index}_tab_pos"),
            }
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("six_floodlight_heads_present", len(heads) == 6, f"Expected 6 heads, found {len(heads)}.")
    ctx.check("six_tilt_joints_present", len(joints) == 6, f"Expected 6 tilt joints, found {len(joints)}.")

    mast_aabb = ctx.part_world_aabb(mast)
    assert mast_aabb is not None
    mast_height = mast_aabb[1][2] - mast_aabb[0][2]
    mast_span_y = mast_aabb[1][1] - mast_aabb[0][1]
    mast_span_x = mast_aabb[1][0] - mast_aabb[0][0]
    ctx.check(
        "mast_has_realistic_field_light_height",
        13.2 <= mast_height <= 13.8,
        f"Mast height should read like a stadium light, got {mast_height:.3f} m.",
    )
    ctx.check(
        "crossarm_has_broad_branching_span",
        4.3 <= mast_span_y <= 4.8,
        f"Branching crossarm should span about 4.5 m across, got {mast_span_y:.3f} m.",
    )
    ctx.check(
        "crossarm_projects_forward_from_pole",
        1.20 <= mast_span_x <= 1.35,
        f"Crossarm should project forward roughly 1.3 m, got {mast_span_x:.3f} m.",
    )

    for spec in head_specs:
        index = spec["index"]
        head = spec["head"]
        joint = spec["joint"]
        shaft = spec["shaft"]
        housing = spec["housing"]
        lens = spec["lens"]
        left_tab = spec["left_tab"]
        right_tab = spec["right_tab"]
        limits = joint.motion_limits

        ctx.check(
            f"head_{index}_tilt_axis_is_lateral",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"Head {index} should tilt about local/world +Y, got axis {joint.axis}.",
        )
        ctx.check(
            f"head_{index}_tilt_limits_match_floodlight_bracket",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and math.isclose(limits.lower, -0.30, abs_tol=1e-6)
            and math.isclose(limits.upper, 0.70, abs_tol=1e-6),
            f"Head {index} should have bounded bracket tilt from -0.30 to 0.70 rad; got {limits}.",
        )

        ctx.expect_contact(head, mast, elem_a=shaft, elem_b=left_tab, name=f"head_{index}_shaft_contacts_left_tab")
        ctx.expect_contact(head, mast, elem_a=shaft, elem_b=right_tab, name=f"head_{index}_shaft_contacts_right_tab")
        ctx.expect_gap(
            head,
            mast,
            axis="x",
            min_gap=0.35,
            positive_elem=housing,
            negative_elem=pole_upper,
            name=f"head_{index}_housing_projects_forward_of_pole",
        )
        ctx.expect_gap(
            head,
            mast,
            axis="x",
            min_gap=0.75,
            positive_elem=lens,
            negative_elem=pole_upper,
            name=f"head_{index}_lens_projects_forward_of_pole",
        )

        rest_lens_aabb = ctx.part_element_world_aabb(head, elem="lens_glass")
        assert rest_lens_aabb is not None
        rest_lens_center_x = 0.5 * (rest_lens_aabb[0][0] + rest_lens_aabb[1][0])

        assert limits is not None and limits.lower is not None and limits.upper is not None
        with ctx.pose({joint: limits.lower}):
            lower_lens_aabb = ctx.part_element_world_aabb(head, elem="lens_glass")
            assert lower_lens_aabb is not None
            lower_lens_center_x = 0.5 * (lower_lens_aabb[0][0] + lower_lens_aabb[1][0])
            ctx.fail_if_parts_overlap_in_current_pose(name=f"head_{index}_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"head_{index}_lower_pose_no_floating")
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=left_tab,
                name=f"head_{index}_lower_pose_left_tab_contact",
            )
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=right_tab,
                name=f"head_{index}_lower_pose_right_tab_contact",
            )
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.90,
                positive_elem=lens,
                negative_elem=pole_upper,
                name=f"head_{index}_lower_pose_lens_stays_forward_of_pole",
            )
        ctx.check(
            f"head_{index}_lower_pose_moves_lens_forward",
            lower_lens_center_x > rest_lens_center_x + 0.045,
            (
                f"Head {index} lower tilt should swing the lens forward: "
                f"rest center x {rest_lens_center_x:.3f}, lower center x {lower_lens_center_x:.3f}."
            ),
        )
        ctx.check(
            f"head_{index}_lower_pose_moves_lens_up",
            lower_lens_aabb[1][2] > rest_lens_aabb[1][2] + 0.15,
            (
                f"Head {index} lower tilt should raise the lens: "
                f"rest max z {rest_lens_aabb[1][2]:.3f}, lower max z {lower_lens_aabb[1][2]:.3f}."
            ),
        )

        with ctx.pose({joint: limits.upper}):
            upper_lens_aabb = ctx.part_element_world_aabb(head, elem="lens_glass")
            assert upper_lens_aabb is not None
            ctx.fail_if_parts_overlap_in_current_pose(name=f"head_{index}_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"head_{index}_upper_pose_no_floating")
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=left_tab,
                name=f"head_{index}_upper_pose_left_tab_contact",
            )
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=right_tab,
                name=f"head_{index}_upper_pose_right_tab_contact",
            )
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.40,
                positive_elem=lens,
                negative_elem=pole_upper,
                name=f"head_{index}_upper_pose_lens_stays_forward_of_pole",
            )
        ctx.check(
            f"head_{index}_upper_pose_moves_lens_back",
            upper_lens_aabb[0][0] < rest_lens_aabb[0][0] - 0.30,
            (
                f"Head {index} upper tilt should swing the lens back: "
                f"rest min x {rest_lens_aabb[0][0]:.3f}, upper min x {upper_lens_aabb[0][0]:.3f}."
            ),
        )
        ctx.check(
            f"head_{index}_upper_pose_moves_lens_down",
            upper_lens_aabb[1][2] < rest_lens_aabb[1][2] - 0.20,
            (
                f"Head {index} upper tilt should drop the lens: "
                f"rest max z {rest_lens_aabb[1][2]:.3f}, upper max z {upper_lens_aabb[1][2]:.3f}."
            ),
        )

    ctx.expect_gap(heads[3], heads[0], axis="y", min_gap=0.70, name="inner_heads_split_across_pole")
    ctx.expect_gap(heads[5], heads[2], axis="y", min_gap=3.20, name="outer_heads_span_broad_crossarm")
    ctx.expect_gap(heads[0], heads[1], axis="y", min_gap=0.42, name="left_inner_and_mid_heads_are_spaced")
    ctx.expect_gap(heads[1], heads[2], axis="y", min_gap=0.42, name="left_mid_and_outer_heads_are_spaced")
    ctx.expect_gap(heads[4], heads[3], axis="y", min_gap=0.42, name="right_inner_and_mid_heads_are_spaced")
    ctx.expect_gap(heads[5], heads[4], axis="y", min_gap=0.42, name="right_mid_and_outer_heads_are_spaced")

    aiming_pose = {joint: angle for joint, angle in zip(joints, (0.18, 0.28, 0.38, 0.18, 0.28, 0.38))}
    with ctx.pose(aiming_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="aiming_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="aiming_pose_no_floating")
        for spec in head_specs:
            index = spec["index"]
            head = spec["head"]
            shaft = spec["shaft"]
            housing = spec["housing"]
            left_tab = spec["left_tab"]
            right_tab = spec["right_tab"]
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=left_tab,
                name=f"head_{index}_left_tab_contact_in_aiming_pose",
            )
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=right_tab,
                name=f"head_{index}_right_tab_contact_in_aiming_pose",
            )
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.20,
                positive_elem=housing,
                negative_elem=pole_upper,
                name=f"head_{index}_clears_pole_in_aiming_pose",
            )

    with ctx.pose({joint: -0.18 for joint in joints}):
        ctx.fail_if_parts_overlap_in_current_pose(name="flattened_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="flattened_pose_no_floating")
        for spec in head_specs:
            index = spec["index"]
            head = spec["head"]
            shaft = spec["shaft"]
            housing = spec["housing"]
            left_tab = spec["left_tab"]
            right_tab = spec["right_tab"]
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=left_tab,
                name=f"head_{index}_retains_left_mount_contact_when_flattened",
            )
            ctx.expect_contact(
                head,
                mast,
                elem_a=shaft,
                elem_b=right_tab,
                name=f"head_{index}_retains_right_mount_contact_when_flattened",
            )
            ctx.expect_gap(
                head,
                mast,
                axis="x",
                min_gap=0.30,
                positive_elem=housing,
                negative_elem=pole_upper,
                name=f"head_{index}_stays_forward_of_pole_when_flattened",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
