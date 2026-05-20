from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK1_LEN = 0.46
LINK2_LEN = 0.28

PIVOT_GAP = 0.024
CHEEK_T = 0.012
JOINT_SETBACK = 0.015

BASE_TOP_Z = -0.14
BASE_PLATE_T = 0.012


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _make_base_bracket() -> cq.Workplane:
    cheek_t = 0.010
    cheek_y = PIVOT_GAP / 2.0 + cheek_t / 2.0
    pivot_x = -JOINT_SETBACK

    base_plate = (
        cq.Workplane("XY")
        .box(0.14, 0.10, BASE_PLATE_T)
        .translate((-0.035, 0.0, BASE_TOP_Z - BASE_PLATE_T / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    spine = _box((0.028, 0.054, 0.116), (-0.072, 0.000, -0.082))
    left_riser = _box((0.020, cheek_t, 0.072), (-0.042, cheek_y, -0.070))
    right_riser = _box((0.020, cheek_t, 0.072), (-0.042, -cheek_y, -0.070))
    left_cheek = _box((0.016, cheek_t, 0.050), (pivot_x, cheek_y, 0.000))
    right_cheek = _box((0.016, cheek_t, 0.050), (pivot_x, -cheek_y, 0.000))
    left_arm = _box((0.038, cheek_t, 0.012), (-0.034, cheek_y, -0.032))
    right_arm = _box((0.038, cheek_t, 0.012), (-0.034, -cheek_y, -0.032))

    bracket = base_plate.union(spine)
    bracket = bracket.union(left_riser).union(right_riser)
    bracket = bracket.union(left_cheek).union(right_cheek)
    bracket = bracket.union(left_arm).union(right_arm)
    return bracket


def _make_first_link() -> cq.Workplane:
    fork_cheek_t = 0.010
    fork_y = PIVOT_GAP / 2.0 + fork_cheek_t / 2.0

    proximal_boss = _cyl_y(0.012, PIVOT_GAP, (0.000, 0.000, 0.000))
    neck = _box((0.050, 0.016, 0.016), (0.034, 0.000, 0.000))
    beam = _box((0.300, 0.016, 0.016), (0.210, 0.000, 0.000))
    left_fork = _box((0.070, fork_cheek_t, 0.016), (LINK1_LEN - 0.035, fork_y, 0.000))
    right_fork = _box((0.070, fork_cheek_t, 0.016), (LINK1_LEN - 0.035, -fork_y, 0.000))

    return proximal_boss.union(neck).union(beam).union(left_fork).union(right_fork)


def _make_second_link() -> cq.Workplane:
    proximal_boss = _cyl_y(0.011, PIVOT_GAP, (0.000, 0.000, 0.000))
    neck = _box((0.045, 0.016, 0.014), (0.030, 0.000, 0.000))
    beam = _box((0.185, 0.016, 0.014), (0.148, 0.000, 0.000))
    tip_block = _box((0.048, 0.024, 0.024), (LINK2_LEN - 0.024, 0.000, 0.000))

    return proximal_boss.union(neck).union(beam).union(tip_block)


def _make_end_pad_mount() -> cq.Workplane:
    return _box((0.030, 0.032, 0.032), (0.015, 0.000, 0.000))


def _make_end_pad_face() -> cq.Workplane:
    backer = _cyl_x(0.023, 0.006, (0.033, 0.000, 0.000))
    rubber_pad = _cyl_x(0.029, 0.010, (0.041, 0.000, 0.000))
    return backer.union(rubber_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_service_arm")

    bracket_gray = model.material("bracket_gray", rgba=(0.33, 0.35, 0.39, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    pad_metal = model.material("pad_metal", rgba=(0.65, 0.67, 0.70, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(_make_base_bracket(), "base_bracket"),
        material=bracket_gray,
        name="bracket_shell",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.14, 0.10, 0.16)),
        mass=3.2,
        origin=Origin(xyz=(-0.02, 0.0, -0.07)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_make_first_link(), "first_link"),
        material=arm_gray,
        name="first_link_shell",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((LINK1_LEN, 0.05, 0.06)),
        mass=1.4,
        origin=Origin(xyz=(LINK1_LEN / 2.0, 0.0, 0.0)),
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_make_second_link(), "second_link"),
        material=arm_gray,
        name="second_link_shell",
    )
    second_link.inertial = Inertial.from_geometry(
        Box((LINK2_LEN, 0.042, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(LINK2_LEN / 2.0, 0.0, 0.0)),
    )

    end_pad = model.part("end_pad")
    end_pad.visual(
        mesh_from_cadquery(_make_end_pad_mount(), "end_pad_mount"),
        material=pad_metal,
        name="pad_mount",
    )
    end_pad.visual(
        mesh_from_cadquery(_make_end_pad_face(), "end_pad_face"),
        material=rubber_black,
        name="pad_face",
    )
    end_pad.inertial = Inertial.from_geometry(
        Box((0.052, 0.060, 0.060)),
        mass=0.20,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=first_link,
        origin=Origin(xyz=(-JOINT_SETBACK, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.8, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.2, lower=-2.15, upper=0.20),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=second_link,
        child=end_pad,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    end_pad = object_model.get_part("end_pad")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    pad_mount = object_model.get_articulation("pad_mount")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        base_bracket,
        first_link,
        reason=(
            "Shoulder hinge barrel is modeled as a simplified shared envelope; "
            "the omitted pin/bushing hardware is absorbed into the bracket and first-link meshes."
        ),
    )
    ctx.allow_overlap(
        first_link,
        second_link,
        reason=(
            "Elbow hinge barrel is modeled as a simplified shared envelope; "
            "the omitted pin/bushing hardware is absorbed into the two link meshes."
        ),
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in ("base_bracket", "first_link", "second_link", "end_pad"):
        ctx.check(f"has_{part_name}", object_model.get_part(part_name) is not None)

    for joint_name in ("shoulder", "elbow", "pad_mount"):
        ctx.check(f"has_{joint_name}", object_model.get_articulation(joint_name) is not None)

    ctx.check(
        "parallel_hinge_axes",
        shoulder.axis == (0.0, 1.0, 0.0) and elbow.axis == (0.0, 1.0, 0.0),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )
    ctx.check(
        "pad_mount_fixed",
        pad_mount.articulation_type == ArticulationType.FIXED,
        details=f"pad articulation type={pad_mount.articulation_type}",
    )

    ctx.expect_contact(first_link, base_bracket, name="shoulder_joint_has_physical_contact")
    ctx.expect_contact(second_link, first_link, name="elbow_joint_has_physical_contact")
    ctx.expect_contact(end_pad, second_link, name="pad_is_mounted_to_second_link")

    ctx.expect_origin_gap(
        second_link,
        base_bracket,
        axis="x",
        min_gap=0.44,
        max_gap=0.48,
        name="first_link_is_the_longer_reach_segment",
    )
    ctx.expect_origin_gap(
        end_pad,
        second_link,
        axis="x",
        min_gap=0.26,
        max_gap=0.30,
        name="second_link_is_shorter_than_first_link",
    )

    ctx.expect_origin_gap(
        end_pad,
        base_bracket,
        axis="x",
        min_gap=0.72,
        max_gap=0.78,
        name="rest_pose_reads_as_extended_service_arm",
    )

    folded_elbow = -1.60
    with ctx.pose({elbow: folded_elbow}):
        ctx.expect_origin_gap(
            end_pad,
            base_bracket,
            axis="x",
            min_gap=0.40,
            max_gap=0.50,
            name="folded_elbow_shortens_reach",
        )

    rest_pad = ctx.part_world_position(end_pad)
    with ctx.pose({shoulder: 0.80}):
        raised_pad = ctx.part_world_position(end_pad)
    ctx.check(
        "shoulder_joint_changes_pad_height",
        rest_pad is not None
        and raised_pad is not None
        and abs(raised_pad[2] - rest_pad[2]) > 0.20,
        details=f"rest={rest_pad}, raised={raised_pad}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
