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


PLATE_T = 0.008
EAR_T = 0.003
BAR_W = 0.015
JOINT_R = 0.012
FORK_SPAN = PLATE_T + 2.0 * EAR_T
Z_OFFSET = PLATE_T / 2.0 + EAR_T / 2.0

LINK_1_LEN = 0.074
LINK_2_LEN = 0.064
END_LINK_LEN = 0.052


def _add_box(part, size, xyz, material, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_disk(part, radius: float, thickness: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=thickness),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_mid_link_visuals(part, material, length: float, name_seed: str) -> None:
    bar_start = -0.002
    bar_end = length - 0.028
    bar_length = bar_end - bar_start

    _add_disk(part, JOINT_R, PLATE_T, (0.0, 0.0, 0.0), material, f"{name_seed}_prox_eye")
    _add_box(
        part,
        (bar_length, BAR_W, PLATE_T),
        ((bar_start + bar_end) / 2.0, 0.0, 0.0),
        material,
        f"{name_seed}_spine",
    )
    _add_box(
        part,
        (0.008, BAR_W * 0.9, FORK_SPAN),
        (length - 0.024, 0.0, 0.0),
        material,
        f"{name_seed}_fork_web",
    )

    for side_name, sign in (("upper", 1.0), ("lower", -1.0)):
        _add_box(
            part,
            (0.020, BAR_W * 0.92, EAR_T),
            (length - 0.010, 0.0, sign * Z_OFFSET),
            material,
            f"{name_seed}_{side_name}_strap",
        )
        _add_disk(
            part,
            JOINT_R,
            EAR_T,
            (length, 0.0, sign * Z_OFFSET),
            material,
            f"{name_seed}_{side_name}_pad",
        )


def _add_end_link_visuals(part, material) -> None:
    bar_start = -0.002
    bar_end = END_LINK_LEN - 0.006
    bar_length = bar_end - bar_start
    tip_x = END_LINK_LEN + 0.016

    _add_disk(part, JOINT_R, PLATE_T, (0.0, 0.0, 0.0), material, "end_eye")
    _add_box(
        part,
        (bar_length, BAR_W, PLATE_T),
        ((bar_start + bar_end) / 2.0, 0.0, 0.0),
        material,
        "end_spine",
    )
    _add_box(
        part,
        (0.022, 0.012, PLATE_T),
        (END_LINK_LEN + 0.004, 0.0, 0.0),
        material,
        "end_tab",
    )
    _add_disk(part, 0.0065, PLATE_T, (tip_x, 0.0, 0.0), material, "end_tip")


def _add_root_bracket_visuals(part, material) -> None:
    _add_box(
        part,
        (0.046, 0.058, PLATE_T),
        (-0.040, 0.0, 0.0),
        material,
        "bracket_plate",
    )
    _add_box(
        part,
        (0.012, 0.022, FORK_SPAN),
        (-0.021, 0.0, 0.0),
        material,
        "bracket_web",
    )

    for side_name, sign in (("upper", 1.0), ("lower", -1.0)):
        _add_box(
            part,
            (0.020, BAR_W * 0.98, EAR_T),
            (-0.010, 0.0, sign * Z_OFFSET),
            material,
            f"bracket_{side_name}_strap",
        )
        _add_disk(
            part,
            JOINT_R,
            EAR_T,
            (0.0, 0.0, sign * Z_OFFSET),
            material,
            f"bracket_{side_name}_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lever_chain")

    bracket_finish = model.material("bracket_finish", rgba=(0.18, 0.20, 0.23, 1.0))
    link_finish = model.material("link_finish", rgba=(0.66, 0.69, 0.73, 1.0))

    root_bracket = model.part("root_bracket")
    _add_root_bracket_visuals(root_bracket, bracket_finish)

    link_1 = model.part("link_1")
    _add_mid_link_visuals(link_1, link_finish, LINK_1_LEN, "link_1")

    link_2 = model.part("link_2")
    _add_mid_link_visuals(link_2, link_finish, LINK_2_LEN, "link_2")

    end_link = model.part("end_link")
    _add_end_link_visuals(end_link, link_finish)

    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=-1.1,
            upper=1.1,
        ),
    )
    model.articulation(
        "link_2_to_end_link",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_link,
        origin=Origin(xyz=(LINK_2_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.4,
            lower=-1.1,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_link = object_model.get_part("end_link")

    root_to_link_1 = object_model.get_articulation("root_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_end_link = object_model.get_articulation("link_2_to_end_link")

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

    for joint in (root_to_link_1, link_1_to_link_2, link_2_to_end_link):
        ctx.check(
            f"{joint.name} axis is planar and parallel",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis was {joint.axis}",
        )

    with ctx.pose(
        {
            root_to_link_1: 0.0,
            link_1_to_link_2: 0.0,
            link_2_to_end_link: 0.0,
        }
    ):
        ctx.expect_contact(root_bracket, link_1, name="root bracket touches link 1")
        ctx.expect_contact(link_1, link_2, name="link 1 touches link 2")
        ctx.expect_contact(link_2, end_link, name="link 2 touches end link")
        ctx.expect_origin_gap(
            link_2,
            link_1,
            axis="x",
            min_gap=LINK_1_LEN - 1e-4,
            max_gap=LINK_1_LEN + 1e-4,
            name="joint spacing from link 1 to link 2",
        )
        ctx.expect_origin_gap(
            end_link,
            link_2,
            axis="x",
            min_gap=LINK_2_LEN - 1e-4,
            max_gap=LINK_2_LEN + 1e-4,
            name="joint spacing from link 2 to end link",
        )

    with ctx.pose(
        {
            root_to_link_1: 0.65,
            link_1_to_link_2: -0.55,
            link_2_to_end_link: 0.45,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in representative bent pose")
        bent_end_origin = ctx.part_world_position(end_link)
        ctx.check(
            "serial lever chain bends in plane",
            bent_end_origin is not None
            and bent_end_origin[1] > 0.015
            and abs(bent_end_origin[2]) < 1e-6,
            details=f"end link origin was {bent_end_origin}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
