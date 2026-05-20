from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_THICKNESS = 0.009
CENTER_BOSS_LENGTH = 0.012
FORK_GAP = 0.012
OUTER_BOSS_LENGTH = 0.005
OUTER_SPAN = FORK_GAP + 2.0 * OUTER_BOSS_LENGTH
BOSS_RADIUS = 0.0085
FORK_REACH = 0.016
FORK_HUB_LENGTH = 0.0045

PROXIMAL_LENGTH = 0.070
MIDDLE_LENGTH = 0.055
DISTAL_LENGTH = 0.045


def _rect_xz(x0: float, x1: float, z_size: float, y_size: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center((x0 + x1) * 0.5, 0.0)
        .rect(x1 - x0, z_size)
        .extrude(y_size * 0.5, both=True)
    )


def _circle_xz(x: float, radius: float, y_size: float) -> cq.Workplane:
    return cq.Workplane("XZ").center(x, 0.0).circle(radius).extrude(y_size * 0.5, both=True)


def _tapered_prism_xz(
    x0: float,
    x1: float,
    z0: float,
    z1: float,
    y_size: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (x0, z0 * 0.5),
                (x1, z1 * 0.5),
                (x1, -z1 * 0.5),
                (x0, -z0 * 0.5),
            ]
        )
        .close()
        .extrude(y_size * 0.5, both=True)
    )


def _half_circle_xz(
    joint_x: float,
    radius: float,
    y_size: float,
    *,
    side: str,
) -> cq.Workplane:
    disc = _circle_xz(joint_x, radius, y_size)
    if side == "rear":
        mask = _rect_xz(joint_x - radius, joint_x, radius * 2.0, y_size)
    elif side == "front":
        mask = _rect_xz(joint_x, joint_x + radius, radius * 2.0, y_size)
    else:
        raise ValueError(f"unsupported half-circle side: {side}")
    return disc.intersect(mask)


def _add_outer_joint_fork(
    shape: cq.Workplane,
    *,
    joint_x: float,
    lug_x0: float,
    lug_height: float,
) -> cq.Workplane:
    y_offset = FORK_GAP * 0.5 + OUTER_BOSS_LENGTH * 0.5
    for sign in (-1.0, 1.0):
        lug = _rect_xz(lug_x0, joint_x, lug_height, OUTER_BOSS_LENGTH).translate(
            (0.0, sign * y_offset, 0.0)
        )
        boss = _half_circle_xz(
            joint_x,
            BOSS_RADIUS,
            OUTER_BOSS_LENGTH,
            side="rear",
        ).translate(
            (0.0, sign * y_offset, 0.0)
        )
        shape = shape.union(lug).union(boss)
    return shape


def _add_outer_fork_rails(
    shape: cq.Workplane,
    *,
    x0: float,
    x1: float,
    z_size: float,
) -> cq.Workplane:
    y_offset = FORK_GAP * 0.5 + OUTER_BOSS_LENGTH * 0.5
    for sign in (-1.0, 1.0):
        rail = _rect_xz(x0, x1, z_size, OUTER_BOSS_LENGTH).translate((0.0, sign * y_offset, 0.0))
        shape = shape.union(rail)
    return shape


def make_root_mount() -> cq.Workplane:
    rear_flange = _rect_xz(-0.046, -0.031, 0.038, OUTER_SPAN + 0.010)
    body_block = _tapered_prism_xz(-0.036, -0.020, 0.030, 0.024, OUTER_SPAN)
    center_rib = _tapered_prism_xz(-0.028, -FORK_REACH, 0.018, 0.016, BODY_THICKNESS)
    fork_hub = _tapered_prism_xz(-FORK_REACH, -FORK_REACH + FORK_HUB_LENGTH, 0.016, 0.014, OUTER_SPAN)

    mount = rear_flange.union(body_block).union(center_rib).union(fork_hub)
    mount = _add_outer_fork_rails(
        mount,
        x0=-FORK_REACH + FORK_HUB_LENGTH,
        x1=-BOSS_RADIUS,
        z_size=0.014,
    )
    mount = _add_outer_joint_fork(
        mount,
        joint_x=0.0,
        lug_x0=-BOSS_RADIUS,
        lug_height=0.014,
    )
    return mount


def make_link_segment(
    *,
    length: float,
    body_height: float,
    bridge_height: float,
    lug_height: float,
) -> cq.Workplane:
    main_web = _tapered_prism_xz(
        BOSS_RADIUS * 0.55,
        length - FORK_REACH,
        body_height,
        bridge_height * 0.72,
        BODY_THICKNESS,
    )
    proximal_boss = _half_circle_xz(0.0, BOSS_RADIUS, CENTER_BOSS_LENGTH, side="front")
    fork_hub = _tapered_prism_xz(
        length - FORK_REACH,
        length - FORK_REACH + FORK_HUB_LENGTH,
        bridge_height,
        bridge_height * 0.90,
        OUTER_SPAN,
    )

    link = main_web.union(proximal_boss).union(fork_hub)
    link = _add_outer_fork_rails(
        link,
        x0=length - FORK_REACH + FORK_HUB_LENGTH,
        x1=length - BOSS_RADIUS,
        z_size=lug_height,
    )
    link = _add_outer_joint_fork(
        link,
        joint_x=length,
        lug_x0=length - BOSS_RADIUS,
        lug_height=lug_height,
    )
    return link


def make_distal_segment() -> cq.Workplane:
    tip_start = DISTAL_LENGTH - 0.013
    main_web = _tapered_prism_xz(BOSS_RADIUS * 0.55, tip_start, 0.016, 0.012, BODY_THICKNESS)
    proximal_boss = _half_circle_xz(0.0, BOSS_RADIUS, CENTER_BOSS_LENGTH, side="front")
    tip_root = _tapered_prism_xz(tip_start - 0.006, tip_start, 0.014, 0.018, CENTER_BOSS_LENGTH)
    square_tip = _rect_xz(tip_start, DISTAL_LENGTH, 0.018, CENTER_BOSS_LENGTH)

    return main_web.union(proximal_boss).union(tip_root).union(square_tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_gripper_finger_chain")

    mount_material = model.material("mount_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    link_material = model.material("link_steel", rgba=(0.63, 0.65, 0.68, 1.0))

    root_mount = model.part("root_mount")
    root_mount.visual(
        mesh_from_cadquery(make_root_mount(), "root_mount"),
        origin=Origin(),
        material=mount_material,
        name="mount_shell",
    )

    proximal = model.part("proximal_segment")
    proximal.visual(
        mesh_from_cadquery(
            make_link_segment(
                length=PROXIMAL_LENGTH,
                body_height=0.026,
                bridge_height=0.024,
                lug_height=0.022,
            ),
            "proximal_segment",
        ),
        origin=Origin(),
        material=link_material,
        name="proximal_shell",
    )

    middle = model.part("middle_segment")
    middle.visual(
        mesh_from_cadquery(
            make_link_segment(
                length=MIDDLE_LENGTH,
                body_height=0.024,
                bridge_height=0.022,
                lug_height=0.020,
            ),
            "middle_segment",
        ),
        origin=Origin(),
        material=link_material,
        name="middle_shell",
    )

    distal = model.part("distal_segment")
    distal.visual(
        mesh_from_cadquery(make_distal_segment(), "distal_segment"),
        origin=Origin(),
        material=link_material,
        name="distal_shell",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=root_mount,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_mount = object_model.get_part("root_mount")
    proximal = object_model.get_part("proximal_segment")
    middle = object_model.get_part("middle_segment")
    distal = object_model.get_part("distal_segment")

    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

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

    ctx.expect_contact(root_mount, proximal, name="root_mount_contacts_proximal")
    ctx.expect_contact(proximal, middle, name="proximal_contacts_middle")
    ctx.expect_contact(middle, distal, name="middle_contacts_distal")

    same_plane_axes = (
        root_to_proximal.axis == (0.0, 1.0, 0.0)
        and proximal_to_middle.axis == (0.0, 1.0, 0.0)
        and middle_to_distal.axis == (0.0, 1.0, 0.0)
    )
    ctx.check(
        "joints_share_one_bending_axis",
        same_plane_axes,
        details=(
            f"axes are {root_to_proximal.axis}, "
            f"{proximal_to_middle.axis}, {middle_to_distal.axis}"
        ),
    )

    rest_tip = ctx.part_world_position(distal)
    with ctx.pose(
        {
            root_to_proximal: 0.12,
            proximal_to_middle: 0.16,
            middle_to_distal: 0.12,
        }
    ):
        curled_tip = ctx.part_world_position(distal)
        bends_in_plane = (
            rest_tip is not None
            and curled_tip is not None
            and curled_tip[2] < rest_tip[2] - 0.004
            and abs(curled_tip[1] - rest_tip[1]) < 0.001
        )
        ctx.check(
            "finger_curls_downward_in_one_plane",
            bends_in_plane,
            details=f"rest={rest_tip}, curled={curled_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
