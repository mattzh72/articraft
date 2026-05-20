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


ROOT_GAP = 0.018
MID_GAP = 0.014
DIST_GAP = 0.011

PROXIMAL_SPAN = 0.052
MIDDLE_SPAN = 0.038
DISTAL_SPAN = 0.024


def box_at(length: float, width: float, height: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").box(length, width, height).translate(center)


def y_cylinder(radius: float, length: float, center_x: float = 0.0, center_z: float = 0.0):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x, length / 2.0, center_z))
    )


def xz_prism(points: list[tuple[float, float]], width: float):
    return cq.Workplane("XZ").polyline(points).close().extrude(width).translate((0.0, width / 2.0, 0.0))


def make_root_fork():
    back_block = box_at(0.028, 0.026, 0.020, (-0.022, 0.0, 0.0))
    cheek_len = 0.016
    cheek_w = 0.004
    cheek_center_y = 0.010
    left_cheek = box_at(cheek_len, cheek_w, 0.020, (-0.001, cheek_center_y, 0.0))
    right_cheek = box_at(cheek_len, cheek_w, 0.020, (-0.001, -cheek_center_y, 0.0))
    return back_block.union(left_cheek).union(right_cheek)


def make_proximal_link():
    barrel = y_cylinder(radius=0.0068, length=0.016)
    body = xz_prism(
        [
            (0.004, -0.0072),
            (0.013, -0.0076),
            (0.030, -0.0068),
            (0.043, -0.0062),
            (0.043, 0.0062),
            (0.030, 0.0068),
            (0.013, 0.0076),
            (0.004, 0.0072),
        ],
        width=0.013,
    )
    clevis_block = box_at(0.014, 0.019, 0.014, (0.049, 0.0, 0.0))
    clevis_slot = box_at(0.010, 0.013, 0.016, (0.051, 0.0, 0.0))
    clevis = clevis_block.cut(clevis_slot)
    return barrel.union(body).union(clevis)


def make_middle_link():
    barrel = y_cylinder(radius=0.0055, length=0.013)
    body = xz_prism(
        [
            (0.003, -0.0061),
            (0.010, -0.0065),
            (0.022, -0.0058),
            (0.032, -0.0052),
            (0.032, 0.0052),
            (0.022, 0.0058),
            (0.010, 0.0065),
            (0.003, 0.0061),
        ],
        width=0.011,
    )
    clevis_block = box_at(0.011, 0.016, 0.012, (0.0375, 0.0, 0.0))
    clevis_slot = box_at(0.008, 0.011, 0.014, (0.039, 0.0, 0.0))
    clevis = clevis_block.cut(clevis_slot)
    return barrel.union(body).union(clevis)


def make_distal_link():
    barrel = y_cylinder(radius=0.0048, length=0.011)
    body = xz_prism(
        [
            (0.003, -0.0052),
            (0.009, -0.0054),
            (0.017, -0.0050),
            (0.024, -0.0047),
            (0.024, 0.0047),
            (0.017, 0.0050),
            (0.009, 0.0054),
            (0.003, 0.0052),
        ],
        width=0.010,
    )
    return barrel.union(body)


def make_tip_pad():
    pad_block = box_at(0.007, 0.012, 0.009, (0.0035, 0.0, -0.0008))
    pad_nose = y_cylinder(radius=0.0045, length=0.012, center_x=0.0075, center_z=-0.0008)
    return pad_block.union(pad_nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_digit")

    anodized = model.material("anodized_aluminum", rgba=(0.66, 0.69, 0.73, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.36, 0.38, 0.42, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.16, 0.17, 0.18, 1.0))

    fork_root = model.part("fork_root")
    fork_root.visual(
        mesh_from_cadquery(make_root_fork(), "fork_root"),
        material=dark_metal,
        name="fork_root_shell",
    )

    proximal = model.part("proximal")
    proximal.visual(
        mesh_from_cadquery(make_proximal_link(), "proximal"),
        material=anodized,
        name="proximal_shell",
    )

    middle = model.part("middle")
    middle.visual(
        mesh_from_cadquery(make_middle_link(), "middle"),
        material=anodized,
        name="middle_shell",
    )

    distal = model.part("distal")
    distal.visual(
        mesh_from_cadquery(make_distal_link(), "distal"),
        material=anodized,
        name="distal_shell",
    )

    tip_pad = model.part("tip_pad")
    tip_pad.visual(
        mesh_from_cadquery(make_tip_pad(), "tip_pad"),
        material=pad_rubber,
        name="tip_pad_shell",
    )

    model.articulation(
        "root_to_proximal",
        ArticulationType.REVOLUTE,
        parent=fork_root,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.20, upper=1.35),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5, lower=0.0, upper=1.60),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "distal_to_tip_pad",
        ArticulationType.FIXED,
        parent=distal,
        child=tip_pad,
        origin=Origin(xyz=(DISTAL_SPAN, 0.0, -0.0010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork_root = object_model.get_part("fork_root")
    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")
    tip_pad = object_model.get_part("tip_pad")

    root_to_proximal = object_model.get_articulation("root_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")
    distal_to_tip_pad = object_model.get_articulation("distal_to_tip_pad")

    ctx.allow_overlap(
        fork_root,
        proximal,
        reason="Root fork captures the proximal knuckle with an intentionally shared hinge-journal volume.",
    )
    ctx.allow_overlap(
        proximal,
        middle,
        reason="Proximal and middle links use an intentionally shared coaxial knuckle volume at the hinge.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        reason="Middle and distal links use an intentionally shared coaxial knuckle volume at the hinge.",
    )

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

    ctx.check(
        "digit uses three revolute joints plus fixed tip mount",
        root_to_proximal.articulation_type == ArticulationType.REVOLUTE
        and proximal_to_middle.articulation_type == ArticulationType.REVOLUTE
        and middle_to_distal.articulation_type == ArticulationType.REVOLUTE
        and distal_to_tip_pad.articulation_type == ArticulationType.FIXED,
        "Unexpected articulation types in digit chain.",
    )

    ctx.expect_contact(fork_root, proximal, contact_tol=8e-4, name="root fork captures proximal")
    ctx.expect_overlap(
        fork_root,
        proximal,
        axes="yz",
        min_overlap=0.014,
        name="proximal stays nested in root fork envelope",
    )
    ctx.expect_contact(proximal, middle, contact_tol=8e-4, name="proximal clevis supports middle")
    ctx.expect_overlap(
        proximal,
        middle,
        axes="yz",
        min_overlap=0.010,
        name="middle stays nested in proximal clevis envelope",
    )
    ctx.expect_contact(middle, distal, contact_tol=8e-4, name="middle clevis supports distal")
    ctx.expect_overlap(
        middle,
        distal,
        axes="yz",
        min_overlap=0.008,
        name="distal stays nested in middle clevis envelope",
    )
    ctx.expect_contact(distal, tip_pad, contact_tol=8e-4, name="tip pad is mounted to distal link")
    ctx.expect_gap(
        tip_pad,
        distal,
        axis="x",
        min_gap=0.0,
        max_gap=8e-4,
        name="tip pad seats flush on distal front face",
    )
    ctx.expect_overlap(
        tip_pad,
        distal,
        axes="yz",
        min_overlap=0.008,
        name="tip pad is centered on distal face",
    )

    neutral_tip = ctx.part_world_position(tip_pad)
    with ctx.pose(
        {
            root_to_proximal: 0.75,
            proximal_to_middle: 0.95,
            middle_to_distal: 0.70,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in curled pose")
        curled_tip = ctx.part_world_position(tip_pad)
        ctx.check(
            "positive joint motion curls the digit",
            neutral_tip is not None
            and curled_tip is not None
            and curled_tip[0] < neutral_tip[0] - 0.020
            and curled_tip[2] < neutral_tip[2] - 0.030,
            f"neutral_tip={neutral_tip}, curled_tip={curled_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
