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


PLATE_T = 0.010
PLATE_W = 0.110
PLATE_H = 0.240

ROOT_Z = 0.165
ROOT_PIVOT_X = PLATE_T + 0.040
ROOT_EAR_W = 0.010
ROOT_GAP = 0.012
ROOT_OUTER_W = ROOT_GAP + 2.0 * ROOT_EAR_W
ROOT_EYE_R = 0.020
ROOT_ARM_H = 0.024
ROOT_PIN_R = 0.007

L1_PITCH = 0.170
L1_PROX_R = 0.018
L1_DIST_R = 0.020
L1_STRAP_H = 0.020
L1_ROOT_W = ROOT_GAP
L1_EAR_W = 0.010
L1_ELBOW_GAP = 0.012
L1_OUTER_W = L1_ELBOW_GAP + 2.0 * L1_EAR_W
L1_BODY_END = L1_PITCH - 0.034
L1_BRIDGE_START = L1_BODY_END - 0.006
L1_BRIDGE_END = L1_PITCH - 0.019
L1_EAR_START = L1_PITCH - 0.026
L1_PIN_R = 0.006

L2_WIDTH = L1_ELBOW_GAP
L2_PROX_R = 0.016
L2_STRAP_H = 0.018
L2_LENGTH = 0.135
L2_END_R = 0.013
L2_PIN_R = L1_PIN_R


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _solid_box(
    x_min: float,
    y_min: float,
    z_min: float,
    size_x: float,
    size_y: float,
    size_z: float,
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeBox(
            size_x,
            size_y,
            size_z,
            cq.Vector(x_min, y_min, z_min),
        )
    )


def _solid_cylinder_y(
    center_x: float,
    y_min: float,
    center_z: float,
    radius: float,
    length_y: float,
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length_y,
            cq.Vector(center_x, y_min, center_z),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _solid_cylinder_x(
    x_min: float,
    center_y: float,
    center_z: float,
    radius: float,
    length_x: float,
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length_x,
            cq.Vector(x_min, center_y, center_z),
            cq.Vector(1.0, 0.0, 0.0),
        )
    )


def make_backplate() -> cq.Workplane:
    bracket = _solid_box(0.0, -PLATE_W / 2.0, 0.0, PLATE_T, PLATE_W, PLATE_H)
    ear_y = ROOT_GAP / 2.0 + ROOT_EAR_W / 2.0
    arm_start_x = PLATE_T - 0.002
    arm_len = ROOT_PIVOT_X - arm_start_x
    for y_center in (-ear_y, ear_y):
        ear_arm = _solid_box(
            arm_start_x,
            y_center - ROOT_EAR_W / 2.0,
            ROOT_Z - ROOT_ARM_H / 2.0,
            arm_len,
            ROOT_EAR_W,
            ROOT_ARM_H,
        )
        ear_eye = _solid_cylinder_y(
            ROOT_PIVOT_X,
            y_center - ROOT_EAR_W / 2.0,
            ROOT_Z,
            ROOT_EYE_R,
            ROOT_EAR_W,
        )
        bracket = bracket.union(ear_arm).union(ear_eye)

    for y_off in (-0.032, 0.032):
        for z_off in (0.050, PLATE_H - 0.050):
            bracket = bracket.cut(
                _solid_cylinder_x(
                    -0.002,
                    y_off,
                    z_off,
                    0.006,
                    PLATE_T + 0.004,
                )
            )

    root_hole = _solid_cylinder_y(
        ROOT_PIVOT_X,
        -ROOT_OUTER_W / 2.0 - 0.002,
        ROOT_Z,
        ROOT_PIN_R,
        ROOT_OUTER_W + 0.004,
    )

    return bracket.cut(root_hole)


def make_first_link() -> cq.Workplane:
    link = _solid_cylinder_y(
        0.0,
        -L1_ROOT_W / 2.0,
        0.0,
        L1_PROX_R,
        L1_ROOT_W,
    )
    link = link.union(
        _solid_box(
            0.0,
            -L1_ROOT_W / 2.0,
            -L1_STRAP_H / 2.0,
            L1_BODY_END,
            L1_ROOT_W,
            L1_STRAP_H,
        )
    )
    link = link.union(
        _solid_box(
            L1_BRIDGE_START,
            -L1_OUTER_W / 2.0,
            -L1_STRAP_H / 2.0,
            L1_BRIDGE_END - L1_BRIDGE_START,
            L1_OUTER_W,
            L1_STRAP_H,
        )
    )

    ear_y = L1_ELBOW_GAP / 2.0 + L1_EAR_W / 2.0
    distal_arm_len = L1_PITCH - L1_EAR_START
    for y_center in (-ear_y, ear_y):
        distal_arm = _solid_box(
            L1_EAR_START,
            y_center - L1_EAR_W / 2.0,
            -L1_STRAP_H / 2.0,
            distal_arm_len,
            L1_EAR_W,
            L1_STRAP_H,
        )
        distal_eye = _solid_cylinder_y(
            L1_PITCH,
            y_center - L1_EAR_W / 2.0,
            0.0,
            L1_DIST_R,
            L1_EAR_W,
        )
        link = link.union(distal_arm).union(distal_eye)

    prox_hole = _solid_cylinder_y(
        0.0,
        -L1_OUTER_W / 2.0 - 0.002,
        0.0,
        ROOT_PIN_R,
        L1_OUTER_W + 0.004,
    )
    distal_hole = _solid_cylinder_y(
        L1_PITCH,
        -L1_OUTER_W / 2.0 - 0.002,
        0.0,
        L1_PIN_R,
        L1_OUTER_W + 0.004,
    )

    return link.cut(prox_hole).cut(distal_hole)


def make_second_link() -> cq.Workplane:
    link = _solid_cylinder_y(
        0.0,
        -L2_WIDTH / 2.0,
        0.0,
        L2_PROX_R,
        L2_WIDTH,
    )
    link = link.union(
        _solid_box(
            0.0,
            -L2_WIDTH / 2.0,
            -L2_STRAP_H / 2.0,
            L2_LENGTH,
            L2_WIDTH,
            L2_STRAP_H,
        )
    )
    link = link.union(
        _solid_cylinder_y(
            L2_LENGTH,
            -L2_WIDTH / 2.0,
            0.0,
            L2_END_R,
            L2_WIDTH,
        )
    )
    prox_hole = _solid_cylinder_y(
        0.0,
        -L2_WIDTH / 2.0 - 0.002,
        0.0,
        L2_PIN_R,
        L2_WIDTH + 0.004,
    )
    return link.cut(prox_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_two_joint_chain")

    plate_finish = model.material("plate_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    link_finish = model.material("link_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    link_finish_dark = model.material("link_finish_dark", rgba=(0.62, 0.65, 0.69, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(make_backplate(), "backplate"),
        material=plate_finish,
        name="backplate_shell",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(make_first_link(), "link_1"),
        material=link_finish,
        name="link_1_shell",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(make_second_link(), "link_2"),
        material=link_finish_dark,
        name="link_2_shell",
    )

    model.articulation(
        "backplate_to_link_1",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=link_1,
        origin=Origin(xyz=(ROOT_PIVOT_X, 0.0, ROOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.10,
            upper=1.20,
        ),
    )

    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(L1_PITCH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.5,
            lower=-1.45,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    root_joint = object_model.get_articulation("backplate_to_link_1")
    elbow_joint = object_model.get_articulation("link_1_to_link_2")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        backplate,
        link_1,
        reason="Root clevis and first link share the revolute knuckle envelope; the pin is implied rather than authored as a separate part.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Elbow clevis and second link share the revolute knuckle envelope; the hinge pin is intentionally not modeled as its own part.",
    )

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
        "joints_are_revolute",
        root_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE,
        "Both joints should be authored as revolute joints.",
    )
    ctx.check(
        "joint_axes_are_parallel",
        tuple(root_joint.axis) == tuple(elbow_joint.axis),
        f"Expected parallel joint axes, got {root_joint.axis} and {elbow_joint.axis}.",
    )

    ctx.expect_contact(backplate, link_1, name="root_joint_surfaces_touch")
    ctx.expect_contact(link_1, link_2, name="elbow_joint_surfaces_touch")
    ctx.expect_gap(link_2, backplate, axis="x", min_gap=0.12, name="distal_link_stands_off_wall_rest")

    with ctx.pose({root_joint: 0.85, elbow_joint: -1.05}):
        ctx.expect_contact(backplate, link_1, name="root_joint_surfaces_touch_bent_pose")
        ctx.expect_contact(link_1, link_2, name="elbow_joint_surfaces_touch_bent_pose")
        ctx.expect_gap(link_2, backplate, axis="x", min_gap=0.07, name="distal_link_stands_off_wall_bent_pose")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_bent_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
