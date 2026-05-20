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

PALM_W = 0.086
PALM_D = 0.036
PALM_H = 0.058
ROOT_MOUNT_W = 0.018
ROOT_MOUNT_D = 0.010
ROOT_MOUNT_H = 0.022
FINGER_ROOT_Z = 0.008
FINGER_ROOT_X = 0.024

HINGE_RADIUS = 0.0038
CENTER_GAP = 0.0066
ROOT_BARREL_LEN = 0.0058
YOKE_DEPTH = 0.008

PROXIMAL_LEN = 0.032
MIDDLE_LEN = 0.026
DISTAL_LEN = 0.021

LEFT_ROOT = (-FINGER_ROOT_X, PALM_D / 2.0 + ROOT_MOUNT_D, FINGER_ROOT_Z)
RIGHT_ROOT = (FINGER_ROOT_X, PALM_D / 2.0 + ROOT_MOUNT_D, FINGER_ROOT_Z)


def _make_palm_shape() -> cq.Workplane:
    palm = cq.Workplane("XY").box(PALM_W, PALM_D, PALM_H)

    for root_x in (-FINGER_ROOT_X, FINGER_ROOT_X):
        mount_center_y = PALM_D / 2.0 + ROOT_MOUNT_D / 2.0
        mount = (
            cq.Workplane("XY")
            .center(root_x, mount_center_y)
            .box(ROOT_MOUNT_W, ROOT_MOUNT_D, ROOT_MOUNT_H)
        )
        slot = (
            cq.Workplane("XY")
            .center(root_x, mount_center_y)
            .box(CENTER_GAP, ROOT_MOUNT_D + 0.002, ROOT_MOUNT_H + 0.002)
        )
        palm = palm.union(mount).cut(slot)

    return palm


def _make_link_shape(
    *,
    length: float,
    root_width: float,
    tip_width: float,
    thickness: float,
    include_yoke: bool,
) -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .moveTo(-root_width / 2.0, 0.0)
        .lineTo(root_width / 2.0, 0.0)
        .lineTo(tip_width / 2.0, length)
        .lineTo(-tip_width / 2.0, length)
        .close()
        .extrude(thickness / 2.0, both=True)
    )

    root_barrel = cq.Workplane("YZ").cylinder(
        ROOT_BARREL_LEN,
        HINGE_RADIUS,
        centered=(True, True, True),
    )
    link = body.union(root_barrel)

    if include_yoke:
        yoke_slot = (
            cq.Workplane("XY")
            .center(0.0, length - YOKE_DEPTH / 2.0)
            .box(CENTER_GAP, YOKE_DEPTH + 0.002, thickness + 0.002)
        )
        link = link.cut(yoke_slot)

    return link


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    inertia_box: tuple[float, float, float],
    mass: float,
    inertial_origin: Origin,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=f"{name}_shell",
    )
    part.inertial = Inertial.from_geometry(
        Box(inertia_box),
        mass=mass,
        origin=inertial_origin,
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_palm_two_finger_chain")

    model.material("palm_polymer", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("finger_alloy", rgba=(0.73, 0.75, 0.79, 1.0))

    palm = _add_mesh_part(
        model,
        name="palm",
        shape=_make_palm_shape(),
        mesh_name="palm_body",
        material="palm_polymer",
        inertia_box=(PALM_W, PALM_D + ROOT_MOUNT_D, PALM_H),
        mass=0.26,
        inertial_origin=Origin(xyz=(0.0, ROOT_MOUNT_D / 2.0, 0.0)),
    )

    left_proximal = _add_mesh_part(
        model,
        name="left_proximal",
        shape=_make_link_shape(
            length=PROXIMAL_LEN,
            root_width=0.015,
            tip_width=0.0135,
            thickness=0.015,
            include_yoke=True,
        ),
        mesh_name="left_proximal",
        material="finger_alloy",
        inertia_box=(0.015, PROXIMAL_LEN, 0.015),
        mass=0.034,
        inertial_origin=Origin(xyz=(0.0, PROXIMAL_LEN / 2.0, 0.0)),
    )
    left_middle = _add_mesh_part(
        model,
        name="left_middle",
        shape=_make_link_shape(
            length=MIDDLE_LEN,
            root_width=0.0135,
            tip_width=0.012,
            thickness=0.013,
            include_yoke=True,
        ),
        mesh_name="left_middle",
        material="finger_alloy",
        inertia_box=(0.0135, MIDDLE_LEN, 0.013),
        mass=0.024,
        inertial_origin=Origin(xyz=(0.0, MIDDLE_LEN / 2.0, 0.0)),
    )
    left_distal = _add_mesh_part(
        model,
        name="left_distal",
        shape=_make_link_shape(
            length=DISTAL_LEN,
            root_width=0.012,
            tip_width=0.010,
            thickness=0.011,
            include_yoke=False,
        ),
        mesh_name="left_distal",
        material="finger_alloy",
        inertia_box=(0.012, DISTAL_LEN, 0.011),
        mass=0.016,
        inertial_origin=Origin(xyz=(0.0, DISTAL_LEN / 2.0, 0.0)),
    )
    right_proximal = _add_mesh_part(
        model,
        name="right_proximal",
        shape=_make_link_shape(
            length=PROXIMAL_LEN,
            root_width=0.015,
            tip_width=0.0135,
            thickness=0.015,
            include_yoke=True,
        ),
        mesh_name="right_proximal",
        material="finger_alloy",
        inertia_box=(0.015, PROXIMAL_LEN, 0.015),
        mass=0.034,
        inertial_origin=Origin(xyz=(0.0, PROXIMAL_LEN / 2.0, 0.0)),
    )
    right_middle = _add_mesh_part(
        model,
        name="right_middle",
        shape=_make_link_shape(
            length=MIDDLE_LEN,
            root_width=0.0135,
            tip_width=0.012,
            thickness=0.013,
            include_yoke=True,
        ),
        mesh_name="right_middle",
        material="finger_alloy",
        inertia_box=(0.0135, MIDDLE_LEN, 0.013),
        mass=0.024,
        inertial_origin=Origin(xyz=(0.0, MIDDLE_LEN / 2.0, 0.0)),
    )
    right_distal = _add_mesh_part(
        model,
        name="right_distal",
        shape=_make_link_shape(
            length=DISTAL_LEN,
            root_width=0.012,
            tip_width=0.010,
            thickness=0.011,
            include_yoke=False,
        ),
        mesh_name="right_distal",
        material="finger_alloy",
        inertia_box=(0.012, DISTAL_LEN, 0.011),
        mass=0.016,
        inertial_origin=Origin(xyz=(0.0, DISTAL_LEN / 2.0, 0.0)),
    )

    for joint_name, parent, child, origin_xyz, limits in (
        (
            "palm_to_left_proximal",
            palm,
            left_proximal,
            LEFT_ROOT,
            MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.15),
        ),
        (
            "left_proximal_to_left_middle",
            left_proximal,
            left_middle,
            (0.0, PROXIMAL_LEN, 0.0),
            MotionLimits(effort=2.2, velocity=3.5, lower=0.0, upper=1.30),
        ),
        (
            "left_middle_to_left_distal",
            left_middle,
            left_distal,
            (0.0, MIDDLE_LEN, 0.0),
            MotionLimits(effort=1.8, velocity=4.0, lower=0.0, upper=1.20),
        ),
        (
            "palm_to_right_proximal",
            palm,
            right_proximal,
            RIGHT_ROOT,
            MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.15),
        ),
        (
            "right_proximal_to_right_middle",
            right_proximal,
            right_middle,
            (0.0, PROXIMAL_LEN, 0.0),
            MotionLimits(effort=2.2, velocity=3.5, lower=0.0, upper=1.30),
        ),
        (
            "right_middle_to_right_distal",
            right_middle,
            right_distal,
            (0.0, MIDDLE_LEN, 0.0),
            MotionLimits(effort=1.8, velocity=4.0, lower=0.0, upper=1.20),
        ),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=origin_xyz),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=limits,
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

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip = object_model.get_articulation("right_middle_to_right_distal")

    ctx.expect_origin_distance(
        left_proximal,
        right_proximal,
        axes="x",
        min_dist=0.040,
        max_dist=0.055,
        name="finger roots are spaced apart across the palm face",
    )
    ctx.expect_origin_gap(
        left_proximal,
        palm,
        axis="y",
        min_gap=0.020,
        max_gap=0.035,
        name="left root sits proud of the palm front face",
    )
    ctx.expect_origin_gap(
        right_proximal,
        palm,
        axis="y",
        min_gap=0.020,
        max_gap=0.035,
        name="right root sits proud of the palm front face",
    )
    ctx.expect_origin_gap(
        left_middle,
        left_proximal,
        axis="y",
        min_gap=0.026,
        max_gap=0.038,
        name="left proximal leads into left middle link",
    )
    ctx.expect_origin_gap(
        left_distal,
        left_middle,
        axis="y",
        min_gap=0.020,
        max_gap=0.032,
        name="left middle leads into left distal link",
    )
    ctx.expect_origin_gap(
        right_middle,
        right_proximal,
        axis="y",
        min_gap=0.026,
        max_gap=0.038,
        name="right proximal leads into right middle link",
    )
    ctx.expect_origin_gap(
        right_distal,
        right_middle,
        axis="y",
        min_gap=0.020,
        max_gap=0.032,
        name="right middle leads into right distal link",
    )

    rest_left = ctx.part_world_position(left_distal)
    rest_right = ctx.part_world_position(right_distal)
    with ctx.pose({left_root: 0.55, left_mid: 0.70, left_tip: 0.45}):
        curled_left = ctx.part_world_position(left_distal)
        carried_right = ctx.part_world_position(right_distal)
    ctx.check(
        "left finger curls independently",
        rest_left is not None
        and curled_left is not None
        and curled_left[2] < rest_left[2] - 0.012
        and carried_right is not None
        and rest_right is not None
        and abs(carried_right[0] - rest_right[0]) < 1e-6
        and abs(carried_right[1] - rest_right[1]) < 1e-6
        and abs(carried_right[2] - rest_right[2]) < 1e-6,
        details=(
            f"rest_left={rest_left}, curled_left={curled_left}, "
            f"rest_right={rest_right}, carried_right={carried_right}"
        ),
    )

    rest_left_again = ctx.part_world_position(left_distal)
    rest_right_again = ctx.part_world_position(right_distal)
    with ctx.pose({right_root: 0.55, right_mid: 0.70, right_tip: 0.45}):
        carried_left = ctx.part_world_position(left_distal)
        curled_right = ctx.part_world_position(right_distal)
    ctx.check(
        "right finger curls independently",
        rest_right_again is not None
        and curled_right is not None
        and curled_right[2] < rest_right_again[2] - 0.012
        and carried_left is not None
        and rest_left_again is not None
        and abs(carried_left[0] - rest_left_again[0]) < 1e-6
        and abs(carried_left[1] - rest_left_again[1]) < 1e-6
        and abs(carried_left[2] - rest_left_again[2]) < 1e-6,
        details=(
            f"rest_right={rest_right_again}, curled_right={curled_right}, "
            f"rest_left={rest_left_again}, carried_left={carried_left}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
