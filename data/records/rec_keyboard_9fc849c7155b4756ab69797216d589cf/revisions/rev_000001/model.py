from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_TOP_Z = 0.024
KEY_REST_Z = 0.031
KEY_TRAVEL = 0.003


def _build_case_shape() -> cq.Workplane:
    """Continuous Alice-style shell with a shallow V notch at the typist edge."""
    footprint = [
        (-0.300, -0.072),
        (-0.210, -0.112),
        (-0.080, -0.092),
        (-0.026, -0.058),
        (0.000, -0.030),
        (0.026, -0.058),
        (0.080, -0.092),
        (0.210, -0.112),
        (0.300, -0.072),
        (0.286, 0.118),
        (0.070, 0.136),
        (0.000, 0.100),
        (-0.070, 0.136),
        (-0.286, 0.118),
    ]
    shell = cq.Workplane("XY").polyline(footprint).close().extrude(CASE_TOP_Z)
    try:
        shell = shell.edges("|Z").fillet(0.006)
    except Exception:
        # Preserve the authored silhouette even if the tight concave center
        # notch defeats a backend fillet operation.
        shell = cq.Workplane("XY").polyline(footprint).close().extrude(CASE_TOP_Z)
    return shell


def _build_keycap_shape(width: float, depth: float, height: float) -> cq.Workplane:
    """Low-profile key cap with softened vertical edges."""
    keycap = cq.Workplane("XY").box(width, depth, height)
    try:
        return keycap.edges("|Z").fillet(min(0.003, width * 0.18, depth * 0.18))
    except Exception:
        return keycap


def _field_to_world(
    field_origin: tuple[float, float],
    yaw: float,
    local_x: float,
    local_y: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        field_origin[0] + local_x * cos_yaw - local_y * sin_yaw,
        field_origin[1] + local_x * sin_yaw + local_y * cos_yaw,
    )


def _add_hinge_mount(
    case,
    material,
    *,
    ear_0_name: str,
    ear_1_name: str,
    clip_name: str,
    x: float,
    y: float,
) -> None:
    """Two fork ears and a rear keeper under one back corner."""
    for visual_name, offset_x in ((ear_0_name, -0.016), (ear_1_name, 0.016)):
        case.visual(
            Box((0.006, 0.022, 0.014)),
            origin=Origin(xyz=(x + offset_x, y, -0.003)),
            material=material,
            name=visual_name,
        )
    case.visual(
        Box((0.042, 0.006, 0.004)),
        origin=Origin(xyz=(x, y + 0.012, -0.002)),
        material=material,
        name=clip_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alice_ergonomic_keyboard")

    case_mat = model.material("warm_graphite_case", rgba=(0.13, 0.125, 0.14, 1.0))
    deck_mat = model.material("recessed_black_deck", rgba=(0.035, 0.038, 0.043, 1.0))
    key_mat = model.material("matte_ivory_keycaps", rgba=(0.88, 0.86, 0.80, 1.0))
    home_key_mat = model.material("slate_home_keycaps", rgba=(0.56, 0.58, 0.58, 1.0))
    accent_mat = model.material("center_accent", rgba=(0.42, 0.34, 0.30, 1.0))
    rubber_mat = model.material("dark_rubber_feet", rgba=(0.025, 0.025, 0.027, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_build_case_shape(), "continuous_notched_case", tolerance=0.0008),
        material=case_mat,
        name="continuous_shell",
    )

    fields = (
        ("left", (-0.128, 0.005), math.radians(12.0)),
        ("right", (0.128, 0.005), math.radians(-12.0)),
    )

    # Two inset, inward-canted key fields make the single case read as an Alice
    # layout instead of a conventional rectangular keyboard.
    for side, origin_xy, yaw in fields:
        well_name = "left_key_well" if side == "left" else "right_key_well"
        rear_lip_name = "left_rear_lip" if side == "left" else "right_rear_lip"
        case.visual(
            Box((0.214, 0.156, 0.003)),
            origin=Origin(xyz=(origin_xy[0], origin_xy[1], CASE_TOP_Z + 0.0015), rpy=(0.0, 0.0, yaw)),
            material=deck_mat,
            name=well_name,
        )
        lip_x, lip_y = _field_to_world(origin_xy, yaw, 0.0, 0.084)
        case.visual(
            Box((0.224, 0.010, 0.006)),
            origin=Origin(xyz=(lip_x, lip_y, CASE_TOP_Z + 0.002), rpy=(0.0, 0.0, yaw)),
            material=case_mat,
            name=rear_lip_name,
        )
        thumb_local_x = 0.056 if side == "left" else -0.056
        thumb_x, thumb_y = _field_to_world(origin_xy, yaw, thumb_local_x, -0.078)
        thumb_deck_name = "left_thumb_deck" if side == "left" else "right_thumb_deck"
        case.visual(
            Box((0.126, 0.068, 0.003)),
            origin=Origin(xyz=(thumb_x, thumb_y, CASE_TOP_Z + 0.0015), rpy=(0.0, 0.0, yaw)),
            material=deck_mat,
            name=thumb_deck_name,
        )

    # Raised center spine and colored separator emphasize the notch and the split.
    case.visual(
        Box((0.018, 0.090, 0.006)),
        origin=Origin(xyz=(0.000, 0.026, CASE_TOP_Z + 0.003)),
        material=case_mat,
        name="center_spine",
    )
    case.visual(
        Box((0.010, 0.036, 0.004)),
        origin=Origin(xyz=(0.000, -0.054, CASE_TOP_Z + 0.002), rpy=(0.0, 0.0, math.radians(45.0))),
        material=accent_mat,
        name="notch_bevel_0",
    )
    case.visual(
        Box((0.010, 0.036, 0.004)),
        origin=Origin(xyz=(0.000, -0.054, CASE_TOP_Z + 0.002), rpy=(0.0, 0.0, math.radians(-45.0))),
        material=accent_mat,
        name="notch_bevel_1",
    )

    # Static rubber pads at the front keep the case visually grounded while the
    # rear tent feet do the adjustable support work.
    for index, x in enumerate((-0.210, 0.210)):
        case.visual(
            Box((0.070, 0.018, 0.006)),
            origin=Origin(xyz=(x, -0.086, -0.003)),
            material=rubber_mat,
            name=f"front_pad_{index}",
        )

    left_hinge_x = -0.232
    right_hinge_x = 0.232
    hinge_y = 0.102
    hinge_z = -0.001
    _add_hinge_mount(
        case,
        case_mat,
        ear_0_name="left_hinge_mount_ear_0",
        ear_1_name="left_hinge_mount_ear_1",
        clip_name="left_hinge_mount_rear_clip",
        x=left_hinge_x,
        y=hinge_y,
    )
    _add_hinge_mount(
        case,
        case_mat,
        ear_0_name="right_hinge_mount_ear_0",
        ear_1_name="right_hinge_mount_ear_1",
        clip_name="right_hinge_mount_rear_clip",
        x=right_hinge_x,
        y=hinge_y,
    )

    normal_key_mesh = mesh_from_cadquery(
        _build_keycap_shape(0.025, 0.023, 0.008),
        "normal_keycap",
        tolerance=0.0006,
    )
    home_key_mesh = mesh_from_cadquery(
        _build_keycap_shape(0.025, 0.023, 0.0085),
        "home_keycap",
        tolerance=0.0006,
    )
    thumb_key_mesh = mesh_from_cadquery(
        _build_keycap_shape(0.046, 0.026, 0.008),
        "thumb_keycap",
        tolerance=0.0006,
    )

    row_ys = (-0.052, -0.019, 0.014, 0.047)
    col_xs = (-0.080, -0.048, -0.016, 0.016, 0.048, 0.080)
    for side, field_origin, yaw in fields:
        for row_index, local_y in enumerate(row_ys):
            for col_index, local_x in enumerate(col_xs):
                world_x, world_y = _field_to_world(field_origin, yaw, local_x, local_y)
                key = model.part(f"{side}_key_{row_index}_{col_index}")
                home_key = row_index == 2 and col_index in (2, 3)
                key.visual(
                    home_key_mesh if home_key else normal_key_mesh,
                    origin=Origin(xyz=(0.0, 0.0, 0.004)),
                    material=home_key_mat if home_key else key_mat,
                    name="keycap",
                )
                key.visual(
                    Box((0.006, 0.006, 0.007)),
                    origin=Origin(xyz=(0.0, 0.0, -0.0005)),
                    material=deck_mat,
                    name="stem",
                )
                model.articulation(
                    f"case_to_{side}_key_{row_index}_{col_index}",
                    ArticulationType.PRISMATIC,
                    parent=case,
                    child=key,
                    origin=Origin(xyz=(world_x, world_y, KEY_REST_Z), rpy=(0.0, 0.0, yaw)),
                    axis=(0.0, 0.0, -1.0),
                    motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
                )

        # Larger thumb keys follow the inner front curve of each half.
        for thumb_index, (local_x, local_y) in enumerate(((0.028, -0.088), (0.078, -0.078))):
            if side == "right":
                local_x = -local_x
            world_x, world_y = _field_to_world(field_origin, yaw, local_x, local_y)
            thumb = model.part(f"{side}_thumb_{thumb_index}")
            thumb.visual(
                thumb_key_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.004)),
                material=key_mat,
                name="keycap",
            )
            thumb.visual(
                Box((0.007, 0.007, 0.007)),
                origin=Origin(xyz=(0.0, 0.0, -0.0005)),
                material=deck_mat,
                name="stem",
            )
            model.articulation(
                f"case_to_{side}_thumb_{thumb_index}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=thumb,
                origin=Origin(xyz=(world_x, world_y, KEY_REST_Z), rpy=(0.0, 0.0, yaw)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=2.5, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
            )

    for side, hinge_x in (("left", left_hinge_x), ("right", right_hinge_x)):
        foot = model.part(f"{side}_tent_foot")
        foot.visual(
            Cylinder(radius=0.004, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_mat,
            name="pin",
        )
        foot.visual(
            Box((0.018, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, -0.008, -0.004)),
            material=rubber_mat,
            name="hinge_tab",
        )
        foot.visual(
            Box((0.020, 0.064, 0.006)),
            origin=Origin(xyz=(0.0, -0.038, -0.006)),
            material=rubber_mat,
            name="leg",
        )
        foot.visual(
            Box((0.038, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, -0.066, -0.009)),
            material=rubber_mat,
            name="toe_pad",
        )
        model.articulation(
            f"case_to_{side}_tent_foot",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    left_key = object_model.get_part("left_key_2_2")
    right_key = object_model.get_part("right_key_2_3")
    left_key_joint = object_model.get_articulation("case_to_left_key_2_2")
    left_foot = object_model.get_part("left_tent_foot")
    right_foot = object_model.get_part("right_tent_foot")
    left_foot_joint = object_model.get_articulation("case_to_left_tent_foot")
    right_foot_joint = object_model.get_articulation("case_to_right_tent_foot")

    ctx.expect_gap(
        left_key,
        case,
        axis="z",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="keycap",
        negative_elem="left_key_well",
        name="left keycaps float above the recessed well",
    )
    ctx.expect_gap(
        right_key,
        case,
        axis="z",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem="keycap",
        negative_elem="right_key_well",
        name="right keycaps float above the recessed well",
    )

    rest_key_pos = ctx.part_world_position(left_key)
    with ctx.pose({left_key_joint: KEY_TRAVEL}):
        depressed_key_pos = ctx.part_world_position(left_key)
        ctx.expect_gap(
            left_key,
            case,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            positive_elem="keycap",
            negative_elem="left_key_well",
            name="pressed key stops just above the deck",
        )
    ctx.check(
        "key plunger moves downward",
        rest_key_pos is not None
        and depressed_key_pos is not None
        and depressed_key_pos[2] < rest_key_pos[2] - 0.0025,
        details=f"rest={rest_key_pos}, depressed={depressed_key_pos}",
    )

    ctx.expect_within(
        left_foot,
        case,
        axes="x",
        inner_elem="pin",
        outer_elem="left_hinge_mount_rear_clip",
        margin=0.002,
        name="left tent pin is retained between hinge ears",
    )
    ctx.expect_within(
        right_foot,
        case,
        axes="x",
        inner_elem="pin",
        outer_elem="right_hinge_mount_rear_clip",
        margin=0.002,
        name="right tent pin is retained between hinge ears",
    )

    left_folded = ctx.part_element_world_aabb(left_foot, elem="toe_pad")
    right_folded = ctx.part_element_world_aabb(right_foot, elem="toe_pad")
    with ctx.pose({left_foot_joint: 1.05, right_foot_joint: 1.05}):
        left_deployed = ctx.part_element_world_aabb(left_foot, elem="toe_pad")
        right_deployed = ctx.part_element_world_aabb(right_foot, elem="toe_pad")

    ctx.check(
        "left rear tent foot swings downward",
        left_folded is not None
        and left_deployed is not None
        and left_deployed[0][2] < left_folded[0][2] - 0.035,
        details=f"folded={left_folded}, deployed={left_deployed}",
    )
    ctx.check(
        "right rear tent foot swings downward",
        right_folded is not None
        and right_deployed is not None
        and right_deployed[0][2] < right_folded[0][2] - 0.035,
        details=f"folded={right_folded}, deployed={right_deployed}",
    )

    return ctx.report()


object_model = build_object_model()
