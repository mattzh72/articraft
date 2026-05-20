from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _palm_plate_geometry():
    outer = rounded_rect_profile(0.220, 0.170, 0.014, corner_segments=8)
    holes = [
        _offset_profile(rounded_rect_profile(0.095, 0.024, 0.008, corner_segments=6), -0.018, -0.043),
        _offset_profile(rounded_rect_profile(0.082, 0.026, 0.008, corner_segments=6), -0.020, 0.000),
        _offset_profile(rounded_rect_profile(0.105, 0.024, 0.008, corner_segments=6), -0.014, 0.043),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, 0.048, cap=True, center=True)


def _link_plate_geometry(length: float, *, has_distal_fork: bool):
    """A slim open-frame finger link: side-view slot extruded along the hinge axis."""
    body_start = 0.012
    body_end = length - (0.026 if has_distal_fork else 0.006)
    body_width = max(body_end - body_start, 0.026)
    body_center = (body_start + body_end) * 0.5
    outer = _offset_profile(
        rounded_rect_profile(body_width, 0.034, 0.015, corner_segments=8),
        body_center,
        0.0,
    )
    slot_width = max(body_width - 0.030, 0.010)
    holes = [
        _offset_profile(
            rounded_rect_profile(slot_width, 0.012, 0.005, corner_segments=5),
            body_center,
            0.0,
        )
    ]
    geom = ExtrudeWithHolesGeometry(outer, holes, 0.012, cap=True, center=True)
    # The extrude helper makes thickness along +Z; rotate it so thickness lies
    # along local Y while the profile remains visible in the finger curl plane.
    geom.rotate_x(-math.pi / 2.0)
    return geom


def _add_axis_cylinder(part, *, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_link_visuals(part, *, length: float, has_distal_fork: bool, frame_mat, metal_mat, rubber_mat):
    part.visual(
        mesh_from_geometry(
            _link_plate_geometry(length, has_distal_fork=has_distal_fork),
            f"{part.name}_open_plate",
        ),
        material=frame_mat,
        name="open_plate",
    )
    _add_axis_cylinder(
        part,
        radius=0.014,
        length=0.024,
        xyz=(0.0, 0.0, 0.0),
        material=metal_mat,
        name="proximal_barrel",
    )

    if has_distal_fork:
        for side, y in (("side_a", -0.017), ("side_b", 0.017)):
            _add_axis_cylinder(
                part,
                radius=0.014,
                length=0.010,
                xyz=(length, y, 0.0),
                material=metal_mat,
                name=f"distal_barrel_{side}",
            )
            part.visual(
                Box((0.024, 0.012, 0.018)),
                origin=Origin(xyz=(length - 0.026, y * 0.62, 0.0)),
                material=frame_mat,
                name=f"distal_web_{side}",
            )
    else:
        part.visual(
            Box((0.020, 0.012, 0.020)),
            origin=Origin(xyz=(length, 0.0, 0.0)),
            material=frame_mat,
            name="tip_neck",
        )
        part.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(length + 0.010, 0.0, -0.001)),
            material=rubber_mat,
            name="tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_dual_finger_rig")

    frame_mat = model.material("anodized_charcoal", color=(0.08, 0.085, 0.09, 1.0))
    metal_mat = model.material("brushed_steel", color=(0.62, 0.66, 0.68, 1.0))
    rubber_mat = model.material("matte_rubber", color=(0.015, 0.016, 0.017, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_geometry(_palm_plate_geometry(), "palm_lightened_block"),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=frame_mat,
        name="lightened_block",
    )

    # Two clevis-style knuckle towers on the grounded palm block.
    hinge_x = 0.086
    hinge_z = 0.096
    finger_y = [-0.038, 0.038]
    for idx, y0 in enumerate(finger_y):
        for side_name, side_y in (("side_a", y0 - 0.017), ("side_b", y0 + 0.017)):
            palm.visual(
                Box((0.036, 0.008, 0.064)),
                origin=Origin(xyz=(hinge_x, side_y, 0.078)),
                material=frame_mat,
                name=f"knuckle_cheek_{idx}_{side_name}",
            )
            _add_axis_cylinder(
                palm,
                radius=0.0135,
                length=0.010,
                xyz=(hinge_x, side_y, hinge_z),
                material=metal_mat,
                name=f"knuckle_barrel_{idx}_{side_name}",
            )
        palm.visual(
            Box((0.050, 0.048, 0.010)),
            origin=Origin(xyz=(hinge_x - 0.006, y0, 0.052)),
            material=frame_mat,
            name=f"tower_foot_{idx}",
        )

    finger_lengths = {
        0: (0.102, 0.078, 0.060),
        1: (0.094, 0.084, 0.070),
    }

    for idx, y0 in enumerate(finger_y):
        proximal = model.part(f"finger_{idx}_proximal")
        middle = model.part(f"finger_{idx}_middle")
        distal = model.part(f"finger_{idx}_distal")
        proximal_len, middle_len, distal_len = finger_lengths[idx]

        _add_link_visuals(
            proximal,
            length=proximal_len,
            has_distal_fork=True,
            frame_mat=frame_mat,
            metal_mat=metal_mat,
            rubber_mat=rubber_mat,
        )
        _add_link_visuals(
            middle,
            length=middle_len,
            has_distal_fork=True,
            frame_mat=frame_mat,
            metal_mat=metal_mat,
            rubber_mat=rubber_mat,
        )
        _add_link_visuals(
            distal,
            length=distal_len,
            has_distal_fork=False,
            frame_mat=frame_mat,
            metal_mat=metal_mat,
            rubber_mat=rubber_mat,
        )

        model.articulation(
            f"finger_{idx}_knuckle",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(hinge_x, y0, hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=0.0, upper=0.82),
        )
        model.articulation(
            f"finger_{idx}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(proximal_len, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.95),
        )
        model.articulation(
            f"finger_{idx}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(middle_len, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=0.88),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    joints = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE]
    ctx.check(
        "two independent three-joint fingers",
        len(joints) == 6
        and all(object_model.get_articulation(f"finger_{idx}_{name}") for idx in (0, 1) for name in ("knuckle", "middle_joint", "tip_joint")),
        details=f"revolute_joints={[joint.name for joint in joints]}",
    )

    f0_knuckle = object_model.get_articulation("finger_0_knuckle")
    f1_knuckle = object_model.get_articulation("finger_1_knuckle")
    f0_mid = object_model.get_articulation("finger_0_middle_joint")
    f1_mid = object_model.get_articulation("finger_1_middle_joint")
    f0_tip_joint = object_model.get_articulation("finger_0_tip_joint")
    f1_tip_joint = object_model.get_articulation("finger_1_tip_joint")
    f0_prox = object_model.get_part("finger_0_proximal")
    f1_prox = object_model.get_part("finger_1_proximal")
    f0_distal = object_model.get_part("finger_0_distal")
    f1_distal = object_model.get_part("finger_1_distal")

    ctx.expect_origin_gap(
        f1_prox,
        f0_prox,
        axis="y",
        min_gap=0.070,
        max_gap=0.082,
        name="two fingers are side-by-side on the palm",
    )
    ctx.check(
        "finger link lengths are deliberately unequal",
        abs(f0_mid.origin.xyz[0] - f1_mid.origin.xyz[0]) > 0.006
        and abs(f0_tip_joint.origin.xyz[0] - f1_tip_joint.origin.xyz[0]) > 0.004,
        details=f"proximal={f0_mid.origin.xyz[0], f1_mid.origin.xyz[0]}, middle={f0_tip_joint.origin.xyz[0], f1_tip_joint.origin.xyz[0]}",
    )

    rest_f0 = ctx.part_world_position(f0_distal)
    rest_f1 = ctx.part_world_position(f1_distal)
    with ctx.pose({f0_knuckle: 0.55}):
        moved_f0 = ctx.part_world_position(f0_distal)
        moved_f1 = ctx.part_world_position(f1_distal)
    ctx.check(
        "finger_0 knuckle curls without driving finger_1",
        rest_f0 is not None
        and rest_f1 is not None
        and moved_f0 is not None
        and moved_f1 is not None
        and moved_f0[2] < rest_f0[2] - 0.020
        and abs(moved_f1[2] - rest_f1[2]) < 1e-6,
        details=f"rest_f0={rest_f0}, moved_f0={moved_f0}, rest_f1={rest_f1}, moved_f1={moved_f1}",
    )

    with ctx.pose({f1_mid: 0.55}):
        moved_mid_f1 = ctx.part_world_position(f1_distal)
    ctx.check(
        "finger_1 middle joint moves its distal link",
        rest_f1 is not None and moved_mid_f1 is not None and moved_mid_f1[2] < rest_f1[2] - 0.020,
        details=f"rest={rest_f1}, moved={moved_mid_f1}",
    )

    def _aabb_center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    rest_tip_aabb = ctx.part_element_world_aabb(f0_distal, elem="tip_pad")
    with ctx.pose({f0_tip_joint: 0.55}):
        flexed_tip_aabb = ctx.part_element_world_aabb(f0_distal, elem="tip_pad")
    ctx.check(
        "distal revolute joint curls the fingertip",
        _aabb_center_z(rest_tip_aabb) is not None
        and _aabb_center_z(flexed_tip_aabb) is not None
        and _aabb_center_z(flexed_tip_aabb) < _aabb_center_z(rest_tip_aabb) - 0.010,
        details=f"rest_tip={rest_tip_aabb}, flexed_tip={flexed_tip_aabb}",
    )

    # Touching hinge knuckles are represented by separate clearanced barrels, so
    # no overlap allowance is required for the rest pose.
    return ctx.report()


object_model = build_object_model()
