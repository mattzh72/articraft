from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


KEY_PITCH = 0.01905
KEY_DEPTH = 0.0160
KEY_GAP = 0.0030
KEY_JOINT_Z = 0.0320


def _keycap_mesh(width: float, depth: float, height: float, mesh_name: str):
    """A gently tapered, rounded-rectangle gaming keycap body."""
    lower = rounded_rect_profile(width, depth, radius=min(0.0024, width * 0.18), corner_segments=4)
    upper = rounded_rect_profile(
        max(width - 0.0032, width * 0.72),
        max(depth - 0.0030, depth * 0.72),
        radius=min(0.0020, width * 0.15),
        corner_segments=4,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tenkeyless_gaming_keyboard")

    case_mat = model.material("case_charcoal", rgba=(0.055, 0.060, 0.070, 1.0))
    plate_mat = model.material("top_plate_black", rgba=(0.015, 0.017, 0.022, 1.0))
    key_mat = model.material("keycap_graphite", rgba=(0.040, 0.043, 0.050, 1.0))
    accent_key_mat = model.material("accent_key_red", rgba=(0.72, 0.055, 0.065, 1.0))
    arrow_key_mat = model.material("arrow_key_blue", rgba=(0.060, 0.22, 0.75, 1.0))
    stem_mat = model.material("switch_stem_red", rgba=(0.75, 0.035, 0.045, 1.0))
    rubber_mat = model.material("rubber_black", rgba=(0.018, 0.018, 0.020, 1.0))
    roller_mat = model.material("knurled_roller", rgba=(0.52, 0.54, 0.57, 1.0))
    rgb_mat = model.material("diffused_rgb", rgba=(0.18, 0.52, 0.95, 0.65))

    case = model.part("case")
    case.visual(
        Box((0.385, 0.150, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=case_mat,
        name="lower_case",
    )
    case.visual(
        Box((0.374, 0.139, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=plate_mat,
        name="top_plate",
    )
    case.visual(
        Box((0.385, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.078, 0.018)),
        material=case_mat,
        name="front_lip",
    )
    case.visual(
        Box((0.385, 0.009, 0.013)),
        origin=Origin(xyz=(0.0, 0.075, 0.017)),
        material=case_mat,
        name="rear_lip",
    )
    case.visual(
        Box((0.006, 0.150, 0.010)),
        origin=Origin(xyz=(-0.195, 0.0, 0.016)),
        material=case_mat,
        name="side_lip_0",
    )
    case.visual(
        Box((0.006, 0.150, 0.010)),
        origin=Origin(xyz=(0.195, 0.0, 0.016)),
        material=case_mat,
        name="side_lip_1",
    )
    case.visual(
        Box((0.338, 0.004, 0.0018)),
        origin=Origin(xyz=(-0.018, -0.064, 0.0269)),
        material=rgb_mat,
        name="front_rgb_diffuser",
    )
    case.visual(
        Box((0.020, 0.004, 0.0018)),
        origin=Origin(xyz=(0.160, 0.068, 0.0269)),
        material=rgb_mat,
        name="roller_rgb_marker",
    )

    # Raised cradle for the upper-right media roller.
    case.visual(
        Box((0.050, 0.009, 0.006)),
        origin=Origin(xyz=(0.160, 0.061, 0.029)),
        material=case_mat,
        name="roller_cradle",
    )
    case.visual(
        Box((0.006, 0.015, 0.014)),
        origin=Origin(xyz=(0.136, 0.061, 0.033)),
        material=case_mat,
        name="roller_support_0",
    )
    case.visual(
        Box((0.006, 0.015, 0.014)),
        origin=Origin(xyz=(0.184, 0.061, 0.033)),
        material=case_mat,
        name="roller_support_1",
    )

    # Two clipped hinge yokes under the rear edge for the flip-out feet.
    foot_hinges = [(-0.122, 0.061, -0.005), (0.122, 0.061, -0.005)]
    for foot_index, (x, y, z) in enumerate(foot_hinges):
        case.visual(
            Box((0.005, 0.014, 0.009)),
            origin=Origin(xyz=(x - 0.0195, y, -0.0045)),
            material=case_mat,
            name=f"foot_mount_{foot_index}_a",
        )
        case.visual(
            Box((0.005, 0.014, 0.009)),
            origin=Origin(xyz=(x + 0.0195, y, -0.0045)),
            material=case_mat,
            name=f"foot_mount_{foot_index}_b",
        )

    key_mesh_cache: dict[float, object] = {}

    def _add_key(
        row_index: int,
        col_index: int,
        center_x: float,
        center_y: float,
        units: float = 1.0,
        material=key_mat,
    ):
        width = units * KEY_PITCH - KEY_GAP
        mesh_key = round(width, 5)
        key_mesh = key_mesh_cache.get(mesh_key)
        if key_mesh is None:
            key_mesh = _keycap_mesh(width, KEY_DEPTH, 0.0090, f"keycap_{mesh_key:.5f}")
            key_mesh_cache[mesh_key] = key_mesh
        key = model.part(f"key_{row_index}_{col_index}")
        key.visual(
            Box((min(0.0070, width * 0.36), 0.0065, 0.0060)),
            origin=Origin(xyz=(0.0, 0.0, -0.0030)),
            material=stem_mat,
            name="stem",
        )
        key.visual(
            key_mesh,
            material=material,
            name="keycap",
        )
        model.articulation(
            f"case_to_key_{row_index}_{col_index}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(center_x, center_y, KEY_JOINT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.4, velocity=0.10, lower=0.0, upper=0.0040),
        )
        return key

    def _add_row(row_index: int, y: float, layout: list[float | None], start_x: float = -0.172):
        x_cursor = start_x
        col_index = 0
        for units in layout:
            if units is None:
                x_cursor += 0.50 * KEY_PITCH
                continue
            center_x = x_cursor + units * KEY_PITCH * 0.5
            material = key_mat
            if row_index == 0 and col_index == 0:
                material = accent_key_mat
            if (row_index == 2 and col_index == 2) or (row_index == 3 and col_index in (1, 2, 3)):
                material = accent_key_mat
            _add_key(row_index, col_index, center_x, y, units, material)
            x_cursor += units * KEY_PITCH
            col_index += 1

    # Main tenkeyless block: 74 alphanumeric/function keys.
    _add_row(0, 0.052, [1.0, None, 1.0, 1.0, 1.0, 1.0, None, 1.0, 1.0, 1.0, 1.0, None, 1.0, 1.0, 1.0, 1.0])
    _add_row(1, 0.031, [1.0] * 13 + [2.0])
    _add_row(2, 0.012, [1.5] + [1.0] * 12 + [1.5])
    _add_row(3, -0.007, [1.75] + [1.0] * 11 + [2.25])
    _add_row(4, -0.026, [2.25] + [1.0] * 10 + [2.75])
    _add_row(5, -0.045, [1.25, 1.25, 1.25, 6.25, 1.25, 1.25, 1.25, 1.25])

    # Navigation island and inverted-T arrow cluster: 13 additional TKL keys.
    nav_xs = (0.132, 0.151, 0.170)
    for local_col, x in enumerate(nav_xs):
        _add_key(1, 14 + local_col, x, 0.031, 1.0)
        _add_key(2, 14 + local_col, x, 0.012, 1.0)
        _add_key(3, 13 + local_col, x, -0.007, 1.0)
    _add_key(4, 12, nav_xs[1], -0.026, 1.0, arrow_key_mat)
    for local_col, x in enumerate(nav_xs):
        _add_key(5, 8 + local_col, x, -0.045, 1.0, arrow_key_mat)

    roller = model.part("media_roller")
    roller.visual(
        Cylinder(radius=0.0072, length=0.042),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_mat,
        name="roller_body",
    )
    for rib_index, offset_x in enumerate((-0.015, -0.009, -0.003, 0.003, 0.009, 0.015)):
        roller.visual(
            Cylinder(radius=0.0079, length=0.0012),
            origin=Origin(xyz=(offset_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=roller_mat,
            name=f"roller_rib_{rib_index}",
        )
    model.articulation(
        "case_to_media_roller",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=roller,
        origin=Origin(xyz=(0.160, 0.061, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0),
    )

    for foot_index, (x, y, z) in enumerate(foot_hinges):
        foot = model.part(f"rear_foot_{foot_index}")
        foot.visual(
            Cylinder(radius=0.0040, length=0.034),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_mat,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.032, 0.046, 0.006)),
            origin=Origin(xyz=(0.0, -0.023, -0.007)),
            material=rubber_mat,
            name="folding_leg",
        )
        foot.visual(
            Box((0.034, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, -0.043, -0.009)),
            material=rubber_mat,
            name="rubber_tip",
        )
        model.articulation(
            f"case_to_rear_foot_{foot_index}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    key_joints = [joint for joint in object_model.articulations if joint.name.startswith("case_to_key_")]
    ctx.check("tenkeyless_key_count", len(key_parts) == 87, details=f"found {len(key_parts)} key parts")
    ctx.check("each_key_has_plunger", len(key_joints) == 87, details=f"found {len(key_joints)} key joints")

    sample_key = object_model.get_part("key_3_2")
    sample_joint = object_model.get_articulation("case_to_key_3_2")
    ctx.expect_contact(
        sample_key,
        "case",
        elem_a="stem",
        elem_b="top_plate",
        contact_tol=0.0005,
        name="sample key stem rests on top plate",
    )
    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: 0.0035}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "sample key plunges downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0030,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    roller_joint = object_model.get_articulation("case_to_media_roller")
    ctx.check(
        "roller_is_continuous",
        roller_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={roller_joint.articulation_type}",
    )
    roller_box = ctx.part_world_aabb("media_roller")
    ctx.check(
        "roller_at_upper_right",
        roller_box is not None and roller_box[0][0] > 0.135 and roller_box[0][1] > 0.052,
        details=f"aabb={roller_box}",
    )

    for foot_index in (0, 1):
        foot = object_model.get_part(f"rear_foot_{foot_index}")
        foot_joint = object_model.get_articulation(f"case_to_rear_foot_{foot_index}")
        ctx.expect_contact(
            foot,
            "case",
            elem_a="hinge_barrel",
            elem_b=f"foot_mount_{foot_index}_a",
            contact_tol=0.0006,
            name=f"rear foot {foot_index} barrel clipped in mount",
        )
        folded_box = ctx.part_world_aabb(foot)
        with ctx.pose({foot_joint: 1.0}):
            deployed_box = ctx.part_world_aabb(foot)
        ctx.check(
            f"rear foot {foot_index} folds down",
            folded_box is not None
            and deployed_box is not None
            and deployed_box[0][2] < folded_box[0][2] - 0.025,
            details=f"folded={folded_box}, deployed={deployed_box}",
        )

    return ctx.report()


object_model = build_object_model()
