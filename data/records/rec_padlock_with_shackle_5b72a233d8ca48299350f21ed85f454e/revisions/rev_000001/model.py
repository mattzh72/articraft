from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WirePath,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _merge_box(geom, size: tuple[float, float, float], center: tuple[float, float, float]) -> None:
    geom.merge(BoxGeometry(size).translate(*center))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_padlock")

    brass = model.material("brass", rgba=(0.77, 0.63, 0.24, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))

    body_width = 0.052
    body_depth = 0.026
    body_height = 0.065
    lower_body_height = 0.053
    collar_height = body_height - lower_body_height
    shackle_leg_x = 0.011
    shackle_hole_half = 0.005

    body = model.part("body")

    lower_shell = ExtrudeGeometry(
        rounded_rect_profile(body_width, body_depth, radius=0.006, corner_segments=8),
        lower_body_height,
    ).translate(0.0, 0.0, lower_body_height * 0.5)
    body.visual(
        mesh_from_geometry(lower_shell, "padlock_body_shell"),
        material=brass,
        name="body_shell",
    )

    collar_z = lower_body_height + collar_height * 0.5
    front_band_depth = (body_depth - 2.0 * shackle_hole_half) * 0.5
    outer_block_width = body_width * 0.5 - (shackle_leg_x + shackle_hole_half)
    center_bridge_width = 2.0 * (shackle_leg_x - shackle_hole_half)

    body.visual(
        Box((body_width, front_band_depth, collar_height)),
        origin=Origin(xyz=(0.0, -(body_depth - front_band_depth) * 0.5, collar_z)),
        material=brass,
        name="top_front_band",
    )
    body.visual(
        Box((body_width, front_band_depth, collar_height)),
        origin=Origin(xyz=(0.0, (body_depth - front_band_depth) * 0.5, collar_z)),
        material=brass,
        name="top_back_band",
    )
    body.visual(
        Box((outer_block_width, 2.0 * shackle_hole_half, collar_height)),
        origin=Origin(xyz=(-(body_width * 0.5 - outer_block_width * 0.5), 0.0, collar_z)),
        material=brass,
        name="left_outer_collar",
    )
    body.visual(
        Box((center_bridge_width, 2.0 * shackle_hole_half, collar_height)),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=brass,
        name="center_bridge",
    )
    body.visual(
        Box((outer_block_width, 2.0 * shackle_hole_half, collar_height)),
        origin=Origin(xyz=(body_width * 0.5 - outer_block_width * 0.5, 0.0, collar_z)),
        material=brass,
        name="right_outer_collar",
    )
    body.visual(
        Box((0.0060, 0.0060, 0.0020)),
        origin=Origin(xyz=(shackle_leg_x, 0.0, 0.053955)),
        material=brass,
        name="right_bore_seat",
    )
    body.visual(
        Box((0.0035, 0.018, 0.012)),
        origin=Origin(xyz=(-0.02425, 0.0, body_height + 0.006)),
        material=satin_steel,
        name="guard_pivot_lug",
    )
    body.visual(
        Box((0.044, 0.0024, 0.050)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5 + 0.0009), 0.028)),
        material=satin_steel,
        name="front_face_plate",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.0030),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5 + 0.0012), 0.0265), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    body.visual(
        Box((0.0020, 0.0034, 0.012)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5 + 0.0010), 0.0230)),
        material=dark_steel,
        name="key_slot",
    )
    body.visual(
        Box((0.0042, 0.0034, 0.0024)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5 + 0.0010), 0.0192)),
        material=dark_steel,
        name="key_slot_ward",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.055, 0.030, 0.078)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    shackle = model.part("shackle")
    shackle_path = (
        WirePath((-2.0 * shackle_leg_x, 0.0, 0.0))
        .line_to((-2.0 * shackle_leg_x, 0.0, 0.028))
        .bezier_to(
            (-2.0 * shackle_leg_x, 0.0, 0.049),
            (-0.008, 0.0, 0.056),
            (0.0, 0.0, 0.046),
            samples=18,
        )
        .line_to((0.0, 0.0, -0.010))
    )
    shackle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                shackle_path.to_points(),
                radius=0.0038,
                samples_per_segment=5,
                radial_segments=20,
                cap_ends=True,
            ),
            "padlock_shackle",
        ),
        material=satin_steel,
        name="shackle_rod",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.032, 0.010, 0.060)),
        mass=0.12,
        origin=Origin(xyz=(-0.011, 0.0, 0.020)),
    )

    guard = model.part("guard")
    guard.visual(
        Cylinder(radius=0.0032, length=0.016),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=satin_steel,
        name="guard_barrel",
    )
    guard.visual(
        Box((0.0030, 0.016, 0.018)),
        origin=Origin(xyz=(0.0015, 0.0, 0.012)),
        material=satin_steel,
        name="guard_arm",
    )
    guard.visual(
        Box((0.0040, 0.016, 0.006)),
        origin=Origin(xyz=(0.0020, 0.0, 0.023)),
        material=satin_steel,
        name="guard_tip",
    )
    guard.visual(
        Cylinder(radius=0.0018, length=0.016),
        origin=Origin(xyz=(0.0020, 0.0, 0.027), rpy=(pi * 0.5, 0.0, 0.0)),
        material=satin_steel,
        name="guard_nose",
    )
    guard.inertial = Inertial.from_geometry(
        Box((0.007, 0.018, 0.031)),
        mass=0.035,
        origin=Origin(xyz=(0.0020, 0.0, 0.0155)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(shackle_leg_x, 0.0, body_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(-0.0193, 0.0, body_height + 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
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

    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    guard = object_model.get_part("guard")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    guard_joint = object_model.get_articulation("body_to_guard")

    ctx.allow_overlap(
        body,
        shackle,
        elem_a="right_bore_seat",
        elem_b="shackle_rod",
        reason="The retained shackle leg is intentionally modeled as lightly seated on the bore floor to keep the shackle visibly captured.",
    )

    ctx.expect_contact(
        guard,
        body,
        elem_a="guard_barrel",
        elem_b="guard_pivot_lug",
        contact_tol=5e-4,
        name="guard barrel stays mounted against pivot lug",
    )

    face_plate = ctx.part_element_world_aabb(body, elem="front_face_plate")
    key_cylinder = ctx.part_element_world_aabb(body, elem="key_cylinder")
    key_slot = ctx.part_element_world_aabb(body, elem="key_slot")
    keyed_face_ok = False
    if face_plate and key_cylinder and key_slot:
        plate_min, plate_max = face_plate
        cylinder_min, cylinder_max = key_cylinder
        slot_min, slot_max = key_slot
        cylinder_center_x = 0.5 * (cylinder_min[0] + cylinder_max[0])
        slot_center_x = 0.5 * (slot_min[0] + slot_max[0])
        keyed_face_ok = (
            cylinder_min[1] < plate_min[1] + 0.0005
            and slot_min[1] < plate_min[1] + 0.0005
            and abs(cylinder_center_x) < 0.001
            and abs(slot_center_x) < 0.001
            and slot_max[2] < cylinder_max[2]
            and slot_min[2] < cylinder_min[2]
        )
    ctx.check(
        "body presents centered keyed front face",
        keyed_face_ok,
        details=f"face_plate={face_plate}, key_cylinder={key_cylinder}, key_slot={key_slot}",
    )

    shackle_rest = ctx.part_element_world_aabb(shackle, elem="shackle_rod")
    with ctx.pose({shackle_joint: 1.10}):
        shackle_open = ctx.part_element_world_aabb(shackle, elem="shackle_rod")
    shackle_opens = False
    if shackle_rest and shackle_open:
        rest_min, rest_max = shackle_rest
        open_min, open_max = shackle_open
        shackle_opens = (
            open_min[1] < rest_min[1] - 0.014
            and open_max[0] > rest_max[0] - 0.004
            and open_max[2] > 0.09
        )
    ctx.check(
        "shackle rotates forward on retained leg",
        shackle_opens,
        details=f"rest={shackle_rest}, open={shackle_open}",
    )

    guard_rest = ctx.part_element_world_aabb(guard, elem="guard_arm")
    with ctx.pose({guard_joint: 0.90}):
        guard_open = ctx.part_element_world_aabb(guard, elem="guard_arm")
    guard_swings = False
    if guard_rest and guard_open:
        rest_min, rest_max = guard_rest
        open_min, open_max = guard_open
        guard_swings = (
            open_min[0] < rest_min[0] - 0.010
            and open_max[2] < rest_max[2] - 0.003
        )
    ctx.check(
        "guard arm swings outward from shackle opening",
        guard_swings,
        details=f"rest={guard_rest}, open={guard_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
