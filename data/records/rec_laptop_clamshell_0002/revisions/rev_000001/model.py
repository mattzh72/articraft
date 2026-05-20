from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BASE_WIDTH = 0.310
BASE_DEPTH = 0.220
BASE_THICKNESS = 0.013

LID_WIDTH = 0.304
LID_DEPTH = 0.216
LID_THICKNESS = 0.006
LID_FRONT_SETBACK = 0.010

HINGE_AXIS_Y = 0.111
HINGE_AXIS_Z = 0.0185
REST_OPEN_ANGLE = math.radians(112.0)
MAX_OPEN_ANGLE = math.radians(135.0)
WIDE_OPEN_DELTA = -(MAX_OPEN_ANGLE - REST_OPEN_ANGLE)


def _rotated_yz(y: float, z: float, angle: float) -> tuple[float, float]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (y * cos_a) - (z * sin_a), (y * sin_a) + (z * cos_a)


def _lid_origin(x: float, y: float, z: float) -> Origin:
    ry, rz = _rotated_yz(y, z, -REST_OPEN_ANGLE)
    return Origin(xyz=(x, ry, rz), rpy=(-REST_OPEN_ANGLE, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thin_laptop", assets=ASSETS)

    body_aluminum = model.material("body_aluminum", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_keys = model.material("dark_keys", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.29, 0.32, 1.0))
    glass = model.material("display_glass", rgba=(0.06, 0.08, 0.10, 0.92))
    bezel = model.material("bezel_black", rgba=(0.05, 0.05, 0.06, 1.0))
    touchpad_tint = model.material("touchpad_tint", rgba=(0.52, 0.54, 0.57, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=body_aluminum,
        name="lower_shell",
    )
    base.visual(
        Box((BASE_WIDTH - 0.008, BASE_DEPTH - 0.010, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS - 0.0008)),
        material=body_aluminum,
        name="deck_surface",
    )
    base.visual(
        Box((0.248, 0.086, 0.0010)),
        origin=Origin(xyz=(0.0, 0.022, BASE_THICKNESS - 0.0009)),
        material=graphite,
        name="keyboard_field",
    )
    base.visual(
        Box((0.110, 0.072, 0.0009)),
        origin=Origin(xyz=(0.0, -0.056, BASE_THICKNESS - 0.00085)),
        material=touchpad_tint,
        name="touchpad",
    )

    key_rows = (
        (0.052, 11, 0.0170),
        (0.036, 11, 0.0170),
        (0.020, 10, 0.0180),
        (0.004, 10, 0.0180),
    )
    key_height = 0.0014
    key_depth = 0.012
    key_z = BASE_THICKNESS - 0.0001 + (key_height * 0.5)
    for row_index, (row_y, count, key_width) in enumerate(key_rows):
        pitch = 0.021
        offset = -0.5 * pitch * (count - 1)
        for column_index in range(count):
            base.visual(
                Box((key_width, key_depth, key_height)),
                origin=Origin(
                    xyz=(
                        offset + (column_index * pitch),
                        row_y,
                        key_z,
                    )
                ),
                material=dark_keys,
                name=f"key_{row_index}_{column_index}",
            )

    bottom_row_y = -0.013
    bottom_key_specs = (
        (-0.103, 0.020),
        (-0.079, 0.020),
        (-0.055, 0.020),
        (0.000, 0.092),
        (0.055, 0.020),
        (0.079, 0.020),
        (0.103, 0.020),
    )
    for column_index, (center_x, width) in enumerate(bottom_key_specs):
        base.visual(
            Box((width, key_depth, key_height)),
            origin=Origin(xyz=(center_x, bottom_row_y, key_z)),
            material=dark_keys,
            name=f"bottom_key_{column_index}",
        )

    for side_name, center_x, base_barrel_x, lid_barrel_x in (
        ("left", -0.096, -0.1025, -0.0890),
        ("right", 0.096, 0.1025, 0.0890),
    ):
        base.visual(
            Box((0.022, 0.014, 0.010)),
            origin=Origin(xyz=(center_x, HINGE_AXIS_Y - 0.006, 0.009)),
            material=graphite,
            name=f"{side_name}_hinge_pedestal",
        )
        base.visual(
            Cylinder(radius=0.0045, length=0.015),
            origin=Origin(
                xyz=(base_barrel_x, HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=graphite,
            name=f"{side_name}_base_hinge_barrel",
        )

    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
    )

    lid = model.part("lid")
    lid_shell_center_y = -(LID_DEPTH * 0.5) - LID_FRONT_SETBACK
    display_center_y = -0.114
    display_width = 0.282
    display_height = 0.176
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        origin=_lid_origin(0.0, lid_shell_center_y, 0.0),
        material=body_aluminum,
        name="lid_shell",
    )
    lid.visual(
        Box((display_width, display_height, 0.0010)),
        origin=_lid_origin(0.0, display_center_y, -0.0015),
        material=glass,
        name="display_glass",
    )
    lid.visual(
        Box((0.010, 0.192, 0.0012)),
        origin=_lid_origin(-0.146, display_center_y, -0.0014),
        material=bezel,
        name="left_bezel",
    )
    lid.visual(
        Box((0.010, 0.192, 0.0012)),
        origin=_lid_origin(0.146, display_center_y, -0.0014),
        material=bezel,
        name="right_bezel",
    )
    lid.visual(
        Box((0.286, 0.018, 0.0012)),
        origin=_lid_origin(0.0, -0.205, -0.0014),
        material=bezel,
        name="top_bezel",
    )
    lid.visual(
        Box((0.286, 0.020, 0.0012)),
        origin=_lid_origin(0.0, -0.020, -0.0014),
        material=bezel,
        name="bottom_bezel",
    )
    for side_name, lid_barrel_x, lid_leaf_x in (
        ("left", -0.0890, -0.0835),
        ("right", 0.0890, 0.0835),
    ):
        lid.visual(
            Box((0.016, 0.012, 0.007)),
            origin=_lid_origin(lid_leaf_x, -0.005, 0.0),
            material=graphite,
            name=f"{side_name}_lid_hinge_leaf",
        )
        lid.visual(
            Cylinder(radius=0.0041, length=0.012),
            origin=Origin(
                xyz=(lid_barrel_x, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=graphite,
            name=f"{side_name}_lid_hinge_barrel",
        )

    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=0.82,
        origin=_lid_origin(0.0, lid_shell_center_y, 0.0),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=WIDE_OPEN_DELTA,
            upper=REST_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")

    deck_surface = base.get_visual("deck_surface")
    keyboard_field = base.get_visual("keyboard_field")
    touchpad = base.get_visual("touchpad")
    left_base_hinge = base.get_visual("left_base_hinge_barrel")
    right_base_hinge = base.get_visual("right_base_hinge_barrel")

    lid_shell = lid.get_visual("lid_shell")
    display_glass = lid.get_visual("display_glass")
    left_lid_hinge = lid.get_visual("left_lid_hinge_barrel")
    right_lid_hinge = lid.get_visual("right_lid_hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=keyboard_field,
        outer_elem=deck_surface,
        name="keyboard field stays within the deck",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=touchpad,
        outer_elem=deck_surface,
        name="touchpad stays centered on the deck",
    )
    ctx.expect_within(
        lid,
        lid,
        axes="xy",
        inner_elem=display_glass,
        outer_elem=lid_shell,
        name="display stays within lid footprint",
    )
    ctx.expect_origin_distance(
        lid,
        base,
        axes="x",
        max_dist=1e-6,
        name="lid hinge stays centered over the base",
    )
    ctx.expect_overlap(
        base,
        lid,
        axes="x",
        min_overlap=0.30,
        elem_a=deck_surface,
        elem_b=lid_shell,
        name="open lid remains centered across laptop width",
    )
    ctx.expect_contact(
        base,
        lid,
        elem_a=left_base_hinge,
        elem_b=left_lid_hinge,
        contact_tol=5e-4,
        name="left hinge knuckles stay seated",
    )
    ctx.expect_contact(
        base,
        lid,
        elem_a=right_base_hinge,
        elem_b=right_lid_hinge,
        contact_tol=5e-4,
        name="right hinge knuckles stay seated",
    )

    open_display_aabb = ctx.part_element_world_aabb(lid, elem=display_glass)
    open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "open display bounds resolve",
        open_display_aabb is not None,
        details="display_glass world AABB should be measurable in the default open pose",
    )
    if open_display_aabb is not None:
        ctx.check(
            "open display rises above the base",
            open_display_aabb[1][2] > 0.19,
            details=f"expected display top above 0.19 m, got {open_display_aabb[1][2]:.4f} m",
        )
        ctx.check(
            "open display sits behind the rear edge",
            open_display_aabb[0][1] > 0.11,
            details=f"expected display minimum y above 0.11 m, got {open_display_aabb[0][1]:.4f} m",
        )
    ctx.check(
        "open lid bounds resolve",
        open_lid_aabb is not None,
        details="lid world AABB should be measurable in the default open pose",
    )

    with ctx.pose({lid_hinge: REST_OPEN_ANGLE}):
        ctx.expect_overlap(
            base,
            lid,
            axes="xy",
            min_overlap=0.20,
            elem_a=deck_surface,
            elem_b=lid_shell,
            name="closed lid covers the keyboard deck",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.0025,
            max_gap=0.0065,
            positive_elem=display_glass,
            negative_elem=keyboard_field,
            name="closed display clears the keyboard field",
        )
        ctx.expect_contact(
            base,
            lid,
            elem_a=left_base_hinge,
            elem_b=left_lid_hinge,
            contact_tol=5e-4,
            name="left hinge stays mated when closed",
        )
        ctx.expect_contact(
            base,
            lid,
            elem_a=right_base_hinge,
            elem_b=right_lid_hinge,
            contact_tol=5e-4,
            name="right hinge stays mated when closed",
        )
        closed_lid_position = ctx.part_world_position(lid)
        closed_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "closed lid origin stays at the hinge height",
            closed_lid_position is not None and closed_lid_position[2] < 0.020,
            details=(
                "lid world origin should return close to the hinge axis height when closed"
                if closed_lid_position is None
                else f"expected z < 0.020 m, got {closed_lid_position[2]:.4f} m"
            ),
        )
        ctx.check(
            "closed lid stays low over the base",
            closed_lid_aabb is not None and closed_lid_aabb[1][2] < 0.10,
            details=(
                "closed lid AABB should be measurable and remain near the base"
                if closed_lid_aabb is None
                else f"expected lid max z < 0.10 m, got {closed_lid_aabb[1][2]:.4f} m"
            ),
        )

    with ctx.pose({lid_hinge: WIDE_OPEN_DELTA}):
        ctx.expect_contact(
            base,
            lid,
            elem_a=left_base_hinge,
            elem_b=left_lid_hinge,
            contact_tol=5e-4,
            name="left hinge stays mated at wide-open pose",
        )
        ctx.expect_contact(
            base,
            lid,
            elem_a=right_base_hinge,
            elem_b=right_lid_hinge,
            contact_tol=5e-4,
            name="right hinge stays mated at wide-open pose",
        )
        wide_open_display_aabb = ctx.part_element_world_aabb(lid, elem=display_glass)
        ctx.check(
            "wide-open display bounds resolve",
            wide_open_display_aabb is not None,
            details="display_glass world AABB should be measurable at the wide-open pose",
        )
        if open_display_aabb is not None and wide_open_display_aabb is not None:
            ctx.check(
                "wide-open lid swings farther rearward",
                wide_open_display_aabb[1][1] > open_display_aabb[1][1] + 0.05,
                details=(
                    "expected display max y to move rearward by at least 0.05 m; "
                    f"rest={open_display_aabb[1][1]:.4f} m, wide-open={wide_open_display_aabb[1][1]:.4f} m"
                ),
            )
            ctx.check(
                "wide-open lid remains visibly raised",
                wide_open_display_aabb[1][2] > 0.17,
                details=f"expected display top above 0.17 m at wide-open pose, got {wide_open_display_aabb[1][2]:.4f} m",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
