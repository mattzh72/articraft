from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_W = 0.050
BODY_D = 0.018
BODY_H = 0.047
SLOT_FLOOR_Z = 0.036
TOP_SLOT_H = BODY_H - SLOT_FLOOR_Z

SHACKLE_ROD_R = 0.0032
SHACKLE_LEG_CENTER_X = 0.013
SHACKLE_SPAN = SHACKLE_LEG_CENTER_X * 2.0
SHACKLE_OPEN_LIMIT = 1.70

CYLINDER_RADIUS = 0.0058
CYLINDER_BEZEL_RADIUS = 0.0067
CYLINDER_LENGTH = 0.0025
CYLINDER_Z = 0.018
CYLINDER_TURN_LIMIT = 0.60
CYLINDER_WINDOW_W = 0.0124
CYLINDER_WINDOW_H = 0.0124

LAMINATION_DEPTHS = (0.0025, 0.0030, 0.0035, 0.0035, 0.0030, 0.0025)
LAMINATION_WIDTHS = (0.050, 0.049, 0.048, 0.048, 0.049, 0.050)
CYLINDER_BACKING_LAMINATION = "lamination_01_core"

TOP_CENTER_BRIDGE_W = 0.018
TOP_SLOT_W = 0.008
TOP_SHOULDER_W = (BODY_W - (2.0 * TOP_SLOT_W) - TOP_CENTER_BRIDGE_W) * 0.5


def _add_body_lamination(
    body_part,
    *,
    index: int,
    width: float,
    depth: float,
    y_center: float,
    front_window: bool,
    steel_material,
) -> None:
    prefix = f"lamination_{index:02d}"

    if front_window:
        side_w = (width - CYLINDER_WINDOW_W) * 0.5
        lower_h = CYLINDER_Z - (CYLINDER_WINDOW_H * 0.5)
        upper_h = SLOT_FLOOR_Z - (CYLINDER_Z + (CYLINDER_WINDOW_H * 0.5))

        body_part.visual(
            Box((side_w, depth, SLOT_FLOOR_Z)),
            origin=Origin(
                xyz=(
                    -(CYLINDER_WINDOW_W * 0.5 + side_w * 0.5),
                    y_center,
                    SLOT_FLOOR_Z * 0.5,
                )
            ),
            material=steel_material,
            name=f"{prefix}_left_frame",
        )
        body_part.visual(
            Box((side_w, depth, SLOT_FLOOR_Z)),
            origin=Origin(
                xyz=(
                    CYLINDER_WINDOW_W * 0.5 + side_w * 0.5,
                    y_center,
                    SLOT_FLOOR_Z * 0.5,
                )
            ),
            material=steel_material,
            name=f"{prefix}_right_frame",
        )
        body_part.visual(
            Box((CYLINDER_WINDOW_W, depth, lower_h)),
            origin=Origin(xyz=(0.0, y_center, lower_h * 0.5)),
            material=steel_material,
            name=f"{prefix}_bottom_band",
        )
        body_part.visual(
            Box((CYLINDER_WINDOW_W, depth, upper_h)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_center,
                    CYLINDER_Z + (CYLINDER_WINDOW_H * 0.5) + upper_h * 0.5,
                )
            ),
            material=steel_material,
            name=f"{prefix}_top_band",
        )
    else:
        body_part.visual(
            Box((width, depth, SLOT_FLOOR_Z)),
            origin=Origin(xyz=(0.0, y_center, SLOT_FLOOR_Z * 0.5)),
            material=steel_material,
            name=CYLINDER_BACKING_LAMINATION if index == 1 else f"{prefix}_core",
        )

    left_x = -(BODY_W * 0.5 - TOP_SHOULDER_W * 0.5)
    right_x = BODY_W * 0.5 - TOP_SHOULDER_W * 0.5

    body_part.visual(
        Box((TOP_SHOULDER_W, depth, TOP_SLOT_H)),
        origin=Origin(xyz=(left_x, y_center, SLOT_FLOOR_Z + TOP_SLOT_H * 0.5)),
        material=steel_material,
        name=f"{prefix}_left_top",
    )
    body_part.visual(
        Box((TOP_CENTER_BRIDGE_W, depth, TOP_SLOT_H)),
        origin=Origin(xyz=(0.0, y_center, SLOT_FLOOR_Z + TOP_SLOT_H * 0.5)),
        material=steel_material,
        name=f"{prefix}_center_top",
    )
    body_part.visual(
        Box((TOP_SHOULDER_W, depth, TOP_SLOT_H)),
        origin=Origin(xyz=(right_x, y_center, SLOT_FLOOR_Z + TOP_SLOT_H * 0.5)),
        material=steel_material,
        name=f"{prefix}_right_top",
    )


def _build_shackle_mesh():
    path = [
        (0.0, 0.0, -TOP_SLOT_H),
        (0.0, 0.0, 0.006),
        (0.0, 0.0, 0.021),
        (SHACKLE_SPAN * 0.22, 0.0, 0.031),
        (SHACKLE_SPAN * 0.50, 0.0, 0.036),
        (SHACKLE_SPAN * 0.78, 0.0, 0.031),
        (SHACKLE_SPAN, 0.0, 0.021),
        (SHACKLE_SPAN, 0.0, 0.006),
        (SHACKLE_SPAN, 0.0, -TOP_SLOT_H),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            path,
            radius=SHACKLE_ROD_R,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "padlock_shackle_rod",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laminated_padlock")

    bright_steel = model.material("bright_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    laminate_steel = model.material("laminate_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.63, 0.24, 1.0))
    keyway_dark = model.material("keyway_dark", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    running_front = -BODY_D * 0.5
    lamination_materials = (
        bright_steel,
        laminate_steel,
        shadow_steel,
        shadow_steel,
        laminate_steel,
        bright_steel,
    )
    for index, (depth, width, material) in enumerate(
        zip(LAMINATION_DEPTHS, LAMINATION_WIDTHS, lamination_materials)
    ):
        y_center = running_front + depth * 0.5
        _add_body_lamination(
            body,
            index=index,
            width=width,
            depth=depth,
            y_center=y_center,
            front_window=index == 0,
            steel_material=material,
        )
        running_front += depth

    body.visual(
        Box((0.042, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, BODY_D * 0.18, 0.003)),
        material=shadow_steel,
        name="body_bottom_reinforcement",
    )

    shackle = model.part("shackle")
    shackle.visual(
        _build_shackle_mesh(),
        material=hardened_steel,
        name="shackle_rod",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_ROD_R * 0.92, length=0.004),
        origin=Origin(xyz=(SHACKLE_SPAN, 0.0, -TOP_SLOT_H + 0.002)),
        material=shadow_steel,
        name="free_leg_tip",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_ROD_R * 1.02, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, -TOP_SLOT_H + 0.00225)),
        material=shadow_steel,
        name="retained_leg_tip",
    )

    cylinder_plug = model.part("cylinder_plug")
    cylinder_plug.visual(
        Cylinder(radius=CYLINDER_RADIUS, length=CYLINDER_LENGTH),
        origin=Origin(
            xyz=(0.0, CYLINDER_LENGTH * 0.5, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="plug_barrel",
    )
    cylinder_plug.visual(
        Cylinder(radius=CYLINDER_BEZEL_RADIUS, length=0.0012),
        origin=Origin(
            xyz=(0.0, -0.0006, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=bright_steel,
        name="plug_bezel",
    )
    cylinder_plug.visual(
        Box((0.0018, 0.0014, 0.0056)),
        origin=Origin(xyz=(0.0, -0.0005, -0.0002)),
        material=keyway_dark,
        name="keyway_slot",
    )
    cylinder_plug.visual(
        Box((0.0032, 0.0014, 0.0018)),
        origin=Origin(xyz=(0.0, -0.0005, -0.0032)),
        material=keyway_dark,
        name="keyway_ward",
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-SHACKLE_LEG_CENTER_X, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=SHACKLE_OPEN_LIMIT,
        ),
    )
    model.articulation(
        "body_to_cylinder",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cylinder_plug,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5, CYLINDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=-CYLINDER_TURN_LIMIT,
            upper=CYLINDER_TURN_LIMIT,
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
    cylinder_plug = object_model.get_part("cylinder_plug")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    cylinder_joint = object_model.get_articulation("body_to_cylinder")

    shackle_limits = shackle_joint.motion_limits
    cylinder_limits = cylinder_joint.motion_limits

    ctx.check(
        "shackle articulation is upward-opening hinge",
        shackle_joint.axis == (0.0, -1.0, 0.0)
        and shackle_limits is not None
        and shackle_limits.lower == 0.0
        and shackle_limits.upper is not None
        and shackle_limits.upper >= 1.5,
        details=f"axis={shackle_joint.axis}, limits={shackle_limits}",
    )
    ctx.check(
        "cylinder articulation is front-face rotation",
        cylinder_joint.axis == (0.0, 1.0, 0.0)
        and cylinder_limits is not None
        and cylinder_limits.lower is not None
        and cylinder_limits.upper is not None
        and cylinder_limits.lower < 0.0 < cylinder_limits.upper,
        details=f"axis={cylinder_joint.axis}, limits={cylinder_limits}",
    )

    with ctx.pose({shackle_joint: 0.0, cylinder_joint: 0.0}):
        ctx.expect_contact(
            shackle,
            body,
            name="closed shackle seats in the body",
        )
        ctx.expect_contact(
            cylinder_plug,
            body,
            elem_a="plug_barrel",
            elem_b=CYLINDER_BACKING_LAMINATION,
            name="plug barrel seats against the front body lamination",
        )
        ctx.expect_overlap(
            cylinder_plug,
            body,
            axes="xz",
            elem_a="plug_barrel",
            min_overlap=0.010,
            name="plug barrel stays centered in the front face",
        )
        rest_free_tip = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")
        rest_keyway = ctx.part_element_world_aabb(cylinder_plug, elem="keyway_slot")

    with ctx.pose({shackle_joint: SHACKLE_OPEN_LIMIT}):
        open_free_tip = ctx.part_element_world_aabb(shackle, elem="free_leg_tip")

    ctx.check(
        "shackle free leg rises when opened",
        rest_free_tip is not None
        and open_free_tip is not None
        and open_free_tip[0][2] > rest_free_tip[0][2] + 0.015,
        details=f"rest={rest_free_tip}, open={open_free_tip}",
    )

    with ctx.pose({cylinder_joint: CYLINDER_TURN_LIMIT}):
        turned_keyway = ctx.part_element_world_aabb(cylinder_plug, elem="keyway_slot")

    rest_x = None if rest_keyway is None else rest_keyway[1][0] - rest_keyway[0][0]
    rest_z = None if rest_keyway is None else rest_keyway[1][2] - rest_keyway[0][2]
    turned_x = None if turned_keyway is None else turned_keyway[1][0] - turned_keyway[0][0]
    turned_z = None if turned_keyway is None else turned_keyway[1][2] - turned_keyway[0][2]

    ctx.check(
        "keyway rotates with the cylinder plug",
        None not in (rest_x, rest_z, turned_x, turned_z)
        and turned_x > rest_x + 0.001
        and turned_x > rest_x * 2.0,
        details=(
            f"rest_x={rest_x}, rest_z={rest_z}, "
            f"turned_x={turned_x}, turned_z={turned_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
