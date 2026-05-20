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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


PANEL_LENGTH = 1.22
PANEL_WIDTH = 0.17
PANEL_HEIGHT = 0.045
PANEL_TOP_Z = -0.005
HINGE_DROP = 0.62
WIRE_TRAVEL = 0.16


def _build_loop_bail_mesh(name: str):
    loop_points = [
        (0.0, 0.0, 0.016),
        (-0.015, 0.0, 0.026),
        (-0.022, 0.0, 0.048),
        (0.0, 0.0, 0.070),
        (0.022, 0.0, 0.048),
        (0.015, 0.0, 0.026),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            loop_points,
            radius=0.0028,
            samples_per_segment=18,
            radial_segments=18,
            closed_spline=True,
            cap_ends=False,
            up_hint=(0.0, 1.0, 0.0),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suspended_linear_led_panel")

    powder_white = model.material("powder_white", rgba=(0.95, 0.96, 0.97, 1.0))
    matte_white = model.material("matte_white", rgba=(0.90, 0.91, 0.92, 1.0))
    opal_diffuser = model.material("opal_diffuser", rgba=(0.98, 0.99, 1.0, 0.72))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    charcoal = model.material("charcoal", rgba=(0.28, 0.29, 0.31, 1.0))

    anchor_ring_mesh = mesh_from_geometry(TorusGeometry(radius=0.0095, tube=0.0023), "anchor_ring")
    loop_bail_mesh = _build_loop_bail_mesh("loop_bail")

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Box((PANEL_LENGTH + 0.10, 0.060, 0.008)),
        origin=Origin(xyz=(PANEL_LENGTH * 0.5, 0.0, 0.024)),
        material=powder_white,
        name="ceiling_spine",
    )
    for side_name, x_pos in (("left", 0.0), ("right", PANEL_LENGTH)):
        ceiling_mount.visual(
            Box((0.090, 0.060, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, 0.012)),
            material=matte_white,
            name=f"{side_name}_canopy",
        )
        ceiling_mount.visual(
            anchor_ring_mesh,
            origin=Origin(xyz=(x_pos, 0.0, -0.009)),
            material=satin_aluminum,
            name=f"{side_name}_grip_ring",
        )
        ceiling_mount.visual(
            Cylinder(radius=0.0025, length=0.016),
            origin=Origin(xyz=(x_pos + 0.011, 0.0, -0.002)),
            material=satin_aluminum,
            name=f"{side_name}_ring_stem",
        )
        ceiling_mount.visual(
            Cylinder(radius=0.0045, length=0.024),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=charcoal,
            name=f"{side_name}_cable_grommet",
        )

    left_hanger = model.part("left_hanger")
    left_hanger.visual(
        Cylinder(radius=0.0016, length=0.498),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=stainless,
        name="wire",
    )
    left_hanger.visual(
        Cylinder(radius=0.0048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.531)),
        material=satin_aluminum,
        name="bottom_ferrule",
    )
    left_hanger.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=charcoal,
        name="top_grip",
    )

    left_loop = model.part("left_loop")
    left_loop.visual(
        Cylinder(radius=0.0050, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="pivot_barrel",
    )
    left_loop.visual(
        Cylinder(radius=0.0040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_aluminum,
        name="neck_stem",
    )
    left_loop.visual(
        loop_bail_mesh,
        material=satin_aluminum,
        name="bail_loop",
    )
    left_loop.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=satin_aluminum,
        name="attach_sleeve",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_LENGTH, PANEL_WIDTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(PANEL_LENGTH * 0.5, 0.0, PANEL_TOP_Z - PANEL_HEIGHT * 0.5)),
        material=powder_white,
        name="body_shell",
    )
    panel.visual(
        Box((PANEL_LENGTH - 0.070, 0.070, 0.012)),
        origin=Origin(xyz=(PANEL_LENGTH * 0.5, 0.0, -0.011)),
        material=matte_white,
        name="top_spine",
    )
    panel.visual(
        Box((PANEL_LENGTH - 0.060, PANEL_WIDTH - 0.050, 0.004)),
        origin=Origin(xyz=(PANEL_LENGTH * 0.5, 0.0, PANEL_TOP_Z - PANEL_HEIGHT + 0.002)),
        material=opal_diffuser,
        name="diffuser",
    )
    panel.visual(
        Box((0.024, 0.050, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.009)),
        material=matte_white,
        name="left_mount_pad",
    )
    panel.visual(
        Box((0.024, 0.050, 0.008)),
        origin=Origin(xyz=(PANEL_LENGTH - 0.010, 0.0, -0.009)),
        material=matte_white,
        name="right_mount_pad",
    )
    panel.visual(
        Box((0.010, PANEL_WIDTH, 0.038)),
        origin=Origin(xyz=(0.005, 0.0, -0.024)),
        material=matte_white,
        name="left_end_cap",
    )
    panel.visual(
        Box((0.010, PANEL_WIDTH, 0.038)),
        origin=Origin(xyz=(PANEL_LENGTH - 0.005, 0.0, -0.024)),
        material=matte_white,
        name="right_end_cap",
    )

    right_loop = model.part("right_loop")
    right_loop.visual(
        Cylinder(radius=0.0050, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="pivot_barrel",
    )
    right_loop.visual(
        Cylinder(radius=0.0040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_aluminum,
        name="neck_stem",
    )
    right_loop.visual(
        loop_bail_mesh,
        material=satin_aluminum,
        name="bail_loop",
    )
    right_loop.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=satin_aluminum,
        name="attach_sleeve",
    )

    right_hanger = model.part("right_hanger")
    right_hanger.visual(
        Cylinder(radius=0.0016, length=0.498),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=stainless,
        name="wire",
    )
    right_hanger.visual(
        Cylinder(radius=0.0048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.531)),
        material=satin_aluminum,
        name="bottom_ferrule",
    )
    right_hanger.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=charcoal,
        name="top_grip",
    )

    model.articulation(
        "left_wire_adjust",
        ArticulationType.PRISMATIC,
        parent=ceiling_mount,
        child=left_hanger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=WIRE_TRAVEL,
        ),
    )
    model.articulation(
        "left_hanger_to_loop",
        ArticulationType.FIXED,
        parent=left_hanger,
        child=left_loop,
        origin=Origin(xyz=(0.0, 0.0, -HINGE_DROP)),
    )
    model.articulation(
        "left_loop_pitch",
        ArticulationType.REVOLUTE,
        parent=left_loop,
        child=panel,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "panel_to_right_loop",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=right_loop,
        origin=Origin(xyz=(PANEL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "right_wire_adjust",
        ArticulationType.PRISMATIC,
        parent=ceiling_mount,
        child=right_hanger,
        origin=Origin(xyz=(PANEL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=WIRE_TRAVEL,
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

    ceiling_mount = object_model.get_part("ceiling_mount")
    left_hanger = object_model.get_part("left_hanger")
    left_loop = object_model.get_part("left_loop")
    panel = object_model.get_part("panel")
    right_loop = object_model.get_part("right_loop")
    right_hanger = object_model.get_part("right_hanger")

    left_wire_adjust = object_model.get_articulation("left_wire_adjust")
    left_loop_pitch = object_model.get_articulation("left_loop_pitch")
    right_loop_pitch = object_model.get_articulation("panel_to_right_loop")
    right_wire_adjust = object_model.get_articulation("right_wire_adjust")

    ctx.expect_contact(
        left_hanger,
        left_loop,
        elem_a="bottom_ferrule",
        elem_b="attach_sleeve",
        contact_tol=5e-4,
        name="left hanger ferrule seats on left loop bracket",
    )
    ctx.expect_contact(
        left_loop,
        panel,
        elem_a="pivot_barrel",
        elem_b="left_mount_pad",
        contact_tol=5e-4,
        name="left loop bracket is mounted on the panel end pad",
    )
    ctx.expect_contact(
        right_loop,
        panel,
        elem_a="pivot_barrel",
        elem_b="right_mount_pad",
        contact_tol=5e-4,
        name="right loop bracket is mounted on the panel end pad",
    )
    ctx.expect_contact(
        right_hanger,
        right_loop,
        elem_a="bottom_ferrule",
        elem_b="attach_sleeve",
        contact_tol=5e-4,
        name="right hanger ferrule meets the right loop bracket",
    )
    ctx.expect_gap(
        ceiling_mount,
        panel,
        axis="z",
        min_gap=0.56,
        max_gap=0.66,
        name="panel hangs well below the ceiling anchors",
    )

    left_rest = ctx.part_world_position(left_hanger)
    right_rest = ctx.part_world_position(right_hanger)
    with ctx.pose({left_wire_adjust: 0.12, right_wire_adjust: 0.10}):
        left_extended = ctx.part_world_position(left_hanger)
        right_extended = ctx.part_world_position(right_hanger)
    ctx.check(
        "left hanger wire extends downward from the ceiling grip",
        left_rest is not None
        and left_extended is not None
        and left_extended[2] < left_rest[2] - 0.10,
        details=f"rest={left_rest}, extended={left_extended}",
    )
    ctx.check(
        "right hanger wire extends downward from the ceiling grip",
        right_rest is not None
        and right_extended is not None
        and right_extended[2] < right_rest[2] - 0.08,
        details=f"rest={right_rest}, extended={right_extended}",
    )

    right_loop_rest_aabb = ctx.part_element_world_aabb(right_loop, elem="attach_sleeve")
    with ctx.pose({right_loop_pitch: 0.22}):
        right_loop_open_aabb = ctx.part_element_world_aabb(right_loop, elem="attach_sleeve")
    right_loop_rest_center_x = (
        (right_loop_rest_aabb[0][0] + right_loop_rest_aabb[1][0]) * 0.5
        if right_loop_rest_aabb is not None
        else None
    )
    right_loop_open_center_x = (
        (right_loop_open_aabb[0][0] + right_loop_open_aabb[1][0]) * 0.5
        if right_loop_open_aabb is not None
        else None
    )
    ctx.check(
        "right loop bracket swings outward about the panel hinge",
        right_loop_rest_center_x is not None
        and right_loop_open_center_x is not None
        and right_loop_open_center_x > right_loop_rest_center_x + 0.010,
        details=f"rest_x={right_loop_rest_center_x}, open_x={right_loop_open_center_x}",
    )

    right_loop_rest_pos = ctx.part_world_position(right_loop)
    with ctx.pose({left_loop_pitch: 0.12}):
        right_loop_tilted_pos = ctx.part_world_position(right_loop)
    ctx.check(
        "left loop hinge can pitch the panel so the far end rises",
        right_loop_rest_pos is not None
        and right_loop_tilted_pos is not None
        and right_loop_tilted_pos[2] > right_loop_rest_pos[2] + 0.10,
        details=f"rest={right_loop_rest_pos}, tilted={right_loop_tilted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
